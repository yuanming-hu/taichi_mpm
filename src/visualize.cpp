/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#include <taichi/system/threading.h>
#include <taichi/visual/texture.h>
#include <taichi/system/profiler.h>
#include <taichi/visual/scene.h>
#include <Partio.h>

#include "mpm.h"

TC_NAMESPACE_BEGIN

template <int dim>
void MPM<dim>::write_partio(const std::string &file_name) const {
  Partio::ParticlesDataMutable *parts = Partio::create();
  Partio::ParticleAttribute posH, vH, mH, typeH, normH, statH, boundH, distH,
      debugH, indexH, limitH, apicH;

  bool verbose = config_backup.get("verbose_bgeo", false);

  posH = parts->addAttribute("position", Partio::VECTOR, 3);
  typeH = parts->addAttribute("type", Partio::INT, 1);
  indexH = parts->addAttribute("index", Partio::INT, 1);
  limitH = parts->addAttribute("limit", Partio::INT, 3);
  vH = parts->addAttribute("v", Partio::VECTOR, 3);

  if (verbose) {
    mH = parts->addAttribute("m", Partio::VECTOR, 1);
    normH = parts->addAttribute("boundary_normal", Partio::VECTOR, 3);
    debugH = parts->addAttribute("debug", Partio::VECTOR, 3);
    statH = parts->addAttribute("states", Partio::INT, 1);
    distH = parts->addAttribute("boundary_distance", Partio::FLOAT, 1);
    boundH = parts->addAttribute("near_boundary", Partio::INT, 1);
    apicH = parts->addAttribute("apic_frobenius_norm", Partio::FLOAT, 1);
  }
  auto particles_sorted = particles;
  std::sort(particles_sorted.begin(), particles_sorted.end(),
            [&](ParticlePtr a, ParticlePtr b) {
              return allocator.get_const(a)->id < allocator.get_const(b)->id;
            });
  for (auto p_i : particles_sorted) {
    const Particle *p = allocator.get_const(p_i);
    int idx = parts->addParticle();
    if (verbose) {
      float32 *m_p = parts->dataWrite<float32>(mH, idx);
      float32 *bn_p = parts->dataWrite<float32>(normH, idx);
      float32 *debug_p = parts->dataWrite<float32>(debugH, idx);
      int *st_p = parts->dataWrite<int>(statH, idx);
      int *nb_p = parts->dataWrite<int>(boundH, idx);
      float32 *dist_p = parts->dataWrite<float32>(distH, idx);
      float32 *apic_p = parts->dataWrite<float32>(apicH, idx);
      real mass = p->get_mass();
      Vector norm = p->boundary_normal;
      m_p[0] = mass;
      for (int k = 0; k < 3; k++)
        bn_p[k] = 0.f;
      for (int k = 0; k < 3; k++)
        debug_p[k] = 0.f;
      for (int k = 0; k < dim; k++)
        bn_p[k] = norm[k];
      for (int k = 0; k < 3; k++)
        debug_p[k] = p->get_debug_info()[k];

      st_p[0] = p->states;
      nb_p[0] = p->near_boundary();
      dist_p[0] = p->boundary_distance * inv_delta_x;
      apic_p[0] =
          (0.5_f * (p->apic_b - p->apic_b.transposed())).frobenius_norm();
    }

    Vector vel = p->get_velocity();
    float32 *v_p = parts->dataWrite<float32>(vH, idx);
    for (int k = 0; k < 3; k++)
      v_p[k] = 0.f;
    for (int k = 0; k < dim; k++)
      v_p[k] = vel[k];
    int *type_p = parts->dataWrite<int>(typeH, idx);
    int *index_p = parts->dataWrite<int>(indexH, idx);
    int *limit_p = parts->dataWrite<int>(limitH, idx);
    float32 *p_p = parts->dataWrite<float32>(posH, idx);

    Vector pos = p->pos;

    for (int k = 0; k < 3; k++)
      p_p[k] = 0.f;

    for (int k = 0; k < dim; k++)
      p_p[k] = pos[k];
    type_p[0] = int(p->is_rigid());
    index_p[0] = p->id;
    limit_p[0] = p->dt_limit;
    limit_p[1] = p->stiffness_limit;
    limit_p[2] = p->cfl_limit;
  }
  Partio::write(file_name.c_str(), *parts);
  parts->release();
}

template <int dim>
void MPM<dim>::write_rigid_body(RigidBody<dim> const *rigid,
                                const std::string &file_name) const {
  TC_STATIC_IF(dim == 2) {
    auto &mesh = rigid->mesh;
    auto const &trans = rigid->get_mesh_to_world();

    std::string fn = file_name + std::string(".poly");
    FILE *f = fopen(fn.c_str(), "w");

    fmt::print(f, "POINTS\n");

    int index = 0;
    for (auto &elem : mesh->elements) {
      for (int i = 0; i < dim; i++) {
        Vector v = transform(trans, elem.v[i]);
        fmt::print(f, "{}: {} {} 0.0\n", ++index, v.x, v.y);
      }
    }

    fmt::print(f, "POLYS\n");

    for (int i = 1; i <= index / 2; ++i) {
      fmt::print(f, "{}: {} {}\n", i, i * 2 - 1, i * 2);
    }

    fmt::print(f, "END\n");

    fclose(f);
  }
  TC_STATIC_ELSE {
    auto &mesh = rigid->mesh;
    auto const &trans = rigid->get_mesh_to_world();

    std::string fn = file_name + std::string(".obj");
    FILE *f = std::fopen(fn.c_str(), "w");
    for (auto &elem : mesh->elements) {
      for (int i = 0; i < dim; i++) {
        Vector v = transform(trans, elem.v[i]);
        fmt::print(f, "v {} {} {}\n", v[0], v[1], v[2]);
      }
    }
    int counter = 0;
    for (auto &_ : mesh->elements) {
      fmt::print(f, "f {} {} {}\n", dim * counter + 1, dim * counter + 2,
                 dim * counter + 3);
      counter++;
      trash(_);
    }
    std::fclose(f);
  }
  TC_STATIC_END_IF
}

template <>
void MPM<3>::visualize() const {
  write_bgeo();
}

template <>
void MPM<2>::visualize() const {
  write_bgeo();
}

template void MPM<2>::write_partio(const std::string &file_name) const;
template void MPM<3>::write_partio(const std::string &file_name) const;

template void MPM<2>::write_rigid_body(RigidBody<2> const *rigid,
                                       const std::string &file_name) const;
template void MPM<3>::write_rigid_body(RigidBody<3> const *rigid,
                                       const std::string &file_name) const;

TC_NAMESPACE_END
