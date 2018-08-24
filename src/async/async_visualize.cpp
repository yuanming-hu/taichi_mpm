/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#include "async_mpm.h"

#include <Partio.h>

TC_NAMESPACE_BEGIN

template <int dim>
void AsyncMPM<dim>::visualize() const {
  ++(const_cast<AsyncMPM<dim> *>(this)->frame_count);

  // output_limits
  for (uint64 offset = 0; offset < scheduler_size; ++offset)
    for (auto &container : particle_pool[offset]) {
      // This part may be removed to accelerate
      Particle *p = const_cast<Particle *>(
          reinterpret_cast<const Particle *>(&container));
      p->dt_limit = int(blocks[offset].continuous_dt_limit);
      p->stiffness_limit = int(blocks[offset].strength_dt_limit);
      p->cfl_limit = int(blocks[offset].cfl_dt_limit);
    }

  auto visualize_state = const_cast<int64 *>(visualize_state_address);

  static int64 visualize_int = 1;
  while (true) {
    if (visualize_int < min_delta_t_int || visualize_int > max_delta_t_int) {
      visualize_int = min_delta_t_int;
    } else {
      visualize_int = visualize_int * 2;
      if (visualize_int > max_delta_t_int) {
        visualize_int = min_delta_t_int;
      }
    }
    bool flag = false;
    for (uint64 offset = 0; offset < scheduler_size; ++offset) {
      if (blocks[offset].continuous_dt_limit == visualize_int) {
        visualize_state[offset] = 2;
        flag = true;
      } else {
        visualize_state[offset] = 0;
      }
    }
    for (uint64 offset = 0; offset < scheduler_size; ++offset)
      if (blocks[offset].continuous_dt_limit == visualize_int)
        for (auto nearby_offset : cached_neighbours[offset])
          if (blocks[nearby_offset].continuous_dt_limit != visualize_int) {
            visualize_state[nearby_offset] = 1;
          }
    if (flag) {
      break;
    }
  }

  std::string directory = this->config_backup.get_string("frame_directory");
  std::string file_name =
      fmt::format("{}/{:04}.bgeo", directory, this->frame_count);

  Partio::ParticlesDataMutable *parts = Partio::create();
  Partio::ParticleAttribute posH, vH, mH, typeH, normH, statH, boundH, distH,
      debugH, indexH, limitH, apicH;

  bool verbose = this->config_backup.get("verbose_bgeo", false);

  posH = parts->addAttribute("position", Partio::VECTOR, 3);
  typeH = parts->addAttribute("type", Partio::INT, 1);
  indexH = parts->addAttribute("index", Partio::INT, 1);
  limitH = parts->addAttribute("limit", Partio::INT, 4);
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

  std::vector<Container> particles_sorted;
  for (uint64 offset = 0; offset < scheduler_size; ++offset)
    for (auto container : particle_pool[offset])
      particles_sorted.push_back(container);
  std::sort(particles_sorted.begin(), particles_sorted.end(),
            [&](Container a, Container b) {
              Particle *p_a = reinterpret_cast<Particle *>(&a);
              Particle *p_b = reinterpret_cast<Particle *>(&b);
              return p_a->id < p_b->id;
            });

  for (auto container : particles_sorted) {
    Particle *p = reinterpret_cast<Particle *>(&container);
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
      dist_p[0] = p->boundary_distance * this->inv_delta_x;
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

    uint64 offset = SparseMask::Linear_Offset(
        MPM<dim>::get_grid_base_pos(pos * this->inv_delta_x));
    uint64 sim_offset =
        (offset >> SparseMask::data_bits >> SparseMask::block_bits) &
        scheduler_mask;

    limit_p[3] = visualize_state[sim_offset];
  }

  Partio::write(file_name.c_str(), *parts);
  parts->release();

  return;
}

template void AsyncMPM<2>::visualize() const;
template void AsyncMPM<3>::visualize() const;

TC_NAMESPACE_END
