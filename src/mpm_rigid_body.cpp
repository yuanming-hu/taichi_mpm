/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#include <taichi/geometry/mesh.h>
#include <taichi/visualization/pakua.h>
#include "mpm.h"
#include "particles.h"
#include "boundary_particle.h"
#include "rigid_body_solver.h"
#include <taichi/system/profiler.h>

TC_NAMESPACE_BEGIN

void check_scripting_parameters(const Config &config) {
  TC_ASSERT_INFO(!config.has_key("scripted"),
                 "'scripted' is deprecated. Please remove.");
  TC_ASSERT_INFO(!config.has_key("position"),
                 "Use 'initial_position' instead of 'position'.")
  TC_ASSERT_INFO(!config.has_key("rotation"),
                 "Use 'initial_rotation' instead of 'rotation'.")

  if (config.has_key("scripted_position")) {
    TC_ASSERT_INFO(!config.has_key("initial_position"),
                   "scripted_position and initial_position cannot coexist.");
    TC_ASSERT_INFO(!config.has_key("initial_velocity"),
                   "scripted_position and initial_velocity cannot coexist.");
  } else {
    TC_ASSERT_INFO(config.has_key("initial_position"),
                   "Please specify one (and only one) of 'scripted_position' "
                   "and 'initial_position'.");
  }

  if (config.has_key("scripted_rotation")) {
    TC_ASSERT_INFO(!config.has_key("initial_rotation"),
                   "scripted_rotation and initial_rotation cannot coexist!");
    TC_ASSERT_INFO(
        !config.has_key("initial_angular_velocity"),
        "scripted_rotation and initial_angular_velocity cannot coexist!");
  }
  if (config.has_key("friction")) {
    TC_ASSERT_INFO(!config.has_key("friction0"),
                   "friction and friction0 cannot coexist!");
    TC_ASSERT_INFO(!config.has_key("friction1"),
                   "friction and friction1 cannot coexist!");
  }
  if (config.has_key("friction0") != config.has_key("friction1")) {
    TC_ERROR("friction0 and friction1 must be specified simultaneuously.");
  }

  if (config.has_key("friction0")) {
    TC_ASSERT_INFO(!config.has_key("friction"),
                   "friction0 and friction cannot coexist.");
  }
}

template <int dim>
std::unique_ptr<RigidBody<dim>> MPM<dim>::create_rigid_body(Config config) {
  std::unique_ptr<RigidBody<dim>> rigid_ptr =
      std::make_unique<RigidBody<dim>>();
  auto &rigid = *rigid_ptr;
  rigid.velocity = config.get("initial_velocity", Vector(0.0f));
  rigid.angular_velocity =
      config.get("initial_angular_velocity",
                 typename AngularVelocity<dim>::ValueType(0.0_f));
  check_scripting_parameters(config);
  if (config.has_key("friction")) {
    rigid.frictions[0] = rigid.frictions[1] = config.get("friction", 0.0f);
  } else {
    rigid.frictions[0] = config.get("friction0", 0.0f);
    rigid.frictions[1] = config.get("friction1", 0.0f);
  }
  rigid.restitution = config.get("restitution", 0.0f);
  rigid.codimensional = config.get<bool>("codimensional");
  rigid.color = config.get("color", Vector3(0.5_f));
  rigid.linear_damping = config.get("linear_damping", 0.0f);
  rigid.angular_damping = config.get("angular_damping", 0.0f);

  typename RigidBody<dim>::PositionFunctionType *f =
      (typename RigidBody<dim>::PositionFunctionType *)config.get(
          "scripted_position", (uint64)0);
  typename RigidBody<dim>::RotationFunctionType *g =
      (typename RigidBody<dim>::RotationFunctionType *)config.get(
          "scripted_rotation", (uint64)0);
  if (f) {
    rigid.pos_func = *f;
    rigid.pos_func_id = config.get<int>("scripted_position_id");
  }
  if (g) {
    rigid.rot_func = *g;
    rigid.rot_func_id = config.get<int>("scripted_rotation_id");
  }

  MatrixP mesh_to_world(1.0_f);

  Vector initial_position;
  if (f) {
    initial_position = (*f)(this->current_t);
  } else {
    initial_position = config.get<Vector>("initial_position");
  }
  rigid.position = initial_position;

  TC_STATIC_IF(dim == 2) {
    real angle = 0;
    if (g) {
      angle = (*g)(this->current_t);
    } else {
      angle = config.get("initial_rotation", 0.0_f);
    }
    rigid.rotation = Rotation<2>(radians(angle));
  }
  TC_STATIC_ELSE {
    Vector euler;
    if (rigid.rot_func) {
      euler = rigid.rot_func(this->current_t);
    } else {
      euler = config.get("initial_rotation", Vector(0.0_f));
    }
    euler = radians(euler);
    Eigen::Quaternion<real> q =
        Eigen::AngleAxis<real>(euler[0], Eigen::Matrix<real, 3, 1>::UnitX()) *
        Eigen::AngleAxis<real>(euler[1], Eigen::Matrix<real, 3, 1>::UnitY()) *
        Eigen::AngleAxis<real>(euler[2], Eigen::Matrix<real, 3, 1>::UnitZ());
    rigid.rotation.value = q;
  }
  TC_STATIC_END_IF
  if (dim == 3) {
    rigid.rotation_axis = config.get<Vector>("rotation_axis", Vector(0.0_f));
  }
  return rigid_ptr;
}

template <int dim>
void MPM<dim>::add_rigid_particle(Config config) {
  auto rigid_ptr = create_rigid_body(config);
  auto &rigid = *rigid_ptr;

  // This config is for particles
  Config config_new = config;
  config_new.set("rigid", &rigid);

  real density;
  if (config.get<bool>("codimensional")) {
    density = config.get("density", 40.0f);
  } else {
    density = config.get("density", 400.0f);
  }

  std::vector<ParticlePtr> added_particles;

  auto add_boundry_particle = [&](
      Vector position, Vector normal,
      typename RigidBody<dim>::ElementType *untransformed) {

    config_new.set("normal", normal)
        .set("rigid", &rigid)
        .set("offset", position - rigid.position);

    auto alloc = allocator.allocate_particle("rigid_boundary");

    RigidBoundaryParticle<dim> *p =
        static_cast<RigidBoundaryParticle<dim> *>(alloc.second);
    p->pos = position;
    p->initialize(config_new);
    p->untransformed_element = *untransformed;
    added_particles.push_back(alloc.first);
  };

  rigid.mesh = std::make_unique<typename RigidBody<dim>::MeshType>();

  auto &mesh = rigid.mesh;
  mesh->initialize(config);

  Vector center_of_mass;
  // Initialize position and rotation

  Vector scale = config.get<Vector>("scale", Vector(1.0_f));

  Vector initial_position = rigid.position;

  // First, scale to make sure we have the correct mass and inertia
  auto &elements = rigid.mesh->elements;
  for (auto &elem : elements) {
    for (int k = 0; k < dim; k++) {
      elem.v[k] = scale * elem.v[k];
    }
  }

  // Second, compute center of mass
  center_of_mass = rigid.initialize_mass_and_inertia(density);
  if (!config.get("recenter", true)) {
    TC_ASSERT(rigid.pos_func);
    TC_ASSERT(rigid.rot_func);
    center_of_mass = Vector(0);
  }

  if (rigid.pos_func) {
    rigid.set_infinity_mass();
  }
  if (rigid.rot_func) {
    rigid.set_infinity_inertia();
  }

  // Third, translate to make sure the mesh has its center of mass at the origin
  for (auto &elem : elements) {
    for (int k = 0; k < dim; k++) {
      elem.v[k] = elem.v[k] - center_of_mass;
    }
  }

  // First set rigid centoid to be origin, so that particles get correct offsets
  rigid.position = Vector(0.0_f);

  TC_STATIC_IF(dim == 2) {
    for (auto &elem : elements) {
      Vector a = elem.v[0], b = elem.v[1];
      int n_samples =
          std::max((int)std::ceil((a - b).length() * inv_delta_x), 2);
      for (int j = 0; j < n_samples; j++) {
        Vector pos = lerp((0.5_f + j) / n_samples, a, b);
        // NOTE: no "corner" boundary particle for 2D
        add_boundry_particle(pos, elem.get_normal(), &elem);
      }
    }
  }
  TC_STATIC_ELSE {
    // Rotation axis restriction (3D only)
    rigid.rotation_axis = config.get("rotation_axis", Vector(0.0_f));

    for (auto &elem : elements) {
      std::vector<Vector> positions;
      Vector x_n = normalize(elem.v[1] - elem.v[0]);
      Vector y_n = normalize(elem.v[2] - elem.v[0]);
      real x_length = length(elem.v[1] - elem.v[0]);
      real y_length = length(elem.v[2] - elem.v[0]);
      for (real _x = min(x_length / 3.0_f, this->delta_x / 2.0_f);
           _x < x_length + this->delta_x; _x += this->delta_x)
        for (real _y = min(y_length / 3.0_f, this->delta_x / 2.0_f);
             _y < y_length + this->delta_x; _y += this->delta_x) {
          real x = ((_x < x_length) ? _x : _x - this->delta_x / 2.0_f);
          real y = ((_y < y_length) ? _y : _y - this->delta_x / 2.0_f);
          if (x / x_length + y / y_length > 1.0_f - eps)
            continue;
          Vector position = elem.v[0] + x_n * x + y_n * y;
          positions.push_back(position);
        }
      for (auto &position : positions)
        add_boundry_particle(position, elem.get_normal(), &elem);
    }

    TC_TRACE("Mesh #elements = {}", mesh->elements.size());
  }
  TC_STATIC_END_IF

  rigid.position = initial_position;
  for (auto &p_i : added_particles) {
    auto p = static_cast<RigidBoundaryParticle<dim> *>(allocator[p_i]);
    p->align_with_rigid_body();
    if (near_boundary(*p)) {
      TC_WARN(
          "Particle close to domain boundary detected when adding rigid body");
      continue;
    }
    particles.push_back(p_i);
  }

  rigids.push_back(std::move(rigid_ptr));
  TC_TRACE("#Particles: {}", particles.size());
}

template <int dim>
void MPM<dim>::advect_rigid_bodies(real dt) {
  for (auto &r : rigids) {
    auto &rigid = *r;
    if (rigid.rotation_axis.abs_max() > 0.1_f) {
      rigid.enforce_angular_velocity_parallel_to(rigid.rotation_axis);
    }

    rigid.advance(this->current_t, dt);

    rigid.apply_impulse(this->gravity * rigid.get_mass() * dt, rigid.position);

    if (rigid.rotation_axis.abs_max() > 0.1_f) {
      rigid.enforce_angular_velocity_parallel_to(rigid.rotation_axis);
    }

    if (config_backup.get("print_rigid_body_state", false)) {
      // TC_P(rigid->get_mass());
      // TC_P(rigid->get_inertia());
      TC_P(rigid.position);
      TC_P(rigid.rotation.value);
      TC_P(rigid.velocity);
      TC_P(rigid.angular_velocity.value);
    }
  }
  parallel_for_each_particle([](Particle &p_) {
    if (p_.is_rigid()) {
      auto *p = dynamic_cast<RigidBoundaryParticle<dim> *>(&p_);
      p->align_with_rigid_body();
    }
  });
}

template <int dim>
void MPM<dim>::rigidify(real dt) {
  if (!config_backup.get<bool>("rigid_body_collision", true)) {
    return;
  }
  std::vector<Collision<dim>> collisions;
  {
    Profiler _("collision detection");
    TC_STATIC_IF(dim == 3) {
      RigidSolver<dim> rigid_solver;
      rigid_solver.detect_rigid_collision(rigids, collisions);
    }
    TC_STATIC_END_IF
  }

  if (!collisions.size()) {
    return;
  }

  {
    Profiler _("collision resolution");
    int iterations = config_backup.get("rigid_body_iterations", 5);
    for (int i = 0; i < iterations; i++) {
      auto rigid_penalty = config_backup.get("rigid_penalty", 1e3_f);
      if (config_backup.get("rigid_body_position_iterations", true)) {
        for (auto &col : collisions) {
          col.project_position(dt, rigid_penalty);
        }
      }
      for (auto &col : collisions) {
        col.project_velocity();
      }
    }
    for (int i = 0; i < iterations; i++) {
      for (auto &col : collisions) {
        col.project_velocity();
      }
    }
  }
}

template <int dim>
void MPM<dim>::rigid_body_levelset_collision(real t, real delta_t) {
  for (auto &p_i : particles) {
    auto &p = *allocator[p_i];
    if (p.is_rigid()) {
      Vector pos = p.pos * inv_delta_x;
      real phi = this->levelset.sample(pos, t);
      Vector gradient = this->levelset.get_spatial_gradient(pos, t);
      RigidBody<dim> *r = dynamic_cast<RigidBoundaryParticle<dim> *>(&p)->rigid;

      if (phi < 0) {
        real friction = r->frictions[0];
        real cRestitution = r->restitution;
        Vector v10 = r->get_velocity_at(p.pos);
        Vector r0 = p.pos - r->position;
        real v0 = dot(gradient, v10);

        real J = -((1 + cRestitution) * v0) *
                 inversed(r->get_impulse_contribution(r0, gradient));

        if (J < 0) {
          continue;
        }

        Vector impulse = J * gradient;
        r->apply_impulse(impulse, p.pos);

        // Friction
        v10 = r->get_velocity_at(p.pos);
        Vector tao = v10 - gradient * dot(gradient, v10);
        if (tao.abs_max() > 1e-7_f) {
          tao = normalized(tao);
          real j = -dot(v10, tao) / (r->get_impulse_contribution(r0, tao));
          j = clamp(j, friction * -J, friction * J);
          Vector fImpulse = j * tao;
          r->apply_impulse(fImpulse, p.pos);
        }
      }
    }
  }
}

template void MPM<2>::rigidify(real dt);
template void MPM<3>::rigidify(real dt);

template std::unique_ptr<RigidBody<2>> MPM<2>::create_rigid_body(Config config);
template std::unique_ptr<RigidBody<3>> MPM<3>::create_rigid_body(Config config);

template void MPM<2>::advect_rigid_bodies(real dt);
template void MPM<3>::advect_rigid_bodies(real dt);

template void MPM<2>::rigid_body_levelset_collision(real t, real delta_t);
template void MPM<3>::rigid_body_levelset_collision(real t, real delta_t);

template void MPM<2>::add_rigid_particle(Config config);
template void MPM<3>::add_rigid_particle(Config config);

TC_NAMESPACE_END
