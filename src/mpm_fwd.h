/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#pragma once

#include <taichi/util.h>
#include <SPGrid/Core/SPGrid_Allocator.h>
#include <SPGrid/Core/SPGrid_Page_Map.h>
#include <taichi/system/threading.h>
#include <taichi/common/bit.h>

TC_NAMESPACE_BEGIN

using namespace SPGrid;
constexpr int mpm_kernel_order = 2;
constexpr bool mpm_use_weighted_reconstruction = true;
constexpr int cdf_kernel_order_rasterize = 2;
constexpr int cdf_kernel_order_gather = 2;
constexpr bool cdf_use_negative = true;
using math::radians;
using math::degrees;

template <int dim>
TC_FORCE_INLINE VectorND<dim, real> friction_project(
    const VectorND<dim, real> &velocity,
    const VectorND<dim, real> &base_velocity,
    const VectorND<dim, real> &normal,
    real friction) {
  using Vector = VectorND<dim, real>;
  auto relative_vel = velocity - base_velocity;
  if (friction == -1) {
    return base_velocity;
  }

  bool slip = friction <= -2;
  if (slip) {
    friction = -friction - 2;
  }

  real normal_norm = dot(normal, relative_vel);
  Vector tangential_relative_vel = relative_vel - normal_norm * normal;

  real tangential_norm = length(tangential_relative_vel);

  real tangential_scale =
      std::max(tangential_norm + std::min(normal_norm, 0.0_f) * friction,
               0.0_f) /
      std::max(1e-30_f, tangential_norm);

  Vector projected_relative_vel =
      tangential_scale * tangential_relative_vel +
      std::max(0.0_f, normal_norm * real(!slip)) * normal;

  return projected_relative_vel + base_velocity;
}

template <int dim>
constexpr int get_particle_size_upper_bound() {
  static_assert(dim == 2 || dim == 3, "only 2D and 3D supported");
  if (dim == 2) {
    return 192;
  } else {
    return 320;
  }
}

template <int dim>
struct GridState {
  VectorND<dim + 1, real> velocity_and_mass;
  float32 distance = 0;
  uint32 states = 0;
  uint32 particle_count;
  Spinlock lock;
  uint16 flags;

  static constexpr int max_num_rigid_bodies = 12;
  static constexpr int total_bits = 32;
  static constexpr uint32 tag_bits = max_num_rigid_bodies * 2;
  static constexpr uint32 id_bits = total_bits - tag_bits;

  static constexpr uint32 tag_mask = (1u << tag_bits) - 1;
  static constexpr uint32 id_mask = ((1u << id_bits) - 1u) << tag_bits;

  int get_rigid_body_id() const {
    return (states >> tag_bits) - 1;
  }

  void set_rigid_body_id(int id) {
    states = (states & ~id_mask) | ((id + 1) << tag_bits);
  }

  float32 get_distance() const {
    return distance;
  }

  void set_distance(float32 new_distance) {
    distance = new_distance;
  }

  uint64 get_states() const {
    return states & tag_mask;
  }

  void set_states(uint32 new_states) {
    states = (states & ~tag_mask) | new_states;
  }

  Spinlock &get_lock() {
    return lock;
  }
};

static_assert(bit::is_power_of_two((int)sizeof(GridState<2>)),
              "GridState<2> size must be POT");

static_assert(bit::is_power_of_two((int)sizeof(GridState<3>)),
              "GridState<3> size must be POT");

template <int dim>
constexpr float64 mpm_reconstruction_guard() {
  static_assert(dim == 2 || dim == 3, "dim must be 2 or three");
  if (dim == 2) {
    return 3e-3_f64;
  } else {
    return 1e-4_f64;
  }
}

template <int dim, int order>
struct MPMKernel;

template <int dim>
class MPM;

template <int dim>
class MPMScheduler;

template <int dim>
class MPMParticle;

TC_NAMESPACE_END
