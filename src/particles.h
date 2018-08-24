/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#pragma once

#include <taichi/util.h>
#include <taichi/math/svd.h>
#include <taichi/math/array.h>
#include <taichi/math/levelset.h>
#include "mpm_fwd.h"

TC_NAMESPACE_BEGIN

template <int dim>
class MPMParticle : public Unit {
 public:
  using Vector = VectorND<dim, real>;
  using VectorP = VectorND<dim + 1, real>;
  using Matrix = MatrixND<dim, real>;
  using Region = RegionND<dim>;

 private:
  VectorP v_and_m;

 public:
  Vector pos;
  // Elastic deformation gradient
  Matrix dg_e;
  // Affine momemtum (APIC)
  Matrix apic_b;
  Vector boundary_normal;
  real boundary_distance;
  real vol;
  int dt_limit;
  int stiffness_limit;
  int cfl_limit;
  bool near_boundary_;
  bool sticky;
  bool is_rigid_;
  uint32 states;
  int32 id;
  real debug;

  TC_IO_DEF_VIRT(v_and_m,
                 pos,
                 dg_e,
                 apic_b,
                 boundary_normal,
                 boundary_distance,
                 vol,
                 dt_limit,
                 stiffness_limit,
                 cfl_limit,
                 sticky,
                 near_boundary_,
                 states,
                 id,
                 is_rigid_);

  TC_FORCE_INLINE bool is_rigid() const {
    return is_rigid_;
  }

  TC_FORCE_INLINE real get_mass() const {
    return v_and_m[dim];
  }

  TC_FORCE_INLINE void set_mass(real mass) {
    v_and_m[dim] = mass;
  }

  TC_FORCE_INLINE Vector get_velocity() const {
    return Vector(v_and_m);
  }

  TC_FORCE_INLINE void set_velocity(const Vector &v) {
    v_and_m = VectorP(v, v_and_m[dim]);
  }

  TC_FORCE_INLINE void set_velocity(const __m128 &v) {
    v_and_m = _mm_blend_ps(v, v_and_m, 0x7);
  }

  MPMParticle() {
    dg_e = Matrix(1.0f);
    apic_b = Matrix(0);
    v_and_m = VectorP(0.0f);
    vol = 1.0f;
    states = 0;
    boundary_normal = Vector(0.0_f);
    boundary_distance = 0.0_f;
    dt_limit = 1;
    stiffness_limit = 1;
    cfl_limit = 1;
    near_boundary_ = false;
    id = 0;
    is_rigid_ = false;
  }

 public:
  bool near_boundary() const {
    return near_boundary_;
  }

  uint32 get_state(int i) const {
    return (states >> (i * 2)) % 4;
  }

  void set_state(int i, int state) {
    states = states ^ ((state ^ get_state(i)) << (i * 2));
  }

  virtual real get_allowed_dt(const real &dx) const = 0;

  virtual void initialize(const Config &config) {
    if (config.has_key("compressibility")) {
      TC_ERROR("'compressibility' is deprecated. Use 'initial_dg' instead");
    }
    dg_e = Matrix(config.get("initial_dg", 1.0_f));
  }

  virtual real potential_energy() const {
    TC_NOT_IMPLEMENTED
    return 0;
  }

  virtual Matrix first_piola_kirchhoff() {
    TC_NOT_IMPLEMENTED
    return Matrix(0.0f);
  }

  virtual Matrix calculate_force() {
    TC_NOT_IMPLEMENTED
    return Matrix(0.0f);
  }

  virtual int plasticity(const Matrix &cdg) {
    return 0;
  }

  virtual Matrix get_first_piola_kirchoff_differential(const Matrix &dF) {
    return Matrix(0.0f);
  }

  virtual ~MPMParticle() {
  }

  virtual real get_stiffness() const {
    TC_NOT_IMPLEMENTED
    return 0.0f;
  }

  virtual Vector3 get_debug_info() const {
    return Vector3(0);
  }

  virtual void set_mu_to_zero() {
  }

  virtual void set_lambda_and_mu_to_zero() {
  }

  /*
  real get_kenetic_energy() const {
    return dot(v, v) * mass * 0.5f;
  }

  Vector get_momentum() const {
    return mass * v;
  }

  real get_inertia(const real &dx) const {
    return 0.5f * dx * dx * this->mass;
  }

  real get_rpic_angular_momentum(const real &dx) const {
    return get_inertia(dx) * apic_b[1][0];
  }

  Vector get_momemtum() const {
    return v * mass;
  }

  void apply_impulse(Vector impulse) {
    v += (1.0f / mass) * impulse;
  }
  */
};

using MPMParticle2D = MPMParticle<2>;
using MPMParticle3D = MPMParticle<3>;

TC_INTERFACE(MPMParticle2D);

TC_INTERFACE(MPMParticle3D);

#define TC_REGISTER_MPM_PARTICLE(name)                                \
  using name##Particle2D = name##Particle<2>;                         \
  using name##Particle3D = name##Particle<3>;                         \
  static_assert(                                                      \
      sizeof(name##Particle2D) <= get_particle_size_upper_bound<2>(), \
      "2D MPM particle (" #name ") cannot exceed 192B");              \
  static_assert(                                                      \
      sizeof(name##Particle3D) <= get_particle_size_upper_bound<3>(), \
      "3D MPM particle (" #name ") cannot exceed 256B");              \
  TC_IMPLEMENTATION(MPMParticle2D, name##Particle2D,                  \
                    name##Particle2D().get_name());                   \
  TC_IMPLEMENTATION(MPMParticle3D, name##Particle3D,                  \
                    name##Particle2D().get_name());

TC_NAMESPACE_END
