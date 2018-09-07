/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#pragma once

#include "mpm.h"
#include "particles.h"
#include <taichi/math/angular.h>
#include <taichi/geometry/mesh.h>

TC_NAMESPACE_BEGIN

template <int dim>
class RigidBoundaryParticle : public MPMParticle<dim> {
 public:
  using Base = MPMParticle<dim>;
  using typename Base::Vector;
  using typename Base::Matrix;
  using Base::pos;
  using ElementType = typename RigidBody<dim>::ElementType;

  RigidBody<dim> *rigid = nullptr;
  ElementType untransformed_element;
  Vector offset;
  Vector original_normal;
  bool climb_rudder = false;

  TC_IO_DEF_WITH_BASE(rigid,
                      offset,
                      untransformed_element,
                      original_normal,
                      climb_rudder);

  RigidBoundaryParticle() : Base() {
    Base::is_rigid_ = true;
  }

  void initialize(const Config &config) override {
    Base::initialize(config);
    rigid = config.get_ptr<RigidBody<dim>>("rigid");
    offset = config.get<Vector>("offset");
    original_normal = config.get<Vector>("normal");
    climb_rudder = config.get("climb_rudder", false);
  }

  void align_with_rigid_body() {
    pos = get_anchor_point();
    this->set_velocity(
        rigid->velocity +
        rigid->angular_velocity.cross(rigid->rotation.rotate(offset)));
  }

  Vector get_normal() const {
    return rigid->rotation.rotate(original_normal);
  }

  real get_allowed_dt(const real &dx) const override {
    return 0.0f;
  }

  std::string get_name() const override {
    return "rigid_boundary";
  }

  Vector get_anchor_point() const {
    return transform(rigid->get_centroid_to_world(), offset);
  }

  ElementType get_world_space_element() const {
    return untransformed_element.get_transformed(rigid->get_mesh_to_world());
  }

  Vector3 get_debug_info() const override {
    return Vector3((real)climb_rudder, 0, 0);
  }
};

TC_NAMESPACE_END
