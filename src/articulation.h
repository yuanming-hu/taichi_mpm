/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#pragma once

#include <taichi/dynamics/rigid_body.h>

TC_NAMESPACE_BEGIN

template <int dim>
class Articulation : public Unit {
 public:
  using RigidBodyType = RigidBody<dim>;

  RigidBodyType *obj[2];

  virtual void apply(real delta_t) const {};
  virtual void penalize(real delta_t) const {};
  virtual void project() const {};
  virtual void update_parameters(real param1, real param2, real param3){};

  TC_IO_DEF_VIRT(obj);
};

using Articulation2D = Articulation<2>;
using Articulation3D = Articulation<3>;

TC_INTERFACE(Articulation2D);
TC_INTERFACE(Articulation3D);

TC_NAMESPACE_END
