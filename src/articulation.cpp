/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#include <pybind11/pybind11.h>
#include "articulation.h"

TC_NAMESPACE_BEGIN

TC_INTERFACE_DEF(Articulation2D, "articulation2d");
TC_INTERFACE_DEF(Articulation3D, "articulation3d");

#define TC_REGISTER_ARTICULATION(name)                    \
  using name##Articulation2D = name##Articulation<2>;     \
  using name##Articulation3D = name##Articulation<3>;     \
  TC_IMPLEMENTATION(Articulation2D, name##Articulation2D, \
                    name##Articulation2D().get_name());   \
  TC_IMPLEMENTATION(Articulation3D, name##Articulation3D, \
                    name##Articulation2D().get_name());

template <int dim>
class RotationArticulation : public Articulation<dim> {
 public:
  using Base = Articulation<dim>;
  using typename Base::RigidBodyType;
  using Base::obj;

  void initialize(const Config &config) override {
    obj[0] = config.get_ptr<RigidBodyType>("obj0");
    obj[1] = config.get_ptr<RigidBodyType>("obj1");
  }

  void project() const override {
    auto total_angular_momemtum =
        obj[0]->get_angular_momemtum() + obj[1]->get_angular_momemtum();
    auto total_inertia =
        obj[0]->get_transformed_inertia() + obj[1]->get_transformed_inertia();
    auto angular_velocity = inversed(total_inertia) * total_angular_momemtum;
    obj[0]->angular_velocity = obj[1]->angular_velocity = angular_velocity;
  }

  std::string get_name() const override {
    return "rotation";
  }
};

TC_REGISTER_ARTICULATION(Rotation);

template <int dim>
class FrozenArticulation : public Articulation<dim> {
 public:
  using Base = Articulation<dim>;
  using typename Base::RigidBodyType;
  using Base::obj;

  void initialize(const Config &config) override {
    obj[0] = config.get_ptr<RigidBodyType>("obj0");
    obj[1] = config.get_ptr<RigidBodyType>("obj1");
  }

  void project() const override;

  std::string get_name() const override {
    return "frozen";
  }
};

template <>
void FrozenArticulation<2>::project() const {
  TC_NOT_IMPLEMENTED;
}
template <>
void FrozenArticulation<3>::project() const {
  auto angular_velocity = obj[0]->angular_velocity;
  angular_velocity.value.x = 0;
  angular_velocity.value.y = 0;
  obj[0]->angular_velocity = angular_velocity;
  auto velocity = obj[0]->velocity;
  velocity.z = 0;
  obj[0]->velocity = velocity;
}

TC_REGISTER_ARTICULATION(Frozen);

template <int dim>
class DistanceArticulation : public Articulation<dim> {
 public:
  using Base = Articulation<dim>;
  using typename Base::RigidBodyType;
  using Vector = VectorND<dim, real>;
  using Base::obj;

  Vector offsets[2];
  real target_distance;
  real penalty;
  TC_IO_DEF_WITH_BASE(offsets, target_distance, penalty);

  void initialize(const Config &config) override {
    obj[0] = config.get_ptr<RigidBodyType>("obj0");
    obj[1] = config.get_ptr<RigidBodyType>("obj1");
    if (obj[1]->id == 0) {
      TC_ASSERT_INFO(config.has_key("offset1"),
                     "When linkning to background rigid body (obj1 = nullptr), "
                     "Distance Articulation must have non-default offset1")
    }
    for (int i = 0; i < 2; i++) {
      offsets[i] =
          config.get<Vector>(fmt::format("offset{}", i), Vector(0.0_f));
      offsets[i] =
          transform(inverse(obj[i]->get_centroid_to_world()), offsets[i], 0);
    }
    Vector p0 = transform(obj[0]->get_centroid_to_world(), offsets[0]);
    Vector p1 = transform(obj[1]->get_centroid_to_world(), offsets[1]);
    Vector n = p0 - p1;
    real initial_distance = n.length();
    target_distance = config.get("target_distance", initial_distance);

    penalty = config.get("penalty", 1e3_f);
  }

  void penalize(real delta_t) const override {
    // Correction term
    Vector p0 = transform(obj[0]->get_centroid_to_world(), offsets[0]);
    Vector p1 = transform(obj[1]->get_centroid_to_world(), offsets[1]);
    Vector n = p0 - p1;

    real current_distance = n.length();

    if (current_distance < 1e-10_f) {
      return;
    }
    n = normalized(n);

    real j = -delta_t * penalty * (target_distance - current_distance);

    obj[0]->apply_impulse(-j * n, p0);
    obj[1]->apply_impulse(j * n, p1);
  }

  void project() const override {
    Vector p0 = transform(obj[0]->get_centroid_to_world(), offsets[0]);
    Vector p1 = transform(obj[1]->get_centroid_to_world(), offsets[1]);
    Vector n = p0 - p1;

    real current_distance = n.length();

    if (current_distance < 1e-10_f) {
      return;
    }
    n = normalized(n);
    Vector v01 = obj[0]->get_velocity_at(p0) - obj[1]->get_velocity_at(p1);

    real j = dot(n, v01) /
             (obj[0]->get_impulse_contribution(p0 - obj[0]->position, n) +
              obj[1]->get_impulse_contribution(p1 - obj[1]->position, n));

    obj[0]->apply_impulse(-j * n, p0);
    obj[1]->apply_impulse(j * n, p1);
  }

  std::string get_name() const override {
    return "distance";
  }
};

TC_REGISTER_ARTICULATION(Distance);

template <int dim>
class AxialRotationArticulation : public Articulation<dim> {
 public:
  using Base = Articulation<dim>;
  using typename Base::RigidBodyType;
  using Base::obj;
  using Vector = VectorND<dim, real>;
  using TorqueType = typename AngularVelocity<dim>::ValueType;

  DistanceArticulation<dim> distance_articulations[2];
  Vector3 axis;
  TC_IO_DEF_WITH_BASE(distance_articulations, axis);

  void initialize(const Config &config) override {
    obj[0] = config.get_ptr<RigidBodyType>("obj0");
    obj[1] = config.get_ptr<RigidBodyType>("obj1");
    auto offset0 = config.get<Vector>("offset0", Vector(0.0_f));

    TC_STATIC_IF(dim == 3) {
      auto input_axis = normalized(config.get<Vector3>("axis"));
      axis = transform(inversed(this->obj[1]->get_centroid_to_world()),
                       input_axis, 0);
    }
    TC_STATIC_END_IF

    Vector offset = obj[0]->position + offset0 - obj[1]->position;
    for (int i = 0; i < 2; i++) {
      Config config_ = config;
      config_.set("penalty", config.get("penalty", 1e3_f));
      TC_STATIC_IF(dim == 3) {
        auto input_axis = normalized(config.get<Vector3>("axis"));
        Vector axial_offset = input_axis * config.get("axis_length", 0.1_f);
        if (i == 1) {
          axial_offset = -axial_offset;
        }
        config_.set("offset0", offset0 + axial_offset)
            .set("offset1", offset + axial_offset)
            .set("target_distance", 0);
      }
      TC_STATIC_ELSE {
        config_.set("offset0", offset0)
            .set("offset1", offset)
            .set("target_distance", 0);
      }
      TC_STATIC_END_IF
      distance_articulations[i].initialize(config_);
    }
  }

  void project() const override {
    distance_articulations[0].project();
    distance_articulations[1].project();
  }

  void penalize(real delta_t) const override {
    distance_articulations[0].penalize(delta_t);
    distance_articulations[1].penalize(delta_t);
  }

  std::string get_name() const override {
    return "axial_rotation";
  }
};
TC_REGISTER_ARTICULATION(AxialRotation);

template <int dim>
class MotorArticulation : public Articulation<dim> {
 public:
  using Base = Articulation<dim>;
  using typename Base::RigidBodyType;
  using Base::obj;
  using Vector = VectorND<dim, real>;
  using TorqueType = typename AngularVelocity<dim>::ValueType;

  real power;
  AxialRotationArticulation<dim> axial_rotation_articulation;
  TC_IO_DEF_WITH_BASE(power, axial_rotation_articulation);

  void initialize(const Config &config) override {
    obj[0] = config.get_ptr<RigidBodyType>("obj0");
    obj[1] = config.get_ptr<RigidBodyType>("obj1");
    axial_rotation_articulation.initialize(config);
    power = config.get<real>("power", 0.0_f);
  }

  void apply(real delta_t) const override {
    TorqueType torque;
    TC_STATIC_IF(dim == 2) {
      torque = power * delta_t;
    }
    TC_STATIC_ELSE {
      Vector3 transformed_axis =
          transform(this->obj[1]->get_centroid_to_world(),
                    axial_rotation_articulation.axis, 0);
      torque = transformed_axis * power * delta_t;
    }
    TC_STATIC_END_IF
    obj[0]->apply_torque(torque);
    obj[1]->apply_torque(-torque);
  }

  void project() const override {
    axial_rotation_articulation.project();
  }

  void penalize(real delta_t) const override {
    axial_rotation_articulation.penalize(delta_t);
  }

  std::string get_name() const override {
    return "motor";
  }
};
TC_REGISTER_ARTICULATION(Motor);

template <int dim>
class StepperArticulation : public Articulation<dim> {
 public:
  using Base = Articulation<dim>;
  using typename Base::RigidBodyType;
  using Base::obj;
  using Vector = VectorND<dim, real>;
  using TorqueType = typename AngularVelocity<dim>::ValueType;

  real angular_velocity;
  bool is_left;
  AxialRotationArticulation<dim> axial_rotation_articulation;
  TC_IO_DEF_WITH_BASE(angular_velocity, is_left, axial_rotation_articulation);

  void initialize(const Config &config) override {
    obj[0] = config.get_ptr<RigidBodyType>("obj0");
    obj[1] = config.get_ptr<RigidBodyType>("obj1");
    axial_rotation_articulation.initialize(config);
    is_left = config.get("is_left", false);
    angular_velocity = config.get<real>("angular_velocity", 0.0_f);
  }

  void apply(real delta_t) const override {
    axial_rotation_articulation.apply(delta_t);
  }

  void project() const override {
    axial_rotation_articulation.project();

    TorqueType torque;
    auto current_vel =
        obj[0]->angular_velocity.value - obj[1]->angular_velocity.value;
    TC_STATIC_IF(dim == 2) {
      TC_NOT_IMPLEMENTED;
      torque = (angular_velocity - current_vel) *
               (this->obj[0]->inertia + this->obj[1]->inertia);
    }
    TC_STATIC_ELSE {
      Vector3 axis = normalized(transform(this->obj[1]->get_centroid_to_world(),
                                          axial_rotation_articulation.axis, 0));

      auto correction_vel_projected = angular_velocity - dot(current_vel, axis);
      torque = inversed(this->obj[0]->get_transformed_inversed_inertia() +
                        this->obj[1]->get_transformed_inversed_inertia()) *
               (axis * correction_vel_projected);
    }
    TC_STATIC_END_IF
    obj[0]->apply_torque(torque);
    obj[1]->apply_torque(-torque);
  }

  void penalize(real delta_t) const override {
    axial_rotation_articulation.penalize(delta_t);
  }

  void update_parameters(real param1, real param2, real param3) override {
    param1 -= 4._f;
    if (param1 > 1.5)
      angular_velocity = -50;
    else if (param1 > 0.75)
      angular_velocity = -10 - 40 * (param1 - 0.75) / 0.75;
    else
      angular_velocity = -10;
  }

  std::string get_name() const override {
    return "stepper";
  }
};
TC_REGISTER_ARTICULATION(Stepper);

TC_NAMESPACE_END
