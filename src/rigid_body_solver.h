/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#pragma once

#include <memory>
#include <vector>
#include <taichi/common/meta.h>
#include <taichi/dynamics/rigid_body.h>
#include <ccd/ccd.h>
#include <ccd/quat.h>
#include <taichi/math/eigen.h>

TC_NAMESPACE_BEGIN

//#define DEBUG_CCD

template <int dim>
class Collision {
 public:
  using Vector = VectorND<dim, real>;
  RigidBody<dim> *objects[2];
  Vector normal;
  Vector p;
  real depth;

  Collision(RigidBody<dim> *a,
            RigidBody<dim> *b,
            real depth,
            Vector normal,
            Vector p)
      : normal(normal), p(p), depth(depth) {
    objects[0] = a;
    objects[1] = b;
  }

  void project_velocity() {
    real friction =
        std::sqrt(objects[0]->frictions[0] * objects[1]->frictions[0]);
    real cRestitution =
        std::sqrt(objects[0]->restitution * objects[1]->restitution);
    Vector v10 =
        objects[1]->get_velocity_at(p) - objects[0]->get_velocity_at(p);
    Vector r0 = p - objects[0]->position, r1 = p - objects[1]->position;
    real v0 = -dot(normal, v10);

    real J;
    J = ((1 + cRestitution) * v0) *
        inversed(objects[0]->get_impulse_contribution(r0, normal) +
                 objects[1]->get_impulse_contribution(r1, normal));
    if (J < 0)
      return;

    Vector impulse = J * normal;
    objects[0]->apply_impulse(-impulse, p);
    objects[1]->apply_impulse(impulse, p);

    v10 = objects[1]->get_velocity_at(p) - objects[0]->get_velocity_at(p);
    Vector tao = v10 - normal * dot(normal, v10);
    if (tao.abs_max() > 1e-7_f) {
      tao = normalized(tao);
      real j = -dot(v10, tao) / (objects[0]->get_impulse_contribution(r0, tao) +
                                 objects[1]->get_impulse_contribution(r1, tao));
      j = std::max(std::min(j, friction * J), -friction * J);
      Vector fImpulse = j * tao;
      objects[0]->apply_impulse(-fImpulse, p);
      objects[1]->apply_impulse(fImpulse, p);
    }
  }

  void project_position(real dt, real penalty) {
    Vector r0 = p - objects[0]->position, r1 = p - objects[1]->position;

    real J;
    J = penalty * dt * depth *
        inversed(objects[0]->get_impulse_contribution(r0, normal) +
                 objects[1]->get_impulse_contribution(r1, normal));

    if (J < 0)
      return;

    Vector impulse = J * normal;
    objects[0]->apply_impulse(-impulse, p);
    objects[1]->apply_impulse(impulse, p);
  }
};

template <int dim>
class RigidSolver {
  using Vector = VectorND<dim, real>;
  using VectorP = VectorND<dim + 1, real>;
  using VectorI = VectorND<dim, int>;
  using Vectori = VectorND<dim, int>;
  using Matrix = MatrixND<dim, real>;
  using MatrixP = MatrixND<dim + 1, real>;

  typedef std::vector<std::unique_ptr<RigidBody<dim>>> RigidsVector;
  typedef std::vector<Collision<dim>> CollisionsVector;

  static inline void center(const void *obj, ccd_vec3_t *center) {
    const RigidBody<dim> *rigid = static_cast<const RigidBody<dim> *>(obj);
    Vector pos_vec = rigid->position;
    ccdVec3Set(center, pos_vec[0], pos_vec[1], pos_vec[2]);
  }

  static inline void convert_rotation_matrix_to_quaternion(
      const Matrix &m_,
      Eigen::Quaternion<real> &q_) {
    q_ = Eigen::Quaternion<real>(to_eigen(m_));
  }

  static inline void centerRigid(const void *obj, ccd_vec3_t *center) {
    const RigidBody<dim> *rigid = static_cast<const RigidBody<dim> *>(obj);
    ccdVec3Set(center, rigid->position[0], rigid->position[1],
               rigid->position[2]);
  }

  static inline void supportRigid(const void *obj,
                                  const ccd_vec3_t *dir_,
                                  ccd_vec3_t *v) {
    const RigidBody<dim> *rigid = static_cast<const RigidBody<dim> *>(obj);
    ccd_vec3_t dir;

    ccdVec3Copy(&dir, dir_);
    ccdVec3Normalize(&dir);

    Vector tdir(dir.v[0], dir.v[1], dir.v[2]);

    auto rot = MatrixP(rigid->rotation.get_rotation_matrix());
    rot[dim][dim] = 1;
    auto trans = rigid->get_mesh_to_world();

    real max_dist = -1e30_f;
    for (auto element : rigid->mesh->elements) {
      for (int i = 0; i < dim; i++) {
        Vector v_ = transform(rot, element.v[i]);
        Vector p_ = transform(trans, element.v[i]);
        auto dist = dot(tdir, v_);
        if (dist > max_dist) {
          max_dist = dist;
          ccdVec3Set(v, p_.x, p_.y, p_.z);
        }
      }
    }
  }

 public:
  void detect_rigid_collision(RigidsVector &rigids,
                              CollisionsVector &collisions);
};

template <>
void RigidSolver<2>::detect_rigid_collision(RigidsVector &rigids,
                                            CollisionsVector &collisions) {
  TC_NOT_IMPLEMENTED
}

template <>
void RigidSolver<3>::detect_rigid_collision(RigidsVector &rigids,
                                            CollisionsVector &collisions) {
  int rigids_number = rigids.size();
  ccd_t ccd;
  std::mutex mut;

  tbb::parallel_for(0, rigids_number * rigids_number, [&](int k) {
    // for (int k = 0; k < rigids_number * rigids_number; k++) {
    int i = k / rigids_number;
    int j = k % rigids_number;
    if (i == 0 || j == 0 || j >= i) {
      return;
    }
    if (rigids[i]->pos_func && rigids[i]->rot_func)
      if (rigids[j]->pos_func && rigids[j]->rot_func)
        return;
    CCD_INIT(&ccd);
    ccd.support1 = supportRigid;
    ccd.support2 = supportRigid;
    ccd.center1 = centerRigid;
    ccd.center2 = centerRigid;
    //      ccd.max_iterations = 200;
    //      ccd.epa_tolerance = 0.00001;
    ccd.mpr_tolerance = 0.0001;
    ccd_real_t depth;
    ccd_vec3_t dir, pos;
    const void *obj1 = static_cast<const void *>(rigids[i].get());
    const void *obj2 = static_cast<const void *>(rigids[j].get());
    int intersect = ccdMPRPenetration(obj1, obj2, &ccd, &depth, &dir, &pos);
    if (!intersect) {
      mut.lock();
      collisions.emplace_back(rigids[i].get(), rigids[j].get(), depth,
                              Vector(dir.v[0], dir.v[1], dir.v[2]),
                              Vector(pos.v[0], pos.v[1], pos.v[2]));
      mut.unlock();
    }
  });
}

TC_NAMESPACE_END
