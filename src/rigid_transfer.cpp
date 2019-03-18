/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#include <taichi/system/profiler.h>
#include <taichi/dynamics/rigid_body.h>
#include "mpm.h"
#include "kernel.h"
#include "boundary_particle.h"

#ifndef MPM_TRANSFER_OPT
// Non-optimized Transfer

TC_NAMESPACE_BEGIN

// writes to grid_state, grid_cdf, grid_sdf_last_rigid_particle, rigid_markers
template <int dim>
void MPM<dim>::rasterize_rigid_boundary() {
  // Note: we assume particles from the same rigid body are contiguous
  parallel_for_each_particle([&](Particle &p_) {
    auto *p = dynamic_cast<RigidBoundaryParticle<dim> *>(&p_);
    if (!p)
      return;
    constexpr int kernel_size = cdf_kernel_order_rasterize + 1;
    RegionND<dim> region(VectorI(0), VectorI(kernel_size));
    Vectori grid_base_pos =
        get_grid_base_pos_with<kernel_size>(p->pos * inv_delta_x);

    auto elem = p->get_world_space_element();
    Matrix world_to_elem = world_to_element(elem);
    Vector p0 = elem.v[0];

    for (auto &ind : region) {
      auto i = ind.get_ipos() + grid_base_pos;
      auto grid_pos = i.template cast<real>() * delta_x;
      Vector coord = world_to_elem * (grid_pos - p0);
      bool negative = coord[dim - 1] < 0;
      real dist_triangle = abs(coord[dim - 1]);
      bool in_range = false;

      TC_STATIC_IF(dim == 2) {
        if (-0.02 <= coord[0] && coord[0] <= 1.02) {
          in_range = true;
        }
      }
      TC_STATIC_ELSE {
        if (0 <= coord[0] && 0 <= coord[1] && coord[0] + coord[1] <= 1) {
          in_range = true;
        }
      }
      TC_STATIC_END_IF

      if (!in_range) {
        continue;
      }

      dist_triangle *= inv_delta_x;

      GridState<dim> &g = get_grid(i);

      g.get_lock().lock();
      if (g.get_rigid_body_id() == -1 ||
          dist_triangle < get_grid(i).get_distance()) {
        g.set_distance(dist_triangle);
        g.set_rigid_body_id(p->rigid->id);
      }
      // TODO: what happens if the relative position of the grid point to a
      // rigid body is ambiguious???
      // Should be fine for most meshes, though?
      g.set_states(g.get_states() |
                   (2 + (int)(negative)) << (p->rigid->id * 2));
      g.get_lock().unlock();
    }
  });
  parallel_for_each_active_grid(
      [&](GridState<dim> &g) { g.set_distance(g.get_distance() * delta_x); });

  TC_STATIC_IF(dim == 2) {
    ArrayND<2, uint32> grid_states_tmp(this->res + Vectori(1), 0);
    for (int e = 0; e < config_backup.get<int>("cdf_expand", 0); e++) {
      for (int k = 0; k < dim; k++) {
        Region region = Region(Vectori::axis(k), res - Vectori::axis(k));
        grid_states_tmp.reset_zero();
        for (auto &ind : region) {
          grid_states_tmp[ind] = this->get_grid(ind.get_ipos()).states;
        }
        for (auto &ind : region) {
          auto update = [&](Vectori offset) {
            auto nei_state = this->get_grid(ind + offset).states;
            auto state = grid_states_tmp[ind];
            auto states_to_add =
                ((nei_state & ~state) & state_mask) & GridState<dim>::tag_mask;
            grid_states_tmp[ind] =
                state | (nei_state & (states_to_add | (states_to_add >> 1)));
          };
          update(Vectori::axis(k));
          update(-Vectori::axis(k));
        }
        for (auto &ind : region) {
          this->get_grid(ind.get_ipos()).states = grid_states_tmp[ind];
        }
      }
      int count = 0;
      Region region = Region(Vectori(0), this->res);
      for (auto &ind : region) {
        count += (0 != this->get_grid(ind.get_ipos()).states);
      }
      // TC_P(count);
    }
  }
  TC_STATIC_END_IF
}

template void MPM<2>::rasterize_rigid_boundary();

template void MPM<3>::rasterize_rigid_boundary();

// Reads grid_state and grid_cdf
template <int dim>
void MPM<dim>::gather_cdf() {
  // Gain new color
  // do not modify existing colors
  /*
    p.states = (particle_state | (new_color_mask << 1) |
                //((grid_state & new_color_mask) * bit));
                (new_color_mask * bit));
                */
  parallel_for_each_particle([&](Particle &p) {
    p.sticky = false;
    if (p.is_rigid()) {
      return;
    }

    using CDFPrecision = float32;

    using VectorPd = VectorND<dim + 1, CDFPrecision>;
    using MatrixPd = MatrixND<dim + 1, CDFPrecision>;
    using Vectord = VectorND<dim, CDFPrecision>;

    p.boundary_distance = 0;
    p.boundary_normal = Vector(0.0_f);
    p.near_boundary_ = false;
    auto pos = p.pos * inv_delta_x;
    auto linearized_offset =
        SparseMask::Linear_Offset(pos.template cast<int>());
    if (dim == 3 && !rigid_page_map->Test_Page(linearized_offset)) {
      return;
    }
    AffinityType &particle_states = p.states;

    constexpr int kernel_size = cdf_kernel_order_gather + 1;
    RegionND<dim> region(VectorI(0), VectorI(kernel_size));
    Vectori grid_base_pos = get_grid_base_pos_with<kernel_size>(pos);
    MPMKernel<dim, kernel_size - 1> kernel(pos, inv_delta_x);

    AffinityType all_boundaries = 0;
    for (auto &ind : region) {
      auto i = ind.get_ipos() + grid_base_pos;
      all_boundaries |= (get_grid(i).get_states() & state_mask);
    }

    // Unset states that the particle does not touch
    particle_states &= (all_boundaries + (all_boundaries >> 1));

    AffinityType all_states_to_add = all_boundaries & (~particle_states);

    while (all_states_to_add != 0) {
      AffinityType state_to_add = (all_states_to_add & -all_states_to_add);
      all_states_to_add ^= state_to_add;
      CDFPrecision weighted_distances[2] = {0, 0};
      for (auto &ind : region) {
        auto i = ind.get_ipos() + grid_base_pos;
        Vectord dpos =
            (pos - i.template cast<real>()).template cast<CDFPrecision>();

        VectorP dw_w = kernel.get_dw_w(ind.get_ipos());

        // Coloring
        GridState<dim> &g = get_grid(i);
        uint64 grid_state = g.get_states();

        if (g.get_rigid_body_id() == -1) {
          continue;
        }

        CDFPrecision d = g.get_distance() * inv_delta_x;
        VectorPd xp(-dpos, 1);
        CDFPrecision weight;
        if (mpm_use_weighted_reconstruction) {
          weight = dw_w[dim];
        } else {
          weight = 1.0f;
        }
        // assert((state_to_add & (state_to_add - 1)) == 0);
        if (grid_state & state_to_add) {
          int sign = (int)((grid_state & (state_to_add >> 1)) != 0);
          weighted_distances[sign] += d * weight;
        }
      }
      if (weighted_distances[0] + weighted_distances[1] > 1e-7_f) {
        p.states |=
            (state_to_add | ((state_to_add >> 1) * int(weighted_distances[0] <
                                                       weighted_distances[1])));
        cutting_counter++;
      }
    }
    if (particle_states != 0) {
      MatrixPd XtX(0);
      VectorPd XtY(0);
      for (auto &ind : region) {
        auto i = ind.get_ipos() + grid_base_pos;
        Vectord dpos =
            (pos - i.template cast<real>()).template cast<CDFPrecision>();

        VectorP dw_w = kernel.get_dw_w(ind.get_ipos());

        GridState<dim> &g = get_grid(i);

        if (g.get_rigid_body_id() == -1) {
          continue;
        }

        // Coloring
        uint64 grid_state = g.get_states();
        uint64 mask = (grid_state & particle_states & state_mask) >> 1;
        CDFPrecision d = g.get_distance() * inv_delta_x;
        VectorPd xp(-dpos, 1);
        CDFPrecision weight;
        if (mpm_use_weighted_reconstruction) {
          weight = dw_w[dim];
        } else {
          weight = 1.0f;
        }
        if (grid_state != 0) {
          if ((grid_state & mask) == (particle_states & mask) &&
              (grid_state != 0)) {
            // same color
            XtX += MatrixPd::outer_product(xp, xp) * weight;
            XtY += VectorPd(-d * dpos, d) * weight;
          } else if (cdf_use_negative) {
            // Only one color different, use negative
            uint64 diff = ((grid_state & mask) ^ (particle_states & mask));
            if (diff > 0 && 0 == (diff & (diff - 1))) {
              XtX += MatrixPd::outer_product(xp, xp) * weight;
              XtY += VectorPd(d * dpos, -d) * weight;
            }
          }
        }
      }
      if (std::abs(determinant(XtX)) > mpm_reconstruction_guard<dim>()) {
        VectorP r = (inversed(XtX) * XtY).template cast<real>();
        p.near_boundary_ = true;
        // r[dim] -= 1 * (1 - length2(Vector(r)));
        p.boundary_distance = r[dim] * delta_x;
        p.debug = length2(Vector(r));
        if (length2(Vector(r)) > 1e-4_f) {
          p.boundary_normal = normalized(Vector(r));
        } else {
          p.boundary_normal = Vector(0);

          // p.sticky = true;

          // TC_WARN("0 boundary normal detected");
        }
      } else {
        // p.states = 0;
        p.boundary_distance = 0;
        p.boundary_normal = Vector(0);
        // p.near_boundary_ = false;
      }
    }
  });
}

template void MPM<2>::gather_cdf();
template void MPM<3>::gather_cdf();

TC_NAMESPACE_END

#endif
