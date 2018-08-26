/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#pragma once

#include <memory>
#include <vector>
#include <memory.h>
#include <string>
#include <functional>
#include <utility>

#include <taichi/visualization/image_buffer.h>
#include <taichi/common/meta.h>
#include <taichi/dynamics/simulation.h>
#include <taichi/math/array_3d.h>
#include <taichi/math/svd.h>
#include <taichi/math/levelset.h>
#include <taichi/system/threading.h>
#include <taichi/visualization/pakua.h>

#include "mpm_fwd.h"
#include "particle_allocator.h"
#include "kernel.h"
#include "particles.h"
#include "articulation.h"
#include "taichi/dynamics/rigid_body.h"

TC_NAMESPACE_BEGIN

template <int dim, int ORDER>
struct MPMKernel;

constexpr uint64 state_mask = 0xAAAAAAAAAAAAAAAA;

template <typename T>
void read_from_binary_file_dynamic(T *t, const std::string &file_name) {
  BinaryInputSerializer reader;
  reader.initialize(file_name);
  t->binary_io(reader);
  reader.finalize();
}

template <typename T>
void write_to_binary_file_dynamic(const T *t, const std::string &file_name) {
  TC_P(t->get_name());
  BinaryOutputSerializer writer;
  writer.initialize();
  t->binary_io(writer);
  writer.finalize();
  writer.write_to_file(file_name);
}

template <int dim>
class MPM : public Simulation<dim> {
 public:
  using Base = Simulation<dim>;
  using typename Base::Vector;
  using typename Base::VectorP;
  using typename Base::VectorI;
  using typename Base::Vectori;
  using typename Base::Matrix;
  using typename Base::MatrixP;

  using Kernel = std::conditional_t<dim == 2,
                                    MPMKernel<dim, mpm_kernel_order>,
                                    MPMFastKernel32>;
  using Index = IndexND<dim>;
  using Region = RegionND<dim>;
  using Particle = MPMParticle<dim>;
  static constexpr int log2_size = 12;
  using SparseGrid = SPGrid_Allocator<GridState<dim>, dim, log2_size>;
  using SparseMask = typename SparseGrid::template Array_type<>::MASK;
  using AffinityType = uint32;
  using ParticlePtr = typename ParticleAllocator<dim>::ParticlePtr;

  // No need to serialize grid
  using PageMap = SPGrid_Page_Map<log2_size>;
  std::unique_ptr<PageMap> page_map;
  std::unique_ptr<PageMap> rigid_page_map;
  std::unique_ptr<PageMap> fat_page_map;
  std::unique_ptr<SparseGrid> grid;

  /***************************************************************
   * Serialized
   * Do not change ordering of this part.
   */
  real penalty;
  VectorI res;
  Region grid_region;
  Vector gravity;
  bool apic;
  real delta_x;
  real inv_delta_x;
  real affine_damping;
  real base_delta_t;
  real cfl;
  real request_t = 0.0f;
  real pushing_force;
  real rpic_damping;
  real apic_damping;
  bool particle_gravity;
  uint32 frame_count = 0;
  int reorder_interval;
  int substep_counter = 0;
  int step_counter = 0;
  uint64 update_counter = 0;
  real sound_smoothed = 0;
  uint64 cutting_counter;
  uint64 plasticity_counter;
  Config config_backup;
  int spgrid_size;

  std::vector<ParticlePtr> particles;
  std::vector<ParticlePtr> particles_;
  std::vector<uint64> particle_sorter;
  std::vector<real> rigid_block_fractions;

  struct BlockMeta {
    uint32 particle_offset;
    uint32 flags;
    TC_IO_DECL {
      TC_IO(particle_offset);
      TC_IO(flags);
    }
  };
  std::vector<BlockMeta> block_meta;
  std::vector<std::unique_ptr<RigidBody<dim>>> rigids;
  std::vector<std::unique_ptr<Articulation<dim>>> articulations;
  ParticleAllocator<dim> allocator;

  TC_IO_DECL_VIRT {
    Base::io(serializer);
    TC_IO(penalty);
    TC_IO(res);
    TC_IO(grid_region);
    TC_IO(gravity);
    TC_IO(apic);
    TC_IO(delta_x);
    TC_IO(inv_delta_x);
    TC_IO(affine_damping);
    TC_IO(base_delta_t);
    TC_IO(cfl);
    TC_IO(request_t);
    TC_IO(pushing_force);
    TC_IO(rpic_damping);
    TC_IO(apic_damping);
    TC_IO(particle_gravity);
    TC_IO(frame_count);
    TC_IO(reorder_interval);
    TC_IO(substep_counter);
    TC_IO(step_counter);
    TC_IO(update_counter);
    TC_IO(sound_smoothed);
    TC_IO(cutting_counter);
    TC_IO(plasticity_counter);
    TC_IO(config_backup);
    TC_IO(spgrid_size);
    TC_IO(particles);
    TC_IO(particles_);
    TC_IO(block_meta);
    TC_IO(particle_sorter);
    TC_IO(rigid_block_fractions);
    TC_IO(rigids);
    TC_IO(articulations);
    TC_IO(allocator);
  }

  bool test() const override;

  void resample();

  void resample_optimized();

  void rasterize(real delta_t, bool with_force = true);

  void rasterize_optimized(real delta_t);

  void gather_cdf();

  void rasterize_rigid_boundary();

  void normalize_grid_and_apply_external_force(Vector velocity_increment);

  real calculate_energy();

  void apply_grid_boundary_conditions(const DynamicLevelSet<dim> &levelset,
                                      real t);

  void apply_dirichlet_boundary_conditions();

  TC_FORCE_INLINE real &grid_mass(const Vectori &ind) {
    return get_grid(ind).velocity_and_mass[dim];
  }

  TC_FORCE_INLINE real &grid_mass(const Index &ind) {
    return get_grid(ind).velocity_and_mass[dim];
  }

  TC_FORCE_INLINE Vector grid_velocity(const Vectori &ind) const {
    return Vector(get_grid(ind).velocity_and_mass);
  }

  TC_FORCE_INLINE Vector grid_velocity(const Index &ind) const {
    return Vector(get_grid(ind).velocity_and_mass);
  }

  void particle_collision_resolution(real t);

  void rigid_body_levelset_collision(real t, real dt);

  void substep();

  template <typename T>
  void parallel_for_each_particle(const T &target) {
    ThreadedTaskManager::run((int)particles.size(), this->num_threads,
                             [&](int i) { target(*allocator[particles[i]]); });
  }

 public:
  MPM() {
  }

  virtual void initialize(const Config &config) override;

  virtual std::string add_particles(const Config &config) override;

  virtual void step(real dt) override;

  std::vector<RenderParticle> get_render_particles() const override;

  void clear_boundary_particles();

  void rigidify(real dt);

  void advect_rigid_bodies(real dt);

  TC_FORCE_INLINE bool has_rigid_body() const {
    return rigids.size() > 1;
  }

  TC_FORCE_INLINE Vectori get_grid_base_pos(const Vector &pos) const {
    return Vectori(
        [&](int i) -> int { return Kernel::get_stencil_start(pos[i]); });
  }

  template <int kernel_size>
  Vectori get_grid_base_pos_with(const Vector &pos) const {
    return Vectori([&](int i) -> int {
      return MPMKernel<dim, kernel_size - 1>::get_stencil_start(pos[i]);
    });
  }

  virtual void visualize() const override;

  void write_partio(const std::string &file_name) const;

  void write_rigid_body(RigidBody<dim> const *rigid,
                        const std::string &file_name) const;

  virtual void add_rigid_particle(Config config);

  virtual std::string get_debug_information() override;

  std::unique_ptr<RigidBody<dim>> create_rigid_body(Config config);

  bool near_boundary(const Particle &p) const {
    auto pos = p.pos * inv_delta_x;
    real bound = 7.0_f;
    if (pos.min() < bound || (pos - res.template cast<real>()).max() > -bound) {
      return true;
    }
    return false;
  }

  void articulate(real delta_t) {
    int articulation_iterations =
        config_backup.get("articulation_iterations", 100);
    if (config_backup.get("sand_climb", false)) {
      /*
      bool first = true;
      pair<real, real> pos_min, pos_max;
      for (auto &p_i : particles) {
        auto &p = *allocator[p_i];
        if (!p.is_rigid())
          continue;
        auto v = p.get_debug_info();
        if (v[0] == 0)
          continue;
        if (first) {
          pos_min = pos_max = make_pair(p.pos[2], p.pos[0]);
          first = false;
        } else {
          pos_min = min(pos_min, make_pair(p.pos[2], p.pos[0]));
          pos_max = max(pos_max, make_pair(p.pos[2], p.pos[0]));
        }
      }
      TC_WARN("Rudder Info : ({}, {})  ({}, {})", pos_min.first, pos_min.second,
              pos_max.first, pos_max.second);
      for (auto &art : articulations) {
        art->update_parameters(this->current_t, pos_min.second - pos_max.second,
                               (pos_min.first + pos_max.first) * 0.5);
      }
      */
    }
    for (auto &art : articulations) {
      art->apply(delta_t);
    }
    for (int i = 0; i < articulation_iterations; i++) {
      for (auto &art : articulations) {
        art->project();
      }
    }
    for (auto &art : articulations) {
      art->penalize(delta_t);
    }
  }

  virtual std::string general_action(const Config &config) override;

  void draw_cdf(const Config &config);

  RigidBody<dim> *get_rigid_body_ptr(int id) {
    if (id == -1) {
      return nullptr;
    } else {
      return rigids[id].get();
    }
  }

  void write_bgeo() const {
    ++(const_cast<MPM<dim> *>(this)->frame_count);
    std::string directory = config_backup.get_string("frame_directory");
    std::string filename = fmt::format("{}/{:04}.bgeo", directory, frame_count);
    write_partio(filename);
    // Start from 1. (0 is the background rigid body.)
    for (int i = 1; i < (int)rigids.size(); i++) {
      filename = fmt::format("{}/rigid_{:03}_{:04}", directory, i, frame_count);
      write_rigid_body(rigids[i].get(), filename);
    }
  }

  TC_FORCE_INLINE GridState<dim> &get_grid(const Vectori &i) {
    return grid->Get_Array()(to_std_array(i));
  }

  TC_FORCE_INLINE GridState<dim> &get_grid(const Index &i) {
    return get_grid(i.get_ipos());
  }

  TC_FORCE_INLINE const GridState<dim> &get_grid(const Vectori &i) const {
    return grid->Get_Array()(to_std_array(i));
  }

  TC_FORCE_INLINE const GridState<dim> &get_grid(const Index &i) const {
    return get_grid(i.get_ipos());
  }

  virtual void sort_allocator();

  void sort_particles_and_populate_grid();

  template <typename T>
  void parallel_for_each_active_grid(const T &target, bool fat = true) {
    std::pair<const uint64_t *, unsigned> blocks;
    if (fat)
      blocks = fat_page_map->Get_Blocks();
    else
      blocks = page_map->Get_Blocks();
    auto grid_array = grid->Get_Array();
    ThreadedTaskManager::run((int)blocks.second, this->num_threads, [&](int b) {
      GridState<dim> *g =
          reinterpret_cast<GridState<dim> *>(&grid_array(blocks.first[b]));
      for (int i = 0; i < (int)SparseMask::elements_per_block; i++) {
        target(g[i]);
      }
    });
  }

  template <typename T>
  void parallel_for_each_active_block(const T &target, bool fat = true) {
    std::pair<const uint64_t *, unsigned> blocks;
    if (fat)
      blocks = fat_page_map->Get_Blocks();
    else
      blocks = page_map->Get_Blocks();
    auto grid_array = grid->Get_Array();
    ThreadedTaskManager::run((int)blocks.second, this->num_threads, [&](int b) {
      GridState<dim> *g =
          reinterpret_cast<GridState<dim> *>(&grid_array(blocks.first[b]));
      target(g);
    });
  }

  template <typename T>
  void parallel_for_each_block_with_index(const std::unique_ptr<PageMap> &page_map,
                                          const T &target,
                                          bool colored = false) {
    std::pair<const uint64_t *, unsigned> blocks = page_map->Get_Blocks();
    auto grid_array = grid->Get_Array();
    if (!colored) {
      ThreadedTaskManager::run(
          (int)blocks.second, this->num_threads, [&](int b) {
            GridState<dim> *g = reinterpret_cast<GridState<dim> *>(
                &grid_array(blocks.first[b]));
            target(b, blocks.first[b], g);
          });
    } else {
      for (int i = 0; i < (1 << dim); i++) {
        ThreadedTaskManager::run(
            (int)blocks.second, this->num_threads, [&](int b) {
              Vectori v(SparseMask::LinearToCoord(blocks.first[b]));
              GridState<dim> *g = reinterpret_cast<GridState<dim> *>(
                  &grid_array(blocks.first[b]));
              Vectori bs = grid_block_size();
              for (int k = 0; k < dim; k++) {
                if ((v[k] / bs[k]) % 2 != (i >> k) % 2) {
                  return;
                }
              }
              target(b, blocks.first[b], g);
            });
      }
    }
  }

  template <typename T>
  void parallel_for_each_block_with_index(const T &target,
                                          bool fat,
                                          bool colored = false) {
    std::pair<const uint64_t *, unsigned> blocks;
    if (fat)
      blocks = fat_page_map->Get_Blocks();
    else
      blocks = page_map->Get_Blocks();
    auto grid_array = grid->Get_Array();
    if (!colored) {
      ThreadedTaskManager::run(
          (int)blocks.second, this->num_threads, [&](int b) {
            GridState<dim> *g = reinterpret_cast<GridState<dim> *>(
                &grid_array(blocks.first[b]));
            target(b, blocks.first[b], g);
          });
    } else {
      for (int i = 0; i < (1 << dim); i++) {
        ThreadedTaskManager::run(
            (int)blocks.second, this->num_threads, [&](int b) {
              Vectori v(SparseMask::LinearToCoord(blocks.first[b]));
              GridState<dim> *g = reinterpret_cast<GridState<dim> *>(
                  &grid_array(blocks.first[b]));
              Vectori bs = grid_block_size();
              for (int k = 0; k < dim; k++) {
                if ((v[k] / bs[k]) % 2 != (i >> k) % 2) {
                  return;
                }
              }
              target(b, blocks.first[b], g);
            });
      }
    }
  }

  Matrix damp_affine_momemtum(const Matrix &b) {
    auto b_sym = 0.5_f * (b + b.transposed());
    auto b_skew = b - b_sym;
    return (1 - rpic_damping) * b_sym + (1 - apic_damping) * b_skew;
  }

  ~MPM();

  Vectori grid_block_size() const {
    Vectori ret;
    ret[0] = 1 << SparseMask::block_xbits;
    ret[1] = 1 << SparseMask::block_ybits;
    TC_STATIC_IF(dim == 3) {
      ret[2] = 1 << SparseMask::block_zbits;
    }
    TC_STATIC_END_IF
    return ret;
  }

  void update_rigid_page_map();

  std::string get_name() const override {
    return "mpm";
  }
};

TC_NAMESPACE_END
