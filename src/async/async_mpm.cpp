/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#include "async_mpm.h"

#include <taichi/system/profiler.h>
#include <taichi/testing.h>

TC_NAMESPACE_BEGIN

template <int dim>
void AsyncMPM<dim>::initialize(const Config &config) {
  MPM<dim>::initialize(config);

  scheduler_size = pow<dim>(this->spgrid_size) / this->grid_block_size().prod();
  TC_ASSERT(bit::is_power_of_two(scheduler_size));
  assert(scheduler_size <= max_scheduler_size);
  TC_INFO("scheduler_size = 2^{}", bit::log2int(scheduler_size));
  scheduler_mask = scheduler_size - 1;

  unit_delta_t = config.get("unit_delta_t", 1e-6_f);
  max_units = config.get("max_units", 8192);
  cfl_dt_mul = config.get("cfl_dt_mul", 1.0_f);
  strength_dt_mul = config.get("strength_dt_mul", 1.0_f);

  blocks.resize(scheduler_size);
  memset(&blocks[0], 0, sizeof(BlockInfo) * blocks.size());

  for (uint64 offset = 0; offset < scheduler_size; ++offset) {
    blocks[offset].strength_dt_limit = 1LL << 31;
    blocks[offset].cfl_dt_limit = 1LL << 31;
    blocks[offset].continuous_dt_limit = 1;
    blocks[offset].local_min_dt_limit = 1;
  }

  uint64 max_offset = SparseMask::Linear_Offset(this->res) >>
                      SparseMask::data_bits >> SparseMask::block_bits;
  TC_DEBUG("Max Offset : {}, Max Mask : {}", max_offset, scheduler_mask);
  TC_ASSERT_INFO(max_offset <= scheduler_mask,
                 "Please rescale scheduler_size.");
  TC_TRACE("AsyncMPM initialize successfully.");

  if (this->config_backup.get("left_boundary", false)) {
    for (uint64 offset = 0; offset < scheduler_size; ++offset) {
      uint64 address = uint64(offset)
                       << SparseMask::data_bits << SparseMask::block_bits;
      auto c = SparseMask::LinearToCoord(address);
      Vector pos;
      for (int i = 0; i < dim; ++i)
        pos[i] = (real)c[i] / this->res[i];
      if (0.0_f <= pos[0] && pos[0] <= 0.2_f)
        boundary.push_back(offset);
    }
  }

  precompute_neighbor_pairs();
}

template <int dim>
std::string AsyncMPM<dim>::add_particles(const Config &config) {
  std::string ret = MPM<dim>::add_particles(config);
  for (auto p : this->particles) {
    uint64 grid_offset = SparseMask::Linear_Offset(MPM<dim>::get_grid_base_pos(
        (reinterpret_cast<Particle *>(&this->allocator.pool[p]))->pos *
        this->inv_delta_x));
    uint64 offset =
        (grid_offset >> SparseMask::data_bits >> SparseMask::block_bits) &
        scheduler_mask;
    particle_pool[offset].push_back(this->allocator.pool[p]);
  }
  this->particles.clear();
  return ret;
}

TC_FORCE_INLINE float32 inv_sqrt(float32 v) {
  auto tmp = _mm_rsqrt_ss(_mm_set1_ps(v));
  return *reinterpret_cast<float32 *>(&tmp);
}

TC_TEST("inv_sqrt") {
  for (int i = 0; i < 1000; i++) {
    auto x = rand() * 10 + 1;
    auto y = inv_sqrt(x);
    TC_CHECK_EQUAL(x * y * y, 1.0_f, 1e-3f);
  }
}

template <int dim>
void AsyncMPM<dim>::update_dt_limits() {
  {
    Profiler _("update dt_limits for non-empty blocks");
    real inv_unit_delta_t = 1.0_f / unit_delta_t;
    ThreadedTaskManager::run(
        scheduler_size, this->num_threads, [&](int offset) {
          if ((current_t_int & (blocks[offset].continuous_dt_limit - 1)) == 0 &&
              !particle_pool[offset].empty()) {
            const auto &pool = particle_pool[offset];
            int64 &sdl = blocks[offset].strength_dt_limit;
            int64 &cdl = blocks[offset].cfl_dt_limit;
            int64 &limit = blocks[offset].continuous_dt_limit;
            // MAX ALLOWED DT
            real min_allowed_dt = 0.1_f;
            real max_abs_v = 1e-16_f;
            for (const auto &container : pool) {
              auto *p = reinterpret_cast<const Particle *>(&container);
              min_allowed_dt =
                  std::min(min_allowed_dt, p->get_allowed_dt(this->delta_x));
              max_abs_v = std::max(max_abs_v, p->get_velocity().length2());
            }
            sdl = int64(strength_dt_mul * min_allowed_dt * inv_unit_delta_t);
            cdl = int64(cfl_dt_mul * this->delta_x * inv_unit_delta_t *
                        inv_sqrt(max_abs_v));
            int64 tmp_limit = min(min(blocks[offset].cfl_dt_limit,
                                      blocks[offset].strength_dt_limit),
                                  max_units);
            if (tmp_limit < 1) {
              for (const auto &container : pool) {
                auto *p = reinterpret_cast<const Particle *>(&container);
                TC_WARN("{} particle {}", p->get_name(),
                        p->get_allowed_dt(this->delta_x));
              }
              TC_STOP;
            }
            while (tmp_limit < limit) {
              limit >>= 1;
            }
            while (tmp_limit >= (limit << 1) &&
                   (current_t_int & ((limit << 1) - 1)) == 0) {
              limit <<= 1;
            }
          }
        });
    update_dt_limit_boundary(true);
  }
  {
    Profiler _("update dt_limits for empty blocks");
    ThreadedTaskManager::run(
        scheduler_size, this->num_threads, [&](int offset) {
          if ((current_t_int & (blocks[offset].continuous_dt_limit - 1)) == 0 &&
              particle_pool[offset].empty()) {
            int64 &limit = blocks[offset].continuous_dt_limit;
            while (max_delta_t_int < limit) {
              limit >>= 1;
            }
            while ((max_delta_t_int >= (limit << 1)) &&
                   (current_t_int & ((limit << 1) - 1)) == 0) {
              limit <<= 1;
            }
          }
        });
    update_dt_limit_boundary(false);
  }
  {
    Profiler _("update dt_limits for boundary blocks");
    for (auto offset : boundary)
      if ((current_t_int & (blocks[offset].continuous_dt_limit - 1)) == 0) {
        int64 &limit = blocks[offset].continuous_dt_limit;
        while (min_delta_t_int < limit) {
          limit >>= 1;
        }
      }
  }
  {
    Profiler _("calculate local_min_dt_limit for each block");
    tbb::parallel_for(
        tbb::blocked_range<int>(0, scheduler_size),
        [&](const tbb::blocked_range<int> &r) {
          for (int offset = r.begin(); offset != r.end(); offset++) {
            TC_ASSERT_INFO(blocks[offset].continuous_dt_limit < (1LL << 31),
                           "Empty blocks are in wrong state");
            if (blocks[offset].continuous_dt_limit == min_delta_t_int)
              continue;
            blocks[offset].local_min_dt_limit = 1LL << 31;
            for (auto nearby_offset : cached_neighbours[offset]) {
              blocks[offset].local_min_dt_limit =
                  min(blocks[offset].local_min_dt_limit,
                      blocks[nearby_offset].particle_t +
                          blocks[nearby_offset].continuous_dt_limit);
            }
          }
        });
  }
  {
    Profiler _("precomputed larger_neighbours smaller_neighbours 1");
    // while (true) {
    // Time::Timer _("precomputed larger_neighbours smaller_neighbours 1");
    std::vector<int> cached_log2(scheduler_size, 0);
    for (uint64 log2 = 0; log2 < max_log2; ++log2) {
      larger_neighbours[log2].clear();
      smaller_neighbours[log2].clear();
    }
    for (uint64 offset = 0; offset < scheduler_size; ++offset)
      cached_log2[offset] = quick_log2(blocks[offset].continuous_dt_limit);

    using LocalStorage =
        std::array<std::array<std::vector<uint32>, max_log2>, 2>;

    tbb::enumerable_thread_specific<LocalStorage> local_storage;

    tbb::parallel_for(
        tbb::blocked_range<uint64>(0, scheduler_size),
        [&](const tbb::blocked_range<uint64> &r) {
          auto &s = local_storage.local();
          for (auto offset = r.begin(); offset != r.end(); offset++) {
            for (auto nearby_offset : cached_neighbours[offset]) {
              if (blocks[offset].continuous_dt_limit <
                  blocks[nearby_offset].continuous_dt_limit) {
                // larger_neighbours[cached_log2[offset]].push_back(nearby_offset);
                s[0][cached_log2[offset]].push_back(nearby_offset);
                // smaller_neighbours[cached_log2[nearby_offset]].push_back(offset);
                s[1][cached_log2[nearby_offset]].push_back(offset);
              }
            }
          }
        });

    for (int i = 0; i < (int)max_log2; i++) {
      for (auto &s : local_storage) {
        larger_neighbours[i].insert(larger_neighbours[i].end(), s[0][i].begin(),
                                    s[0][i].end());
        smaller_neighbours[i].insert(smaller_neighbours[i].end(),
                                     s[1][i].begin(), s[1][i].end());
      }
    }

    /*
    for (uint64 offset = 0; offset < scheduler_size; ++offset)
      for (auto nearby_offset : cached_neighbours[offset])
        if (blocks[offset].continuous_dt_limit <
    blocks[nearby_offset].continuous_dt_limit) {
          larger_neighbours[cached_log2[offset]].push_back(nearby_offset);
          smaller_neighbours[cached_log2[nearby_offset]].push_back(offset);
        }
    */
  }
  {
    Profiler _("precomputed larger_neighbours smaller_neighbours 2");
    for (uint64 log2 = 0; log2 < max_log2; ++log2) {
      sort(larger_neighbours[log2].begin(), larger_neighbours[log2].end());
      larger_neighbours[log2].erase(unique(larger_neighbours[log2].begin(),
                                           larger_neighbours[log2].end()),
                                    larger_neighbours[log2].end());
      sort(smaller_neighbours[log2].begin(), smaller_neighbours[log2].end());
      smaller_neighbours[log2].erase(unique(smaller_neighbours[log2].begin(),
                                            smaller_neighbours[log2].end()),
                                     smaller_neighbours[log2].end());
    }
    if (particles_cnt.size() < this->allocator.pool.size()) {
      particles_cnt.resize(this->allocator.pool.size());
    }
  }
}

template <int dim>
void AsyncMPM<dim>::advance(int64 limit) {
  std::vector<uint8> has_copied(scheduler_size, 0);
  {
    TC_PROFILER("gather_from_pool");

    ++global_cnt;

    {
      TC_PROFILER("Smaller neighbours");
      for (auto nearby_offset : smaller_neighbours[quick_log2(limit)]) {
        TC_ASSERT_INFO(blocks[nearby_offset].particle_t == current_t_int,
                       "particle_pool broken 2");
        has_copied[nearby_offset] = true;
        gather_from_pool(particle_pool[nearby_offset]);
      }
    }

    tbb::enumerable_thread_specific<std::vector<ParticlePtr>> local_vectors;

    {
      TC_PROFILER("Equal neighbours (scatter)");
      tbb::parallel_for(
          tbb::blocked_range<uint64>(0, scheduler_size),
          [&](const tbb::blocked_range<uint64> &r) {
            auto &vec = local_vectors.local();
            for (uint64 offset = r.begin(); offset < r.end(); ++offset) {
              if (blocks[offset].continuous_dt_limit == limit) {
                TC_ASSERT_INFO(blocks[offset].particle_t == current_t_int,
                               "particle_pool broken 1");
                gather_from_pool(particle_pool[offset], vec);
              }
            }
          });
    }

    {
      TC_PROFILER("Larger neighbours");
      // for (auto nearby_offset : larger_neighbours[quick_log2(limit)]) {
      const auto &neighbours = larger_neighbours[quick_log2(limit)];
      tbb::parallel_for(tbb::blocked_range<std::size_t>(0, neighbours.size()),
                        [&](const tbb::blocked_range<std::size_t> &r) {
                          auto &vec = local_vectors.local();
                          for (uint64 i = r.begin(); i < r.end(); ++i) {
                            auto nearby_offset = neighbours[i];
                            TC_ASSERT_INFO(
                                blocks[nearby_offset].backup_t == current_t_int,
                                "backup_pool broken");
                            has_copied[nearby_offset] = true;
                            gather_from_pool(backup_pool[nearby_offset], vec);
                          }
                        });
    }

    {
      TC_PROFILER("Gather");
      for (auto &vec : local_vectors) {
        this->particles.insert(this->particles.end(), vec.begin(), vec.end());
      }
    }
  }
  {
    Profiler _("backup_current_dt_limit");
    for (uint64 offset = 0; offset < scheduler_size; ++offset)
      if (blocks[offset].continuous_dt_limit == limit &&
          blocks[offset].particle_t == current_t_int) {
        backup_pool[offset].clear();
        backup_pool[offset].swap(particle_pool[offset]);
        blocks[offset].backup_t = current_t_int;
      }
  }

  this->update_counter += this->particles.size();

  MPM<dim>::substep();

  {
    Profiler _("update backup_t and particle_t");
    for (uint64 offset = 0; offset < scheduler_size; ++offset) {
      if (blocks[offset].continuous_dt_limit == limit) {
        blocks[offset].particle_t = current_t_int + limit;
      } else if (has_copied[offset] &&
                 blocks[offset].continuous_dt_limit > limit &&
                 blocks[offset].local_min_dt_limit == current_t_int + limit) {
        backup_pool[offset].clear();
        blocks[offset].backup_t = current_t_int + limit;
      }
    }
  }

  {
    Profiler _("update backup_pool and particle_pool");
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, this->particles.size()),
        [&](const tbb::blocked_range<std::size_t> &r) {
          for (std::size_t i = r.begin(); i != r.end(); i++) {
            auto p = this->particles[i];
            uint64 grid_offset =
                SparseMask::Linear_Offset(MPM<dim>::get_grid_base_pos(
                    this->allocator[p]->pos * this->inv_delta_x));
            uint64 offset = (grid_offset >> SparseMask::data_bits >>
                             SparseMask::block_bits) &
                            scheduler_mask;
            if (blocks[offset].continuous_dt_limit == limit) {
              // TODO: why can't we use emplace_back here with clang++-7?
              particle_pool[offset].push_back(this->allocator.pool[p]);
            } else if (has_copied[offset] &&
                       blocks[offset].continuous_dt_limit > limit &&
                       blocks[offset].local_min_dt_limit ==
                           current_t_int + limit) {
              backup_pool[offset].push_back(this->allocator.pool[p]);
            }
          }
        });
  }

  this->particles.clear();
}

template <int dim>
void AsyncMPM<dim>::step(real dt) {
  if (dt < 0) {
    MPM<dim>::substep();
    this->request_t = this->current_t;
  } else {
    this->request_t += dt;
    do {
      TC_PROFILE("update_dt_limits", update_dt_limits());
      if (this->config_backup.get("print_energy", false)) {
        static real last_output = 0.0_f;
        if (this->current_t > last_output + dt &&
            current_t_int % max_delta_t_int == 0) {
          last_output += dt;
          TC_ASSERT(this->particles.empty());
          for (uint64 offset = 0; offset < scheduler_size; ++offset)
            gather_all_from_pool(particle_pool[offset]);
          std::sort(this->particles.begin(), this->particles.end());
          this->particles.resize(
              std::unique(this->particles.begin(), this->particles.end()) -
              this->particles.begin());
          TC_P(this->particles.size());
          TC_P(this->calculate_energy());
          /*
          if (this->current_t > 0.5) {
            this->visualize();
            exit(0);
          }
          */
          this->particles.clear();
        }
      }
      for (int64 delta_t_int = max_delta_t_int; delta_t_int >= min_delta_t_int;
           delta_t_int >>= 1)
        if (this->current_t_int % delta_t_int == 0) {
          this->current_t = this->unit_delta_t * current_t_int;
          this->base_delta_t = this->unit_delta_t * delta_t_int;
          TC_PROFILE("advance", advance(delta_t_int));
        }
      current_t_int += min_delta_t_int - current_t_int % min_delta_t_int;
      this->current_t = this->unit_delta_t * current_t_int;

    } while (this->current_t < this->request_t);
  }
  this->step_counter += 1;
  TC_WARN("Times of particle updating : {}", this->update_counter);
}

using AsyncMPM2D = AsyncMPM<2>;
using AsyncMPM3D = AsyncMPM<3>;

TC_IMPLEMENTATION(Simulation2D, AsyncMPM2D, "async_mpm");
TC_IMPLEMENTATION(Simulation3D, AsyncMPM3D, "async_mpm");

TC_NAMESPACE_END
