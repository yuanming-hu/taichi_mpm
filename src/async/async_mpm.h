/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#pragma once

#include "../mpm.h"

#include <algorithm>

TC_NAMESPACE_BEGIN

template <typename T>
struct SmallVector {
  int tail;
  T data[31];  // 32 ints in total if T is uint32; 128 B = 2 cachelines

  SmallVector() {
    tail = 0;
  }

  TC_FORCE_INLINE void push_back(const T &t) {
    data[tail++] = t;
  }
  TC_FORCE_INLINE std::size_t size() const {
    return (std::size_t)tail;
  }
  TC_FORCE_INLINE void resize(std::size_t s) const {
    tail = (int)s;
  }
  TC_FORCE_INLINE const T *begin() const {
    return &data[0];
  }
  TC_FORCE_INLINE const T *end() const {
    return &data[tail];
  }

  TC_FORCE_INLINE T *begin() {
    return &data[0];
  }
  TC_FORCE_INLINE T *end() {
    return &data[tail];
  }

  void sort_and_unique() {
    TC_ERROR_IF(tail >= 32, "Small Vector Overflow");
    std::sort(begin(), end());
    tail = (int)(std::unique(begin(), end()) - begin());
  }
};

template <int dim>
class AsyncMPM : public MPM<dim> {
  static constexpr uint64 max_scheduler_size = (1UL << 21);
  static constexpr uint64 max_log2 = 20;

  uint64 scheduler_size;
  uint64 scheduler_mask;

  using Base = MPM<dim>;
  using Container = ParticleContainer<dim>;
  using Base::log2_size;

  using typename Base::Vector;
  using typename Base::SparseGrid;
  using typename Base::SparseMask;
  using typename Base::Particle;
  using typename Base::ParticlePtr;

  real cfl_dt_mul;
  real strength_dt_mul;
  real unit_delta_t;
  int64 max_units;
  int64 current_t_int = 0;
  int64 min_delta_t_int;
  int64 max_delta_t_int;

  tbb::concurrent_vector<Container> particle_pool[max_scheduler_size];
  tbb::concurrent_vector<Container> backup_pool[max_scheduler_size];

  std::vector<Container> particle_pool_tmp[max_scheduler_size];
  std::vector<Container> backup_pool_tmp[max_scheduler_size];

  struct BlockInfo {
    int64 strength_dt_limit;
    int64 cfl_dt_limit;
    int64 continuous_dt_limit;
    int64 local_min_dt_limit;
    int cached_log2;
    bool has_copied;
    int64 particle_t;
    int64 backup_t;
    TC_IO_DEF(strength_dt_limit,
              cfl_dt_limit,
              continuous_dt_limit,
              local_min_dt_limit,
              cached_log2,
              has_copied,
              particle_t,
              backup_t);
  };
  std::vector<BlockInfo> blocks;

  int64 visualize_state_address[max_scheduler_size];
  SmallVector<uint32> cached_neighbours[max_scheduler_size];  // Note: use
                                                              // 32-bit instead
                                                              // of 64-bit

  std::vector<uint32> larger_neighbours[max_log2];
  std::vector<uint32> smaller_neighbours[max_log2];

  std::vector<uint64> boundary;

  std::vector<int> particles_cnt;
  int global_cnt;

  // TODO: simplify here
  virtual void binary_io(BinaryOutputSerializer &ser) const override {
    this->io(ser);
  }
  virtual void binary_io(BinaryInputSerializer &ser) const override {
    this->io(ser);
  }

  // TC_IO_DECL_VIRT_OVERRIDE {
  template <typename S>
  void io(S &serializer) const {
    return;
    TC_NOT_IMPLEMENTED  // Loading container will lead to wrong vtable address!
        TC_WARN("Async IO");
    Base::io(serializer);
    TC_WARN("Back Async IO");
    TC_IO(cfl_dt_mul, strength_dt_mul, unit_delta_t, max_units, current_t_int,
          min_delta_t_int, max_delta_t_int);
    TC_WARN("Back Async IO");
    auto *cthis = const_cast<AsyncMPM *>(this);
    if
      TC_SERIALIZER_IS(BinaryInputSerializer) {
        for (uint64 i = 0; i < max_scheduler_size; i++) {
          TC_IO(particle_pool_tmp[i]);
          TC_IO(backup_pool_tmp[i]);
        }
        TC_WARN("Back Async IO");
        for (uint64 i = 0; i < max_scheduler_size; i++) {
          cthis->particle_pool[i] = tbb::concurrent_vector<Container>(
              particle_pool_tmp[i].begin(), particle_pool_tmp[i].end());
          cthis->backup_pool[i] = tbb::concurrent_vector<Container>(
              backup_pool_tmp[i].begin(), backup_pool_tmp[i].end());
        }
      }
    else {
      for (uint64 i = 0; i < max_scheduler_size; i++) {
        cthis->particle_pool_tmp[i] = std::vector<Container>(
            particle_pool[i].begin(), particle_pool[i].end());
        cthis->backup_pool_tmp[i] = std::vector<Container>(
            backup_pool[i].begin(), backup_pool[i].end());
      }
      for (uint64 i = 0; i < max_scheduler_size; i++) {
        TC_IO(particle_pool_tmp[i]);
        TC_IO(backup_pool_tmp[i]);
      }
    }
    TC_IO(blocks);
    TC_IO(visualize_state_address);
    TC_IO(cached_neighbours);
    TC_IO(larger_neighbours);
    TC_IO(smaller_neighbours);
    TC_IO(boundary);
    TC_IO(particles_cnt);
    TC_IO(global_cnt);
  }

 public:
  AsyncMPM() {
  }

  virtual void initialize(const Config &config) override;

  virtual void sort_allocator() override {
  }

  virtual void visualize() const override;

  void update_dt_limit_boundary(bool non_empty) {
    min_delta_t_int = (1LL << 31);
    max_delta_t_int = 1;
    for (uint64 offset = 0; offset < scheduler_size; ++offset) {
      if (non_empty && particle_pool[offset].empty())
        continue;
      min_delta_t_int =
          std::min(min_delta_t_int, blocks[offset].continuous_dt_limit);
      max_delta_t_int =
          std::max(max_delta_t_int, blocks[offset].continuous_dt_limit);
    }
  }

  void gather_from_pool(const tbb::concurrent_vector<Container> &pool) {
    for (const auto &container : pool) {
      int id = (reinterpret_cast<const Particle *>(&container))->id;
      TC_ASSERT_INFO(id == this->allocator[id]->id, "Allocator for MPM broke.");
      if (particles_cnt[id] != global_cnt) {
        particles_cnt[id] = global_cnt;
        memcpy(&this->allocator.pool[id], &container,
               (size_t)get_particle_size_upper_bound<dim>());
        this->particles.push_back((ParticlePtr)id);
      }
    }
  }

  void gather_all_from_pool(const tbb::concurrent_vector<Container> &pool) {
    for (const auto &container : pool) {
      int id = (reinterpret_cast<const Particle *>(&container))->id;
      TC_ASSERT_INFO(id == this->allocator[id]->id, "Allocator for MPM broke.");
      particles_cnt[id] = global_cnt;
      memcpy(&this->allocator.pool[id], &container,
             (size_t)get_particle_size_upper_bound<dim>());
      this->particles.push_back((ParticlePtr)id);
    }
  }

  void gather_from_pool(const tbb::concurrent_vector<Container> &pool,
                        std::vector<ParticlePtr> &dest) {
    for (const auto &container : pool) {
      int id = (reinterpret_cast<const Particle *>(&container))->id;
      TC_ASSERT_INFO(id == this->allocator[id]->id, "Allocator for MPM broke.");
      if (particles_cnt[id] != global_cnt) {
        particles_cnt[id] = global_cnt;
        memcpy(&this->allocator.pool[id], &container,
               (size_t)get_particle_size_upper_bound<dim>());
        dest.push_back((ParticlePtr)id);
      }
    }
  }

  void update_dt_limits();
  void advance(int64 limit);

  virtual std::string add_particles(const Config &config) override;
  virtual void step(real dt) override;

  int quick_log2(int64 limit) {
    int ret = 0;
    while (limit > 1LL) {
      limit >>= 1;
      ++ret;
    }
    return ret;
  }

  std::string get_name() const override {
    return "async_mpm";
  }

  void precompute_neighbor_pairs(){TC_STATIC_IF(dim ==
                                                2){ThreadedTaskManager::run(
      (int)scheduler_size,
      this->num_threads,
      [&](int offset) {
        auto x = 1 << SparseMask::block_xbits;
        auto y = 1 << SparseMask::block_ybits;
        uint64 address = uint64(offset)
                         << SparseMask::data_bits << SparseMask::block_bits;
        auto c = SparseMask::LinearToCoord(address);
        SmallVector<uint32> &neighbours = cached_neighbours[offset];
        for (int i = -1 + (c[0] == 0); i < 2; i++) {
          for (int j = -1 + (c[1] == 0); j < 2; j++) {
            uint64 nearby_address = SparseMask::Packed_Add(
                address, SparseMask::Linear_Offset(x * i, y * j));
            uint64 nearby_offset = nearby_address >> SparseMask::data_bits >>
                                   SparseMask::block_bits;
            if (uint64(offset) != nearby_offset &&
                nearby_offset == (nearby_offset & scheduler_mask))
              neighbours.push_back(nearby_offset);
          }
        }
        neighbours.sort_and_unique();
      });
} TC_STATIC_ELSE {
  ThreadedTaskManager::run(
      (int)scheduler_size, this->num_threads, [&](int offset) {
        auto x = 1 << SparseMask::block_xbits;
        auto y = 1 << SparseMask::block_ybits;
        auto z = 1 << SparseMask::block_zbits;
        uint64 address = uint64(offset)
                         << SparseMask::data_bits << SparseMask::block_bits;
        auto c = SparseMask::LinearToCoord(address);
        auto &neighbours = cached_neighbours[offset];
        for (int i = -1 + (c[0] == 0); i < 2; i++) {
          for (int j = -1 + (c[1] == 0); j < 2; j++) {
            for (int k = -1 + (c[2] == 0); k < 2; k++) {
              uint64 nearby_address = SparseMask::Packed_Add(
                  address, SparseMask::Linear_Offset(x * i, y * j, z * k));
              uint64 nearby_offset = nearby_address >> SparseMask::data_bits >>
                                     SparseMask::block_bits;
              if (uint64(offset) != nearby_offset &&
                  nearby_offset == (nearby_offset & scheduler_mask))
                neighbours.push_back(nearby_offset);
            }
          }
        }
        neighbours.sort_and_unique();
      });
}
TC_STATIC_END_IF
}

int64 ret[52][52][22];
bool pol[52][52][22];

void debug(int64 *arr) {
  memset(ret, 0, sizeof(ret));
  auto g = this->grid_block_size();
  for (uint64 offset = 0; offset < scheduler_size; ++offset) {
    auto c = SparseMask::LinearToCoord(
        uint64(offset) << SparseMask::data_bits << SparseMask::block_bits);
    int x = c[0] / g[0];
    int y = c[1] / g[1];
    int z = c[2] / g[2];
    if (c[0] > this->res[0] || c[1] > this->res[1] || c[2] > this->res[2])
      continue;
    //      TC_DEBUG("{} {} {}, {} {} {}", c[0], c[1], c[2], g[0], g[1], g[2]);
    ret[x][y][z] = min(arr[offset], int64(9999));
    pol[x][y][z] = !particle_pool[offset].empty();
  }
  //  for (int k = 0; k <= this->res[2] / g[2]; ++k) {
  for (int k = 5; k <= 5; ++k) {
    bool flag = false;
    for (int i = 0; i <= this->res[0] / g[0]; ++i)
      for (int j = 0; j <= this->res[1] / g[1]; ++j)
        if (pol[i][j][k])
          flag = true;
    if (!flag) {
      continue;
    }
    TC_DEBUG("Step {}   Layer {} : ", this->step_counter, k);
    for (int j = this->res[1] / g[1]; j >= 0; --j) {
      for (int i = 0; i <= this->res[0] / g[0]; ++i)
        if (pol[i][j][k])
          printf("[%4lld] ", int64(ret[i][j][k]));
        else
          printf(" %4lld  ", int64(ret[i][j][k]));
      printf("\n");
    }
    printf("\n");
  }
}
}
;

TC_NAMESPACE_END
