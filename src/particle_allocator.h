/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#pragma once

#include <taichi/common/util.h>

#include "mpm_fwd.h"
#include "particles.h"

TC_NAMESPACE_BEGIN

template <typename T>
inline T *remove_const(const T *v) {
  return const_cast<T *>(v);
}

template <int dim>
struct ParticleContainer {
  uint8 data[get_particle_size_upper_bound<dim>()];
  ParticleContainer() {
    memset(data, 0, sizeof(data));
  }
  ParticleContainer(const ParticleContainer &container) {
    memcpy(&data, &container.data, get_particle_size_upper_bound<dim>());
  }
  TC_IO_DEF(data);
};

template <int dim>
class ParticleAllocator {
 public:
  using Particle = MPMParticle<dim>;
  using ParticlePtr = uint32;

  uint64 particle_counter = 0;
  std::vector<ParticleContainer<dim> > pool;
  std::vector<ParticleContainer<dim> > pool_;

  TC_IO_DECL {
    TC_IO(particle_counter);
    if (TC_SERIALIZER_IS(BinaryOutputSerializer)) {
      // Output
      std::size_t n = pool.size();
      serializer(n);
      for (std::size_t i = 0; i < n; i++) {
        auto p = remove_const(this)->operator[](i);
        serializer(p->get_name());
        p->binary_io(serializer);
      }
    } else {
      // Input
      std::size_t n;
      serializer(n);
      remove_const(this)->pool.resize(n);
      for (std::size_t i = 0; i < n; i++) {
        std::string name;
        serializer(name);
        auto p = remove_const(this)->operator[](i);
        create_instance_placement<Particle>(name, p);
        p->binary_io(serializer);
      }
    }
  }

  std::pair<ParticlePtr, Particle *> allocate_particle(std::string alias) {
    pool.emplace_back();
    auto index = ParticlePtr(pool.size()) - 1;
    Particle *p = create_instance_placement<Particle>(alias, &pool[index]);
    p->id = particle_counter++;
    return std::make_pair(index, p);
  }

  Particle *operator[](const ParticlePtr &ptr) {
    return reinterpret_cast<Particle *>(&pool[ptr]);
  }

  const Particle *get_const(const ParticlePtr &ptr) const {
    return reinterpret_cast<const Particle *>(&pool[ptr]);
  }

  void gc(std::size_t particle_count) {
    pool.resize(particle_count);
    pool_.resize(particle_count);
  }
};

TC_NAMESPACE_END
