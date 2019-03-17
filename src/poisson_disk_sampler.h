/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#pragma once

#include <taichi/util.h>
#include <taichi/math/array.h>
#include <taichi/math/levelset.h>
#include <taichi/system/threading.h>
#include <taichi/visual/texture.h>

TC_NAMESPACE_BEGIN

// PoissonDiskSampler only supports uniform density_texture
template <int dim>
class PoissonDiskSampler {
  using Vector = VectorND<dim, real>;
  using Vectori = VectorND<dim, int>;
  using Index = IndexND<dim>;
  using Region = RegionND<dim>;

  int max_attempts = 30;
  bool periodic = false;
  real min_distance, h;
  Vector periodic_bound = Vector(40.0_f);
  Vector min_corner, max_corner;

  std::vector<float> points_list;
  size_t points_size;

  // get min_corner, max_corner, min_distance
  void get_ready(std::shared_ptr<Texture> density_texture,
                 const RegionND<dim> &region,
                 real dx,
                 real specific_min_distance = -1.0_f) {
    real ppc = 0.0_f;
    min_corner = Vector(1.0_f);
    max_corner = Vector(0.0_f);
    for (auto &ind : region) {
      Vector coord = (ind.get_ipos().template cast<real>() + Vector(0.5f)) * dx;
      real sample = density_texture->sample(coord).x;
      ppc = std::max(sample, ppc);
      if (sample > 0.0_f) {
        if (min_corner[0] > max_corner[0]) {
          min_corner = coord;
          max_corner = coord;
        } else {
          min_corner = min(min_corner, coord);
          max_corner = max(max_corner, coord);
        }
      }
    }
    min_corner -= Vector(dx);
    max_corner += Vector(dx);
    if (min_corner[0] > max_corner[0] + 0.5_f)
      TC_ERROR("density_texture is empty");
    real v = std::pow(dx, dim) / (real)ppc;
    if (dim == 2) {
      min_distance = std::sqrt(v * ((real)2 / 3));
    } else if (dim == 3) {
      min_distance = std::pow(v * ((real)13 / 18), (real)1 / 3);
    } else {
      TC_ERROR("PoissonDiskSampler only supports 2D and 3D");
    }
    if (specific_min_distance > 0)
      min_distance = specific_min_distance;
  }

  Vectori get_index(const Vector &pos) const {
    Vectori coord;
    for (size_t d = 0; d < dim; ++d)
      coord[d] = int(std::floor((pos[d] - min_corner[d]) / h));
    return coord;
  }

  Vector get_random_point_nearby(const Vector &center) const {
    while (true) {
      Vector pos;
      for (int i = 0; i < dim; ++i)
        pos[i] = rand() * 2.0_f - 1.0_f;
      real pos_2 = dot(pos, pos);
      if (0.25_f <= pos_2 && pos_2 <= 1.0_f) {
        return pos * min_distance * 2.0_f + center;
      }
    }
    return Vector(0.0_f);
  }

  bool checkDistance(const Vector &point,
                     const ArrayND<dim, int> &background_grid,
                     const std::vector<Vector> &samples) const {
    Vectori index = get_index(point);
    for (int d = 0; d < dim; ++d) {
      if (index[d] < 0 || index[d] >= background_grid.get_res()[d])
        return false;
    }
    if (background_grid[index] != -1)
      return false;
    real min_distance_2 = min_distance * min_distance;

    Vectori local_min_index = index - Vectori(2);
    Vectori local_max_index = index + Vectori(3);

    RegionND<dim> local_region(local_min_index, local_max_index);
    for (auto &ind_ : local_region) {
      Index ind = ind_;
      Vectori delta(0);
      Vector offset(0);
      if (!periodic) {
        bool outside_grid = false;
        for (int d = 0; d < dim; ++d) {
          int range = background_grid.get_res()[d];
          if (ind[d] < 0 || ind[d] >= range)
            outside_grid = true;
        }
        if (outside_grid)
          continue;
      } else {
        for (int d = 0; d < dim; ++d) {
          if (ind[d] < 0) {
            delta[d] += background_grid.get_res()[d];
            offset[d] += max_corner[d] - min_corner[d];
          } else if (ind[d] >= background_grid.get_res()[d]) {
            delta[d] -= background_grid.get_res()[d];
            offset[d] -= max_corner[d] - min_corner[d];
          }
        }
        ind = ind + delta;
      }
      if (background_grid[ind] == -1)
        continue;
      Vector x = point + offset - samples[background_grid[ind]];
      if (dot(x, x) < min_distance_2) {
        return false;
      }
    }
    return true;
  }

 public:
  PoissonDiskSampler() {
    std::string full_fn =
        absolute_path(fmt::format("$mpm/periodic_pd_{}d.dat", dim));
    std::ifstream is(full_fn, std::ifstream::in | std::ifstream::binary);
    TC_ASSERT(is.is_open());
    is.read((char *)(&points_size), sizeof(size_t));
    points_list.resize(points_size * dim);
    is.read((char *)(&points_list[0]), sizeof(float) * points_size * dim);
    is.close();
  }

  ~PoissonDiskSampler() {
  }

  void sample_from_periodic_data(std::shared_ptr<Texture> density_texture,
                                 const RegionND<dim> &region,
                                 real dx,
                                 std::vector<Vector> &samples,
                                 real specific_min_distance = -1.0_f) {
    // TC_ASSERT_INFO(dim == 3, "sample_from_periodic_data only supports 3d");

    get_ready(density_texture, region, dx, specific_min_distance);

    Vector region_size = periodic_bound * min_distance;
    Vector box_size = max_corner - min_corner;

    Vectori region_id_min(0);
    Vectori region_id_max(0);
    for (int d = 0; d < dim; ++d)
      region_id_max[d] = std::ceil(box_size[d] / region_size[d]);
    Region region_id(region_id_min, region_id_max);

    {
      // Time::Timer timer("Poisson Disk Sample Filtering");
      for (std::size_t i = 0; i < points_size; i++) {
        Vector new_point(0.0_f);
        for (int d = 0; d < dim; ++d)
          new_point[d] = points_list[i * dim + d];
        new_point = new_point * min_distance + min_corner;
        for (auto &ind : region_id) {
          Vector coord =
              new_point + region_size * (ind.get_ipos().template cast<real>() +
                                         Vector(0.5_f));
          real sample = density_texture->sample(coord).x;
          if (sample > 0.0_f)
            samples.push_back(coord);
        }
      }
    }
  }

  void sample_packed(std::shared_ptr<Texture> density_texture,
                     std::shared_ptr<Texture> local_texture,
                     const RegionND<dim> &region,
                     real dx,
                     real radius,
                     real gap,
                     std::vector<Vector> &samples) {
    std::vector<Vector> centers;
    sample_from_periodic_data(density_texture, region, dx, centers,
                              radius * 2.0_f + gap);
    std::vector<Vector> local_samples;
    sample_from_periodic_data(local_texture, region, dx, local_samples);
    for (auto &center : centers)
      for (auto &sample : local_samples)
        samples.push_back(sample - (max_corner + min_corner) * 0.5_f + center);
  }

  void sample_from_source(std::shared_ptr<Texture> density_texture,
                          const RegionND<dim> &region,
                          real dx,
                          Vector sample_offset,
                          Vector sample_advection,
                          std::vector<Vector> &samples) {
    // TC_ASSERT_INFO(dim == 3, "sample_from_periodic_data only supports 3d");

    get_ready(density_texture, region, dx);

    Vector region_size = periodic_bound * min_distance;
    Vector box_size = max_corner - min_corner;

    Vectori region_id_min(0);
    Vectori region_id_max(0);
    for (int d = 0; d < dim; ++d)
      region_id_max[d] = std::ceil(box_size[d] / region_size[d]);
    Region region_id(region_id_min, region_id_max);

    {
      Time::Timer timer("Poisson Disk Sample Filtering");
      for (std::size_t i = 0; i < points_size; i++) {
        Vector new_point(0.0_f);
        for (int d = 0; d < dim; ++d) {
          new_point[d] =
              points_list[i * dim + d] * min_distance + sample_offset[d];
          new_point[d] -=
              floor(new_point[d] / region_size[d] + 0.5_f) * region_size[d];
        }
        for (auto &ind : region_id) {
          Vector coord = min_corner + new_point +
                         region_size * (ind.get_ipos().template cast<real>() +
                                        Vector(0.5_f));
          Vector coord_next = coord + sample_advection;
          real sample = density_texture->sample(coord).x;
          real sample_next = density_texture->sample(coord_next).x;
          if (sample > 0.0_f && sample_next == 0.0_f)
            samples.push_back(coord);
        }
      }
    }
  }

  // This function generates precomputed periodic data
  void write_periodic_data() {
    std::vector<Vector> samples;

    min_corner = -0.5_f * periodic_bound;
    max_corner = 0.5_f * periodic_bound;
    min_distance = 1.0_f;
    periodic = true;

    // Step 0.
    h = min_distance / std::sqrt(real(dim));

    // Step 1.
    Vector cell_numbers_candidate = max_corner - min_corner;
    Vectori cell_numbers;
    for (int d = 0; d < dim; ++d)
      cell_numbers[d] = std::ceil(cell_numbers_candidate[d] / h);
    ArrayND<dim, int> background_grid(cell_numbers, -1);
    std::vector<int> active_list;
    Vector coord = (min_corner + max_corner) * 0.5_f;
    samples.push_back(coord);
    active_list.push_back(0);
    background_grid[get_index(coord)] = 0;

    // Step 2.
    while (active_list.size()) {
      int rand_index = rand_int() % active_list.size();
      Vector current_point = samples[active_list[rand_index]];
      std::swap(active_list[rand_index], active_list[active_list.size() - 1]);
      bool found_at_least_one = false;
      for (int i = 0; i < max_attempts; ++i) {
        Vector new_point = get_random_point_nearby(current_point);
        if (!periodic) {
          bool outside_box = false;
          for (int d = 0; d < dim; ++d)
            if (new_point[d] < min_corner[d] || new_point[d] > max_corner[d])
              outside_box = true;
          if (outside_box)
            continue;
        } else {
          for (int d = 0; d < dim; ++d) {
            if (new_point[d] < min_corner[d])
              new_point[d] += max_corner[d] - min_corner[d];
            else if (new_point[d] > max_corner[d])
              new_point[d] -= max_corner[d] - min_corner[d];
          }
        }
        if (checkDistance(new_point, background_grid, samples)) {
          found_at_least_one = true;
          samples.push_back(new_point);
          int index = samples.size() - 1;
          active_list.push_back(index);
          background_grid[get_index(new_point)] = index;
        }
      }
      if (!found_at_least_one)
        active_list.pop_back();
    }

    std::string full_fn =
        absolute_path(fmt::format("$mpm/periodic_pd_{}d.dat", dim));
    std::ofstream os(full_fn, std::ofstream::out | std::ofstream::binary);
    size_t size = samples.size();
    os.write((char *)(&size), sizeof(size_t));
    for (size_t i = 0; i < size; ++i)
      os.write((char *)(&samples[i][0]), sizeof(float) * dim);
    os.close();
    TC_INFO("Successfully generated periodic data!");
    // no more process after generating periodic data
    TC_STOP;
  }

  // Fast Poisson Disk Sampling in Arbitrary Dimensions
  void sample(std::shared_ptr<Texture> density_texture,
              const RegionND<dim> &region,
              real dx,
              std::vector<Vector> &samples) {
    get_ready(density_texture, region, dx);

    // Step 0.
    h = min_distance / std::sqrt(real(dim));

    // Step 1.
    Vector cell_numbers_candidate = max_corner - min_corner;
    Vectori cell_numbers;
    for (int d = 0; d < dim; ++d)
      cell_numbers[d] = std::ceil(cell_numbers_candidate[d] / h);
    ArrayND<dim, int> background_grid(cell_numbers, -1);
    std::vector<int> active_list;
    for (auto &ind : region) {
      Vector coord = (ind.get_ipos().template cast<real>() + Vector(0.5f)) * dx;
      real sample = density_texture->sample(coord).x;
      if (sample > 0.0_f) {
        samples.push_back(coord);
        active_list.push_back(0);
        background_grid[get_index(coord)] = 0;
        break;
      }
    }

    // Step 2.
    while (active_list.size()) {
      int rand_index = rand_int() % active_list.size();
      Vector current_point = samples[active_list[rand_index]];
      std::swap(active_list[rand_index], active_list[active_list.size() - 1]);
      bool found_at_least_one = false;
      for (int i = 0; i < max_attempts; ++i) {
        Vector new_point = get_random_point_nearby(current_point);
        if (!periodic) {
          bool outside_box = false;
          for (int d = 0; d < dim; ++d)
            if (new_point[d] < min_corner[d] || new_point[d] > max_corner[d])
              outside_box = true;
          if (outside_box)
            continue;
        } else {
          for (int d = 0; d < dim; ++d) {
            if (new_point[d] < min_corner[d])
              new_point[d] += max_corner[d] - min_corner[d];
            else if (new_point[d] > max_corner[d])
              new_point[d] -= max_corner[d] - min_corner[d];
          }
        }
        if (density_texture->sample(new_point).x == 0)
          continue;
        if (checkDistance(new_point, background_grid, samples)) {
          found_at_least_one = true;
          samples.push_back(new_point);
          int index = samples.size() - 1;
          active_list.push_back(index);
          background_grid[get_index(new_point)] = index;
        }
      }
      if (!found_at_least_one)
        active_list.pop_back();
    }
  }
};

TC_NAMESPACE_END