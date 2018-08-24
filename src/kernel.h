/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#pragma once

#include <taichi/math/math.h>

TC_NAMESPACE_BEGIN

//#define MPM_TRANSFER_OPT

template <int dim, int order_>
struct MPMKernelBase {
  constexpr static int D = dim;
  constexpr static int order = order_;
  constexpr static int kernel_size = order + 1;

  using Vector = VectorND<dim, real>;
  using VectorP = VectorND<dim + 1, real>;
  using VectorI = VectorND<dim, int>;

  VectorP w_stages[D][kernel_size];
  Vector4 w_cache[D];
  Vector4 dw_cache[D];
  VectorP kernels;
  real inv_delta_x;

  TC_FORCE_INLINE void shuffle() {
    for (int k = 0; k < kernel_size; k++) {
      for (int j = 0; j < D; j++) {
        w_stages[j][k] = VectorP([&](int i) -> real {
          if (j == i) {
            return dw_cache[j][k] * inv_delta_x;
          } else {
            return w_cache[j][k];
          }
        });
      }
    }
  }

  TC_FORCE_INLINE VectorP get_dw_w(const VectorI &k) const {
    VectorP ret = w_stages[0][k[0]];
    for (int i = 1; i < dim; i++) {
      ret *= w_stages[i][k[i]];
    }
    return ret;
  }

  TC_FORCE_INLINE Vector get_dw(const VectorI &k) const {
    VectorP ret = w_stages[0][k[0]];
    for (int i = 1; i < dim; i++) {
      ret *= w_stages[i][k[i]];
    }
    return Vector(ret);
  }

  TC_FORCE_INLINE real get_w(const VectorI &k) const {
    VectorP ret = w_stages[0][k[0]];
    for (int i = 1; i < dim; i++) {
      ret *= w_stages[i][k[i]];
    }
    return ret[D];
  }

  static TC_FORCE_INLINE constexpr real inv_D() {
    return 6.0f - real(order);
  }
};

template <int dim, int order>
struct MPMKernel;

// Linear kernel
template <int dim>
struct MPMKernel<dim, 1> : public MPMKernelBase<dim, 1> {
  using Base = MPMKernelBase<dim, 1>;
  using Vector = typename Base::Vector;

  TC_FORCE_INLINE MPMKernel(const Vector &pos, real inv_delta_x) {
    calculate_kernel(pos);
    this->inv_delta_x = inv_delta_x;
    this->shuffle();
  }

  TC_FORCE_INLINE static constexpr int get_stencil_start(real x) {
    return int(x);
  }

  TC_FORCE_INLINE void calculate_kernel(const Vector &pos) {
    Vector p_fract = fract(pos);
    for (int k = 0; k < dim; k++) {
      const Vector4 t = Vector4(p_fract[k]) - Vector4(0, 1, 0, 0);
      this->w_cache[k] = Vector4(-1.f, 1.f, 0, 0) * t + Vector4(1.f, 1.f, 0, 0);
      this->dw_cache[k] = Vector4(1.f, -1.f, 0, 0);
    }
  }
};

// Quadratic kernel
template <int dim>
struct MPMKernel<dim, 2> : public MPMKernelBase<dim, 2> {
  using Base = MPMKernelBase<dim, 2>;
  using Vector = typename Base::Vector;

  TC_FORCE_INLINE MPMKernel(const Vector &pos,
                            real inv_delta_x,
                            bool do_calculate_kernel = true,
                            bool do_shuffle = true) {
    if (do_calculate_kernel)
      calculate_kernel(pos);
    this->inv_delta_x = inv_delta_x;
    if (do_shuffle)
      this->shuffle();
  }

  TC_FORCE_INLINE static int get_stencil_start(real x) {
    return int(x - 0.5f);
  }

  TC_FORCE_INLINE void calculate_kernel(const Vector &pos) {
    Vector p_fract = fract(pos - Vector(0.5f));
    for (int k = 0; k < dim; k++) {
      const Vector4 t = Vector4(p_fract[k]) - Vector4(-0.5f, 0.5f, 1.5f, 0.0f);
      auto tt = t * t;
      this->w_cache[k] = Vector4(0.5f, -1.0f, 0.5f, 0.0f) * tt +
                         Vector4(-1.5f, 0.0f, 1.5f, 0.0f) * t +
                         Vector4(1.125f, 0.75f, 1.125f, 0.0f);
      this->dw_cache[k] =
          Vector4(1.0f, -2.0f, 1.0f, 0.0f) * t + Vector4(-1.5f, 0, 1.5f, 0.0f);
    }
  }
};

// Cubic kernel
template <int dim>
struct MPMKernel<dim, 3> : public MPMKernelBase<dim, 3> {
  using Base = MPMKernelBase<dim, 3>;
  using Vector = typename Base::Vector;

  TC_FORCE_INLINE MPMKernel(const Vector &pos, real inv_delta_x) {
    calculate_kernel(pos);
    this->inv_delta_x = inv_delta_x;
    this->shuffle();
  }

  TC_FORCE_INLINE static constexpr int get_stencil_start(real x) {
    return int(x) - 1;
  }

  TC_FORCE_INLINE void calculate_kernel(const Vector &pos) {
    Vector p_fract = fract(pos);
    for (int k = 0; k < dim; k++) {
      const Vector4 t = Vector4(p_fract[k]) - Vector4(-1, 0, 1, 2);
      auto tt = t * t;
      auto ttt = tt * t;
      this->w_cache[k] = Vector4(-1 / 6.0f, 0.5f, -0.5f, 1 / 6.0f) * ttt +
                         Vector4(1, -1, -1, 1) * tt + Vector4(-2, 0, 0, 2) * t +
                         Vector4(4 / 3.0f, 2 / 3.0f, 2 / 3.0f, 4 / 3.0f);
      this->dw_cache[k] = (Vector4(-0.5f, 1.5f, -1.5f, 0.5f) * tt +
                           Vector4(2, -2, -2, 2) * t + Vector4(-2, 0, 0, 2));
    }
  }
};

struct MPMFastKernel32 : public MPMKernelBase<3, 2> {
  static constexpr int dim = 3;
  using Base = MPMKernelBase<dim, 2>;
  using Vector = typename Base::Vector;
  TC_ALIGNED(64) VectorP kernels[3][3][3];

  TC_FORCE_INLINE MPMFastKernel32(const Vector &pos, real inv_delta_x) {
    calculate_kernel(pos);
    this->inv_delta_x = inv_delta_x;
    this->shuffle();
    VectorP ret(1);
    for (int i = 0; i < 3; i++) {
      VectorP ret1 = ret * w_stages[0][i];
      for (int j = 0; j < 3; j++) {
        VectorP ret2 = ret1 * w_stages[1][j];
        for (int k = 0; k < 3; k++) {
          kernels[i][j][k] = ret2 * w_stages[2][k];
        }
      }
    }
  }

  TC_FORCE_INLINE VectorP get_dw_w(const Vector3i &i) {
    return kernels[i.x][i.y][i.z];
  }

  TC_FORCE_INLINE static int get_stencil_start(real x) {
    return int(x - 0.5f);
  }

  TC_FORCE_INLINE void calculate_kernel(const Vector &pos) {
    Vector p_fract = fract(pos - Vector(0.5f));
    for (int k = 0; k < dim; k++) {
      const Vector4 t = Vector4(p_fract[k]) - Vector4(-0.5f, 0.5f, 1.5f, 0.0f);
      auto tt = t * t;
      this->w_cache[k] = Vector4(0.5f, -1.0f, 0.5f, 0.0f) * tt +
                         Vector4(-1.5f, 0.0f, 1.5f, 0.0f) * t +
                         Vector4(1.125f, 0.75f, 1.125f, 0.0f);
      this->dw_cache[k] =
          Vector4(1.0f, -2.0f, 1.0f, 0.0f) * t + Vector4(-1.5f, 0, 1.5f, 0.0f);
    }
  }
};

TC_NAMESPACE_END
