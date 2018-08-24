/*******************************************************************************
    Copyright (c) The Taichi MPM Authors (2018- ). All Rights Reserved.
    The use of this software is governed by the LICENSE file.
*******************************************************************************/

#include <taichi/common/testing.h>

#include "mpm_fwd.h"
#include "kernel.h"

TC_NAMESPACE_BEGIN

template <int dim, int order>
void test_kernel() {
  using Vector = VectorND<dim, real>;
  for (int l = 0; l < 100; l++) {
    auto pos = Vector::rand() * 10.0f;
    MPMKernel<dim, order> kernel(pos, 1.0f);
    for (int j = 0; j < dim; j++) {
      CHECK(kernel.w_cache[j].sum() == Approx(1.0_f));
    }
    for (int j = 0; j < dim; j++) {
      CHECK(kernel.dw_cache[j].sum() == Approx(0.0_f).margin(1e-6_f));
    }
  }
}

TC_TEST("mpm_kernel") {
  test_kernel<2, 3>();
  test_kernel<2, 2>();
  test_kernel<3, 3>();
  test_kernel<3, 2>();
}

TC_TEST("mpm_fast_kernel32") {
  constexpr int dim = 3;
  constexpr int order = 2;
  using Vector = VectorND<dim, real>;
  for (int l = 0; l < 100; l++) {
    auto pos = Vector::rand() * 10.0f;
    MPMKernel<dim, order> kernel(pos, 1.0f);
    MPMFastKernel32 fast_kernel(pos, 1.0f);
    RegionND<3> region(Vector3i(0), Vector3i(3));
    for (auto &ind : region) {
      auto i = ind.get_ipos();
      auto slow = kernel.get_dw_w(i);
      auto fast = fast_kernel.get_dw_w(i);
      CHECK(length(slow - fast) == Approx(0).margin(1e-6_f));
    }
  }
}

TC_NAMESPACE_END
