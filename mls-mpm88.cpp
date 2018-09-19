// 88-Line Moving Least Squares Material Point Method (MLS-MPM)  [with comments]
// To compile:    g++ mls-mpm88.cpp -std=c++14 -g -lX11 -lpthread -O3 -o mls-mpm
#include "taichi.h"      // NOTE: Make sure to download the whole mls-mpm88.zip,
using namespace taichi;  //       which includes mls-mpm88.cpp and **taichi.h**.
const int n = 64 /*grid resolution (cells)*/, window_size = 800;  // Version 1.0
const real dt = 1e-4_f, frame_dt = 1e-3_f, dx = 1.0_f / n, inv_dx = 1.0_f / dx;
auto particle_mass = 1.0_f, vol = 1.0_f;
auto hardening = 10.0_f, E = 1e4_f, nu = 0.2_f; 
real mu_0 = E / (2 * (1 + nu)), lambda_0 = E * nu / ((1+nu) * (1 - 2 * nu));
using Vec = Vector2; using Mat = Matrix2;
bool plastic = true;                  // set to false for purely elastic objects
struct Particle { Vec x, v; Mat F, C; real Jp;
  Particle(Vec x, Vec v=Vec(0)) : x(x), v(v), F(1), C(0), Jp(1) {} };
std::vector<Particle> particles;
Vector3 grid[n + 1][n + 1];          // velocity + mass, node res = cell res + 1

void advance(real dt) {
  std::memset(grid, 0, sizeof(grid));                              // Reset grid
  for (auto &p : particles) {                                             // P2G
    Vector2i base_coord =(p.x*inv_dx-Vec(0.5_f)).cast<int>();//elment-wise floor
    Vec fx = p.x * inv_dx - base_coord.cast<real>();
    // Quadratic kernels, see http://mpm.graphics Formula (123)
    Vec w[3]{Vec(0.5) * sqr(Vec(1.5) - fx), Vec(0.75) - sqr(fx - Vec(1.0)),
             Vec(0.5) * sqr(fx - Vec(0.5))};
    auto e = std::exp(hardening * (1.0_f - p.Jp)), mu=mu_0*e, lambda=lambda_0*e;
    real J = determinant(p.F);         //                         Current volume
    Mat r, s; polar_decomp(p.F, r, s); //Polor decomp. for fixed corotated model
    auto stress =                           // Cauchy stress times dt and inv_dx
        -4*inv_dx*inv_dx*dt*vol*(2*mu*(p.F-r) * transposed(p.F)+lambda*(J-1)*J);
    auto affine = stress+particle_mass*p.C;
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { // Scatter to grid
        auto dpos = (Vec(i, j) - fx) * dx;
        Vector3 mv(p.v * particle_mass, particle_mass); //translational momentum
        grid[base_coord.x + i][base_coord.y + j] +=
                                  w[i].x*w[j].y * (mv + Vector3(affine*dpos, 0));
      }
  }
  for(int i = 0; i <= n; i++) for(int j = 0; j <= n; j++) { //For all grid nodes
      auto &g = grid[i][j];
      if (g[2] > 0) {                                // No need for epsilon here
        g /= g[2];                                   //        Normalize by mass
        g += dt * Vector3(0, -100, 0);               //                  Gravity
        real boundary=0.05,x=(real)i/n,y=real(j)/n; //boundary thick.,node coord
        if (x < boundary||x > 1-boundary||y > 1-boundary) g=Vector3(0); //Sticky
        if (y < boundary) g[1] = std::max(0.0_f, g[1]);             //"Separate"
      }
    }
  for (auto &p : particles) {                                // Grid to particle
    Vector2i base_coord =(p.x*inv_dx-Vec(0.5_f)).cast<int>();//elment-wise floor
    Vec fx = p.x * inv_dx - base_coord.cast<real>();
    Vec w[3]{Vec(0.5) * sqr(Vec(1.5) - fx), Vec(0.75) - sqr(fx - Vec(1.0)),
             Vec(0.5) * sqr(fx - Vec(0.5))};
    p.C = Mat(0); p.v = Vec(0);
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        auto dpos = (Vec(i, j) - fx),
            grid_v = Vec(grid[base_coord.x + i][base_coord.y + j]);
        auto weight = w[i].x * w[j].y;
        p.v += weight * grid_v;                                      // Velocity
        p.C += 4 * inv_dx * Mat::outer_product(weight * grid_v, dpos); // APIC C
      }
    p.x += dt * p.v;                                                // Advection
    auto F = (Mat(1) + dt * p.C) * p.F;                      // MLS-MPM F-update
    Mat svd_u, sig, svd_v; svd(F, svd_u, sig, svd_v);
    for (int i = 0; i < 2 * int(plastic); i++)                // Snow Plasticity
      sig[i][i] = clamp(sig[i][i], 1.0_f - 2.5e-2_f, 1.0_f + 7.5e-3_f);
    real oldJ = determinant(F); F = svd_u * sig * transposed(svd_v);
    real Jp_new = clamp(p.Jp * oldJ / determinant(F), 0.6_f, 20.0_f);
    p.Jp = Jp_new; p.F = F;
  }
}
void add_object(Vec center) {                                  // Seed particles
  for (int i = 0; i < 700; i++)  // Randomly sample 1000 particles in the square
    particles.push_back(Particle((Vec::rand()*2.0_f-Vec(1)) * 0.08_f + center));
}
int main() {
  GUI gui("Taichi Demo: Real-time MLS-MPM 2D ", window_size, window_size);
  add_object(Vec(0.5,0.4)); add_object(Vec(0.45,0.6));add_object(Vec(0.55,0.8));
  for (int i = 0;; i++) {                              //              Main Loop
    advance(dt);                                       //     Advance simulation
    if (i % int(frame_dt / dt) == 0) {                 //           Redraw frame
      gui.get_canvas().clear(Vector4(0.2, 0.4, 0.7, 1.0_f)); // Clear background
      for (auto p : particles)                                 // Draw particles
        gui.buffer[(p.x * (inv_dx * window_size/n)).cast<int>()] = Vector4(0.8);
      gui.update();                                              // Update image
    }//Reference: A Moving Least Squares Material Point Method with Displacement
  } //             Discontinuity and Two-Way Rigid Body Coupling (SIGGRAPH 2018)
}  //  By Yuanming Hu (who also wrote this 88-line version), Yu Fang, Ziheng Ge,
//                          Ziyin Qu, Yixin Zhu, Andre Pradhana, Chenfanfu Jiang
