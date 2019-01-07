//88-Line 2D Moving Least Squares Material Point Method (MLS-MPM)[with comments]
#define TC_IMAGE_IO   // Uncomment this line for image exporting functionality
#include "taichi.h"    // Note: You DO NOT have to install taichi or taichi_mpm.
using namespace taichi;// You only need [taichi.h] - see below for instructions.
const int n = 80 /*grid resolution (cells)*/, window_size = 800;
const real dt = 1e-4_f, frame_dt = 1e-3_f, dx = 1.0_f / n, inv_dx = 1.0_f / dx;
auto particle_mass = 1.0_f, vol = 1.0_f;
auto hardening = 10.0_f, E = 1e4_f, nu = 0.2_f;
real mu_0 = E / (2 * (1 + nu)), lambda_0 = E * nu / ((1+nu) * (1 - 2 * nu));
using Vec = Vector2; using Mat = Matrix2; bool plastic = true;
struct Particle { Vec x, v; Mat F, C; real Jp; int c/*color*/;
  Particle(Vec x, int c, Vec v=Vec(0)) : x(x), v(v), F(1), C(0), Jp(1), c(c){}};
std::vector<Particle> particles;
Vector3 grid[n + 1][n + 1];          // velocity + mass, node_res = cell_res + 1

void advance(real dt) {
  std::memset(grid, 0, sizeof(grid));                              // Reset grid
  for (auto &p : particles) {                                             // P2G
    Vector2i base_coord=(p.x*inv_dx-Vec(0.5_f)).cast<int>();//element-wise floor
    Vec fx = p.x * inv_dx - base_coord.cast<real>();
    // Quadratic kernels  [http://mpm.graphics   Eqn. 123, with x=fx, fx-1,fx-2]
    Vec w[3]{Vec(0.5) * sqr(Vec(1.5) - fx), Vec(0.75) - sqr(fx - Vec(1.0)),
             Vec(0.5) * sqr(fx - Vec(0.5))};
    auto e = std::exp(hardening * (1.0_f - p.Jp)), mu=mu_0*e, lambda=lambda_0*e;
    real J = determinant(p.F);         //                         Current volume
    Mat r, s; polar_decomp(p.F, r, s); //Polar decomp. for fixed corotated model
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
        g += dt * Vector3(0, -200, 0);               //                  Gravity
        real boundary=0.05,x=(real)i/n,y=real(j)/n; //boundary thick.,node coord
        if (x < boundary||x > 1-boundary||y > 1-boundary) g=Vector3(0); //Sticky
        if (y < boundary) g[1] = std::max(0.0_f, g[1]);             //"Separate"
      }
    }
  for (auto &p : particles) {                                // Grid to particle
    Vector2i base_coord=(p.x*inv_dx-Vec(0.5_f)).cast<int>();//element-wise floor
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
void add_object(Vec center, int c) {   // Seed particles with position and color
  for (int i = 0; i < 1000; i++)  // Randomly sample 1000 particles in the square
    particles.push_back(Particle((Vec::rand()*2.0_f-Vec(1))*0.08_f + center, c));
}
int main() {
  GUI gui("Real-time 2D MLS-MPM", window_size, window_size);
  add_object(Vec(0.55,0.45), 0xED553B); add_object(Vec(0.45,0.65), 0xF2B134);
  add_object(Vec(0.55,0.85), 0x068587); auto &canvas = gui.get_canvas();int f=0;
  for (int i = 0;; i++) {                              //              Main Loop
    advance(dt);                                       //     Advance simulation
    if (i % int(frame_dt / dt) == 0) {                 //        Visualize frame
      canvas.clear(0x112F41);                          //       Clear background
      canvas.rect(Vec(0.04), Vec(0.96)).radius(2).color(0x4FB99F).close();// Box
      for(auto p:particles)canvas.circle(p.x).radius(2).color(p.c);//Particles
      gui.update();                                              // Update image
      // canvas.img.write_as_image(fmt::format("tmp/{:05d}.png", f++));
    }
  }
} //----------------------------------------------------------------------------

/* -----------------------------------------------------------------------------
** Reference: A Moving Least Squares Material Point Method with Displacement
              Discontinuity and Two-Way Rigid Body Coupling (SIGGRAPH 2018)

  By Yuanming Hu (who also wrote this 88-line version), Yu Fang, Ziheng Ge,
           Ziyin Qu, Yixin Zhu, Andre Pradhana, Chenfanfu Jiang


** Build Instructions:

Step 1: Download and unzip mls-mpm88.zip (Link: http://bit.ly/mls-mpm88)
        Now you should have "mls-mpm88.cpp" and "taichi.h".

Step 2: Compile and run

* Linux:
    g++ mls-mpm88.cpp -std=c++14 -g -lX11 -lpthread -O3 -o mls-mpm
    ./mls-mpm


* Windows (MinGW):
    g++ mls-mpm88.cpp -std=c++14 -lgdi32 -lpthread -O3 -o mls-mpm
    .\mls-mpm.exe


* Windows (Visual Studio 2017+):
  - Create an "Empty Project"
  - Use taichi.h as the only header, and mls-mpm88.cpp as the only source
  - Change configuration to "Release" and "x64"
  - Press F5 to compile and run


* OS X:
    g++ mls-mpm88.cpp -std=c++14 -framework Cocoa -lpthread -O3 -o mls-mpm
    ./mls-mpm


** FAQ:
Q1: What does "1e-4_f" mean?
A1: The same as 1e-4f.

Q2: What is "real"?
A2: real = float in this file.

Q3: What are the hex numbers like 0xED553B?
A3: They are RGB color values.
    The color scheme is borrowed from
    https://color.adobe.com/Copy-of-Copy-of-Core-color-theme-11449181/

Q4: How can I get higher-quality?
A4: Change n to 320; Change dt to 1e-5; Change E to 2e4;
    Change particle per cube from 500 to 8000 (Ln 72).
    After the change, the whole animation takes ~3 minutes on my computer.

Q5: How to record the animation?
A5: Uncomment Ln 2 and 85 and create a folder named "tmp".
    The frames will be saved to "tmp/XXXXX.png".

    To get a video, you can use ffmpeg. If you already have taichi installed,
    you can simply go to the "tmp" folder and execute

      ti video 60

    where 60 stands for 60 FPS. A file named "video.mp4" is what you want.


For more questions, please email yuanming _at_ mit.edu
                    or visit https://github.com/yuanming-hu/taichi_mpm/issues.

                                                       Last Update: Nov 16, 2018
                                                       Version 1.4

----------------------------------------------------------------------------- */
