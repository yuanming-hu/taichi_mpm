#  High-Performance MLS-MPM Solver with Cutting and Coupling (CPIC)
<img src="https://github.com/yuanming-hu/public_files/raw/master/graphics/mls-mpm-cpic/water_wheel.gif"> <img src="https://github.com/yuanming-hu/public_files/raw/master/graphics/mls-mpm-cpic/sand_paddles.gif">

For the reproducibility of, and further development on **A Moving Least Squares Material Point Method with Displacement Discontinuity and Two-Way Rigid Body Coupling**, ACM Transactions on Graphics (SIGGRAPH 2018).

By [Yuanming Hu (MIT CSAIL)](http://taichi.graphics/me/), [Yu Fang (Tsinghua University)](http://squarefk.com/), Ziheng Ge (University of Science and Technology of China), Ziyin Qu (University of Pennsylvania), [Yixin Zhu (UCLA)](https://www.yzhu.io/), [Andre Pradhana (University of Pennsylvania)](https://www.seas.upenn.edu/~apradh/menu/about.html), [Chenfanfu Jiang (University of Pennsylvania)](https://www.seas.upenn.edu/~cffjiang/).

**Note: this software is released under the GPLv3.0 license. For commercial use, please email Yuanming Hu for negotiation.**

#### [[Introduction & Demo Video](https://www.youtube.com/watch?v=8iyvhGF9f7o)] [[Paper](http://taichi.graphics/wp-content/uploads/2018/05/mls-mpm-cpic.pdf)] [[Supplemental Document](http://taichi.graphics/wp-content/uploads/2018/04/mls-mpm-cpic-supp.pdf)] [[88-Line MLS-MPM](https://github.com/yuanming-hu/taichi_mpm/blob/master/mls-mpm88.cpp)]
#### [[SIGGRAPH 2018 Fast Forward](https://youtu.be/9RlNEgwTtPI)] [[PDF Slides](https://github.com/yuanming-hu/taichi_mpm/releases/download/SIGGRAPH2018/mls-mpm-cpic-slides.pdf)] [[PDF Slides with Notes](https://github.com/yuanming-hu/taichi_mpm/releases/download/SIGGRAPH2018/mls-mpm-cpic-slides-with-notes.pdf)]


<img src="https://github.com/yuanming-hu/public_files/raw/master/graphics/mls-mpm-cpic/armodillo.gif" style=""> <img src="https://github.com/yuanming-hu/public_files/raw/master/graphics/mls-mpm-cpic/debris_flow.gif">
<img src="https://github.com/yuanming-hu/public_files/raw/master/graphics/mls-mpm-cpic/sand-sweep.gif"> <img src="https://github.com/yuanming-hu/public_files/raw/master/graphics/mls-mpm-cpic/sand_stir.gif">
<img src="https://github.com/yuanming-hu/public_files/raw/master/graphics/mls-mpm-cpic/bunny.gif"> <img src="https://github.com/yuanming-hu/public_files/raw/master/graphics/mls-mpm-cpic/robot_forward.gif">
<img src="https://github.com/yuanming-hu/public_files/raw/master/graphics/mls-mpm-cpic/banana.gif"> <img src="https://github.com/yuanming-hu/public_files/raw/master/graphics/mls-mpm-cpic/cheese.gif">

## Installation
### Linux
Install [`taichi`](https://github.com/yuanming-hu/taichi) [[Instructions](https://taichi.readthedocs.io/en/latest/installation.html)].
Then, in command line
```
ti install mpm
```
and it will install this taichi package automatically.
### Windows and OSX
Support coming in Sepetember.

## Run demos
Every script under the folder `scripts/mls-cpic` is executable with `python3`.
# Visualize the results
 - Outputs are in `taichi/outputs/mpm/`;
 - Install [Houdini Apprentice](https://www.sidefx.com/products/houdini-apprentice/) (which is free);
 - Create a `File` node in Houdini to visualize the `bgeo` (particles), `obj` (3D meshes), `poly` (2D polygons) files.

# Python 3 API

## MPM.initialize
(You only need to specify `res` in most cases. The default parameters generally work well.) 
All parameters:
 - res: (`Vector<dim, int>`) grid resolution. The length of this vector also specifies the dimensionality of the simulation.
 - base_delta_t : (`real`, default: `1e-4`) delta t
 - delta_x: (`real`, default: `1.0 / res[0]`)
 - particle_collision (`bool`, default: `False`): push particles inside level sets out (turn off when you are using sticky level sets)
 - pushing_force: (`real`, default: `20000.0`) If things do not separate, use this. Typical value: 20000.0.
 - gravity (`Vector<dim>`, default: `(0, -10, 0)` for 3D, `(0, -10)` for 2D)
 - frame_dt: (`real`, default: `0.01`) You can set to `1 / 24` for real frame rate.
 - num_threads: (`int`, default: `-1`) Number of threads to use. `-1` means maximum threads.
 - num_frames: (`int`, default: `1000`) Number of frames to simulate.
 - penalty: (`real`, default: `0`) Penetration penalty. Typical values are `1e3` ~ `1e4`.
 - optimized: (`bool`, default: `True`) Turn on optimization or not. Turning it off if you need to benchmark the less optimized transfers.
 - task_id: (`string`, default: `taichi` will use the current file name)
 - rigid_body_levelset_collision: (`bool`, default: `False`) Collide rigid body with level set? (Useful for wine & glass.)
 - rpic_damping: (`real`, default: `0`) RPIC damping value should be between 0 and 1 (inclusive).
 - apic_damping: (`real`, default: `0`) APIC damping value should be between 0 and 1 (inclusive).
 - warn_particle_deletion: (`bool`, default: `False`) Log warning when particles get deleted
 - verbose_bgeo: (`bool`, default: `false`) If `true`, output particle attributes other than `position`.
 - reorder_interval: (`int`, default: `1000`) If bigger than error, sort particle storage in memory every `reorder_interval` substeps.
 - clean_boundary: (`int`, default: `1000`) If bigger than error, sort particle storage in memory every `reorder_interval` substeps.
 - ...
 
## MPM.add_particles
 - type: `rigid`, `snow`, `jelly`, `sand`. For non-rigid type, see [Particle Attributes](#attributes)
 - color: (`Vector<3, real>`)
 - pd : (`bool`, default: `True`) Is poisson disk sampling or not? Doesn't support type: `rigid`, `sdf`
 - pd_periodic : (`bool`, default: `True`) Is poisson disk periodic or not? Doesn't support 2D
 - pd_source : (`bool`, default: `False`) Is poisson disk sampling from source or not (need to define`frame_update`)? Doesn't support 2D
 - For type `rigid`:
 - rotation_axis : (`Vector<3, real>`, default: `(0, 0, 0)`) Let the object rotate along with only this axis. Useful for fans or wheels.
 - codimensional : (`bool`, must be explicitly specified) Is thin shell or not?
 - restitution: (`real`, default: `0.0`) Coefficient of restitution
 - friction: (`real`, default: `0.0`) Coefficient of friction
 - density: (`real`, default: `40` for thin shell, `400` for non-thin shell) Volume/area density.
 - scale: (`Vector<3, real>`, default: `(1, 1, 1)`) rescale the object, bigger or smaller
 - initial_position: (`Vector<dim, real>`, must be explicitly specified)
 - initial_velocity: (`Vector<dim, real>`, default: `(0, 0, 0)`)
 - scripted_position: (`function(real) => Vector<3, real>`)
 - initial_rotation: (`Vector<1 (2D) or 3 (3D), real>`, default: `(0, 0, 0)`) Euler angles
 - initial_angular_velocity: (`Vector<1 (2D) or 3 (3D), real>`, default: `(0, 0, 0)`)
 - scripted_rotation: (`function(real) => Vector<1 (2D) or 3 (3D), real>`) Takes time, returns Euler angles
 - (Translational/Rotational) static objects are also considered as scripted, but with a fixed scripting function i.e. `tc.constant_function13(tc.Vector(0, 0, 0))`
 - linear_damping: (`real`, default: `0`) damping of linear velocity. Typical value: `1`
 - angular_damping: (`real`, default: `0`) damping of angular velocity. Typical value: `1`
 - ...
 
# <a name="attributes"></a>Particle Attributes
 * `jelly`
    - `E`:  (`real`, default: `1e5`) Young's modulus
    - `nu`:  (`real`, default: `0.3`) Poisson's ratio
 * `snow`
    - `hardeing` (`real`, default: `10`) Hardening coefficient
    - `mu_0` (`real`, default: `58333.3`) Lame parameter
    - `lambda_0` (`real`, default: `38888.9`) Lame parameter
    - `theta_c` (`real`, default: `2.5e-2`) Critical compression
    - `theta_s` (`real`, default: `7.5e-3`) Critical stretch
 * `sand`
    - `mu_0` (`real`, default: `136038`) Lame parameter
    - `lambda_0` (`real`, default: `204057`) Lame parameter
    - `friction_angle` (`real`, default: `30`)
    - `cohesion` (`real`, default: `0`)
    - `beta` (`real`, default: `1`)
 * `water`
    - `k`:  (`real`, default: `1e5`) Bulk modulus
    - `gamma`:  (`real`, default: `7`)
 * `von_mises`
    - `youngs_modulus`:  (`real`, default: `5e3`) Young's modulus (for elasticity)
    - `poisson_ratio`:  (`real`, default: `0.4`) Poisson's ratio (for elasticity, usually no need to change)
    - `yield_stress`: (`real`, default:`1.0`) Radius of yield surface (for plasticity)
 * ...

## Script Examples
 - Scripted motion: `scripted_motion_3d.py`.
 - Rigid-ground collison: `rigid_ground_collision.py`.
 - When you're making an rotating wheel example, e.g. `thin_wheels_fans.py` and the wheel is not turning in the right direction, you can try `reverse_vertices=True`.
 - ...
 
# Notes
 - Matrices in taichi are column major. E.g. A[3][1] is the element at row 2 and column 4.
 - All indices, unless explicitly specified, are 0-based.
 - Use `real`, in most cases, instead of `float` or `double`.
 - Float point constants should be suffixed with `_f`, so that it will have type `real`, instead of `float` or `double`. Example: `1.5_f` (`float` or `double` depending on build precision) instead of `1.5` (always `double`) or `1.5f` (always `float`)
 - Always pull `taichi` (the main lib, *master* branch) after updating `taichi_mpm`.
 - When a particles moves too close to the boundary (4-8 dx) it will be deleted.
 - Whenever you can any compile/linking problem:
   - Make sure `taichi` is up-to-date
   - Invoke `CMake` so that all no source files will be detected
   - Rebuild
 - ...
  
# Friction Coefficient
 - Separate: positive values, `0.4` means coeff of friction `0.4`
 - Sticky: -1
 - Slip: -2
 - Slip with friction: `-2.4` means coeff of friction `0.4` with slip
 
# Articulation

Syntax:

```$python
    object1 = mpm.add_particles(...)
    object2 = mpm.add_particles(...)

    mpm.add_articulation(type='motor', obj0=object1, obj1=object2, axis=(0, 0, 1), power=0.05)
```

 * Rotation: enforce two objects to have the same rotation.
   - `type`:  `rotation`
   - `obj0`, `obj1`: two objects
   - Use case: blabe and wheel in `water wheel` examples 
   
 * Distance: enforce two points on two different object to have constant distance
   - `type`:  `distance`
   - `obj0`, `obj1`: two objects
   - `offset0`, `offset1`: (`Vector<dim, real>`, default: `(0, 0, 0)`) offset of two points to the center of mass to each object, in world space
   - `distance` (`real`, default: initial distance between two poitns) target distance
   - `penalty` (`real`, default: `1e5`) corrective penalty
   - Use case: hammer in `crashing_castle` examples 
   
 * Motor: enforce object to rotate along an axis on another object, and apply torque
   - `type`:  `motor`
   - `obj0`: the `wheel` object
   - `obj1`: the `body` object
   - `axis`: (`Vector<dim, real>`) the rotation axis in world space
   - `power` (`real`, default: `0`) torque applied per second
   - Use case: wheels for cars, and legs for the robot
   - Example: `motor.py`
 * Stepper: enforce object to rotate along an axis on another object at a fixed angular velocity
   - `type`:  `motor`
   - `obj0`: the `wheel` object
   - `obj1`: the `body` object
   - `axis`: (`Vector<dim, real>`) the rotation axis in world space
   - `angular_velocity` (`real`) 
   - Use case: Fixed-rotation-speed wheels for cars, and legs for the robot

# Source Sampling
 - If you want to source particles continuously from a object, please set `pd_source = True` in `add_particles`
 - `initial_velocity` should be a non-zero vector
 - Remember to also set `delta_t=frame_dt` in `add_particles`, which enables the frequency of sampling to be consistent with its initial velocity
 - There might be some artifact due to the effect of gravity. You can reduce that artifact by  increasing `update_frequency`.
 - Example: `source_sampling.py`, `source_sampling_2d.py`

## 88-Line Version [[Download](https://github.com/yuanming-hu/taichi_mpm/releases/download/SIGGRAPH2018/mls-mpm88.zip)]
``` C++
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
```


## Mathematical Comparisons with Traditional MPM
<img src="/data/images/comparisons.jpg" with="1000">

# Performance

# Bibtex
Please cite our [paper](http://taichi.graphics/wp-content/uploads/2018/05/mls-mpm-cpic.pdf) if you use this code for your research: 
```
@article{hu2018mlsmpmcpic,
  title={A Moving Least Squares Material Point Method with Displacement Discontinuity and Two-Way Rigid Body Coupling},
  author={Hu, Yuanming and Fang, Yu and Ge, Ziheng and Qu, Ziyin and Zhu, Yixin and Pradhana, Andre and Jiang, Chenfanfu},
  journal={ACM Transactions on Graphics (TOG)},
  volume={37},
  number={4},
  pages={150},
  year={2018},
  publisher={ACM}
}
```
