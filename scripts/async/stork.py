import taichi as tc
import os
import math
import random
from async_mpm import AsyncMPM

r = 250
task_id = 'stork'
Async = True

if __name__ == '__main__':
  res = (r, r, r)

  if Async:
    mpm = AsyncMPM(
        res=res,
        particle_collision=False,
        optimized=True,
        verbose_bgeo=True,
        gravity=(0, 0, 0),
        max_units=128,
        task_id=task_id,
        strength_dt_mul=0.8,
        print_energy=True,
        num_frames=480)
  else:
    mpm = tc.dynamics.MPM(
        res=res,
        particle_collision=False,
        optimized=True,
        verbose_bgeo=True,
        gravity=(0, 0, 0),
        base_delta_t=1e-6 * 8,
        task_id=task_id,
        print_energy=True,
        num_frames=480)

  levelset = mpm.create_levelset()
  # levelset.add_plane(tc.Vector(0, 1, 0), -0.3)
  levelset.add_cuboid((0.5, 0.1, 0.3), (0.7, 0.35, 0.7), False)
  levelset.set_friction(-1)
  mpm.set_levelset(levelset, False)

  tex = tc.Texture(
      'mesh',
      translate=(0.5, 0.5, 0.5),
      scale=(0.005, 0.005, 0.005),
      adaptive=True,
      filename='$mpm/stork.obj') * 12
  tex = tc.Texture(
      'rotate', tex=tex, rotate_axis=1, rotate_times=1).translate((0.1, 0, 0))
  tex_up = tc.Texture(
      'bound', tex=tex, axis=1, bounds=(0.4, 0.9), outside_val=(0, 0, 0))
  tex_down = tc.Texture(
      'bound', tex=tex, axis=1, bounds=(0.1, 0.4), outside_val=(0, 0, 0))

  tex_sphere = tc.Texture('sphere', center=(0.6, 0.45, 0.5), radius=0.06)
  tex_joint = tex_up * tex_sphere
  tex_up = tex_up * (1 - tex_sphere)

  tex_sphere = tc.Texture('sphere', center=(0.5, 0.64, 0.5), radius=0.038)
  tex_neck = tex_up * tex_sphere
  tex_up = tex_up * (1 - tex_sphere)

  speed = 4
  mpm.add_particles(
      type='elastic',
      density_tex=tex_neck.id,
      E=4e3,
      density=10,
      stork_nod=speed,)
  mpm.add_particles(
      type='elastic',
      density_tex=tex_up.id,
      E=4e3,
      density=10,
      stork_nod=speed,)
  mpm.add_particles(
      type='elastic',
      density_tex=tex_joint.id,
      E=2e4,
      density=10,
      stork_nod=speed,)
  mpm.add_particles(
      type='elastic',
      density_tex=tex_down.id,
      initial_velocity=(0, 0, 0),
      E=4e5,
      density=10,
      stork_nod=speed,)

  mpm.simulate(clear_output_directory=True)
