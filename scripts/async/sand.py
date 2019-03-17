import taichi as tc
from math import *
from random import *
from async_mpm import AsyncMPM

r = 400
task_id = 'sand'
Async = True

if __name__ == '__main__':
  res = (r, r, r)

  if Async:
    mpm = AsyncMPM(
        res=res,
        particle_collision=False,
        optimized=True,
        verbose_bgeo=True,
        gravity=(0, -1, 0),
        task_id=task_id,
        cfl_dt_mul=0.1,
        num_frames=120)
  else:
    mpm = tc.dynamics.MPM(
        res=res,
        particle_collision=False,
        optimized=True,
        verbose_bgeo=True,
        gravity=(0, -1, 0),
        base_delta_t=1e-6 * 64,
        task_id=task_id,
        num_frames=120)

  levelset = mpm.create_levelset()
  levelset.add_cuboid((0, 0.2, 0.05), (0.95, 0.95, 0.95), True)
  levelset.set_friction(-3)
  mpm.set_levelset(levelset, False)

  tex = tc.Texture(
      'mesh',
      translate=(0.5, 0.5, 0.5),
      scale=(0.012, 0.012, 0.012),
      adaptive=True,
      filename='$mpm/cute_dragon.obj') * 8
  tex = tc.Texture(
      'rotate', tex=tex, rotate_axis=1, rotate_times=3).translate((0.1, -0.125,
                                                                   0))

  mpm.add_particles(
      type='sand',
      density_tex=tex.id,
      lambda_0=20000,
      mu_0=1000,
      friction_angle=10)

  tex_sphere = tc.Texture(
      'mesh',
      translate=(0, 0, 0),
      scale=(0.015, 0.015, 0.015),
      adaptive=True,
      filename='$mpm/sphere_small.obj') * 8
  seed(1)
  for i in range(6):
    x = 0.05 + random() * 0.05
    y = 0.25 + random() * 0.2
    z = 0.45 + random() * 0.1
    if i == 0:
      continue
    tex = tex_sphere.translate((x, y, z))
    mpm.add_particles(
        type='elastic', E=2e5, density_tex=tex.id, initial_velocity=(5, 0, 0))

  mpm.simulate()
