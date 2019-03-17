import taichi as tc
import os
import math
import random
from async_mpm import AsyncMPM

r = 250
task_id = 'balls'
Async = True

n = 6
box = (0.9 - 0.1) / n
first = 0.1 + 0.5 * box
radius = box * 0.9 * 0.5

if __name__ == '__main__':
  res = (r, r, r)

  if Async:
    mpm = AsyncMPM(
        res=res,
        particle_collision=False,
        optimized=True,
        verbose_bgeo=True,
        gravity=(0, 0, 0),
        task_id=task_id,
        num_frames=120)
  else:
    mpm = tc.dynamics.MPM(
        res=res,
        particle_collision=False,
        optimized=True,
        gravity=(0, 0, 0),
        base_delta_t=1e-6 * 64,
        task_id=task_id,
        num_frames=120)

  def levelset_generator(t):
    during = 1
    if t > during:
      t = during
    d = 0.12 * t / during
    levelset = mpm.create_levelset()
    levelset.add_cuboid((0.1 + d, 0.5, 0.1 + d), (0.9 - d, 0.5 + box, 0.9 - d),
                        True)
    levelset.set_friction(-2)
    return levelset

  mpm.set_levelset(levelset_generator, True)

  tex_sphere = tc.Texture(
      'mesh',
      translate=((0, 0, 0)),
      scale=(radius, radius, radius),
      adaptive=True,
      filename='$mpm/sphere_small.obj') * 8
  for i in range(n):
    for j in range(n):
      center = (first + i * box, 0.5 + box * 0.5, first + j * box)
      tex = tex_sphere.translate(center)
      if i == 1 and j == 4:
        E = 2e5
      elif i == 2 and j == 1:
        E = 8e3
      elif i == 3 and j == 1:
        E = 8e3
      elif i == 4 and j == 2:
        E = 4e4
      else:
        E = 1e3
      mpm.add_particles(
          type='elastic', density_tex=tex.id, E=E, initial_velocity=(0, 0, 0))

  mpm.simulate()
