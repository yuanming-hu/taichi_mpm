import taichi as tc
from math import *
from random import *
from async_mpm import AsyncMPM

r = 300
task_id = 'combo'
Async = True

if __name__ == '__main__':
  res = (r, r, r)

  if Async:
    mpm = AsyncMPM(
        res=res,
        particle_collision=False,
        optimized=True,
        verbose_bgeo=True,
        gravity=(0, -10, 0),
        task_id=task_id,
        max_units=128,
        strength_dt_mul=0.8,
        num_frames=240)
  else:
    mpm = tc.dynamics.MPM(
        res=res,
        particle_collision=False,
        optimized=True,
        verbose_bgeo=True,
        gravity=(0, -10, 0),
        base_delta_t=1e-6 * 16,
        task_id=task_id,
        num_frames=240)

  levelset = mpm.create_levelset()
  levelset.add_plane(tc.Vector(0, 0, 1), -0.41)
  levelset.add_plane(tc.Vector(0, 0, -1), 0.59)
  levelset.set_friction(-1)
  mpm.set_levelset(levelset, False)

  tex = tc.Texture('ring', outer=0.025)
  tex = tc.Texture(
      'bound', tex=tex, axis=2, bounds=(0.4, 0.6), outside_val=(0, 0, 0)) * 8
  ys = [0.5, 0.38, 0.26, 0.14]
  xs = [6, 7, 6, 7]
  for i, y in enumerate(ys):
    first = xs[i]
    for j in range(first):
      x = 0.05 + 0.9 / (first + 1) * (j + 1)
      tex_stick = tex.translate((x - 0.5, y - 0.5, 0))
      mpm.add_particles(
          type='elastic',
          E=5e5,
          density_tex=tex_stick.id,)

  tex_dragon = tc.Texture(
      'mesh',
      translate=(0.5, 0.5, 0.5),
      scale=(0.006, 0.006, 0.006),
      adaptive=True,
      filename='$mpm/cute_dragon.obj') * 8
  tex_dragon = tc.Texture(
      'rotate', tex=tex_dragon, rotate_axis=1, rotate_times=2)

  seed(2)
  frame_num = 0

  def frame_update(t, frame_dt):
    global frame_num
    frame_num = frame_num + 1
    if frame_num % 48 == 1:
      start = 0.2
      for i in range(4):
        tex = tex_dragon.rotate_angle(45 - random() * 90 * pi / 180).translate(
            (start + i * 0.2 - 0.5, 0.7 - 0.5, 0))
        kinds = []
        if frame_num == 1:
          kinds = [0, 2, 1, 4]
        if frame_num == 48 + 1:
          kinds = [2, 3, 0, 0]
        if frame_num == 48 * 2 + 1:
          kinds = [2, 0, 1, 3]
        if frame_num == 48 * 3 + 1:
          kinds = [1, 0, 2, 4]
        if frame_num == 48 * 4 + 1:
          continue
        kind = kinds[i]
        if kind == 0:
          mpm.add_particles(
              type='elastic',
              density_tex=tex.id,)
        elif kind == 1:
          mpm.add_particles(
              type='sand',
              density_tex=tex.id,
              lambda_0=20000,
              mu_0=1000,
              friction_angle=10)
        elif kind == 2:
          mpm.add_particles(
              type='snow',
              density_tex=tex.id,)
        elif kind == 3:
          mpm.add_particles(
              type='water',
              density_tex=tex.id,
              gamma=1,
              k=10000,)
        else:
          mpm.add_particles(
              type='von_mises',
              density_tex=tex.id,)

    return

  mpm.simulate(frame_update=frame_update)
