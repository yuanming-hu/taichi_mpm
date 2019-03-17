import taichi as tc
from math import *
from random import *
from async_mpm import AsyncMPM

r = 400
task_id = 'water'
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
        num_frames=240)
  else:
    mpm = tc.dynamics.MPM(
        res=res,
        particle_collision=False,
        optimized=True,
        verbose_bgeo=True,
        gravity=(0, -1, 0),
        base_delta_t=1e-6 * 16,
        task_id=task_id,
        num_frames=240)

  levelset = mpm.create_levelset()
  levelset.add_cuboid((0, 0.2, 0.05), (0.95, 0.95, 0.95), True)
  levelset.set_friction(-2.5)
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
      type='elastic',
      density_tex=tex.id,)

  tex = tc.Texture('ring', outer=0.03) * 8
  tex = tc.Texture(
      'bound', tex=tex, axis=2, bounds=(0.45, 0.55), outside_val=(0, 0, 0))
  tex = tc.Texture(
      'rotate', tex=tex, rotate_axis=1, rotate_times=1).translate((-0.4, -0.1,
                                                                   0))

  def frame_update(t, frame_dt):
    if t < 0.3:
      mpm.add_particles(
          type='water',
          gamma=1,
          k=10000,
          pd_source=True,
          delta_t=frame_dt,
          density_tex=tex.id,
          initial_velocity=(2, 0, 0),)

  mpm.simulate(frame_update=frame_update)
