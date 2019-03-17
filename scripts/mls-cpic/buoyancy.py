import taichi as tc
from math import *
from random import *

r = 320
friction = 0.3

if __name__ == '__main__':
  mpm = tc.dynamics.MPM(
      res=(r + 1, r + 1, r + 1),
      gravity=(0, -10, 0),
      pushing_force=0,
      penalty=1e3,
      rigid_penalty=1e4,
      task_id='buoyancy',
      friction=0)

  walls_pos = [
      tc.Vector(0.5, 0.41, 0.5),
      tc.Vector(0.2, 0.5, 0.5),
      tc.Vector(0.8, 0.5, 0.5),
      tc.Vector(0.5, 0.5, 0.35),
      tc.Vector(0.5, 0.5, 0.65)
  ]
  walls_scale = [(6.5, 0.5, 3.5), (0.5, 2.3, 3.5), (0.5, 2.3, 3.5),
                 (6.5, 2.3, 0.5), (6.5, 2.3, 0.5)]

  for i in range(5):
    pos = walls_pos[i]
    scale = walls_scale[i]
    mpm.add_particles(
        type='rigid',
        density=100000,
        friction=friction,
        scale=scale,
        scripted_position=tc.constant_function13(pos),
        scripted_rotation=tc.constant_function13(tc.Vector(0, 0, 0)),
        codimensional=False,
        mesh_fn='$mpm/cube_smooth_coarse.obj')

  ratio1 = 0.398
  for j in range(2):
    for k in range(3):
      center = (0.56, 0.455 + 0.04 * j, 0.46 + 0.04 * k)
      mpm.add_particles(
          type='rigid',
          density=200 * (k + 1),
          friction=friction,
          scale=(ratio1, ratio1, ratio1),
          initial_position=center,
          codimensional=False,
          mesh_fn='$mpm/cube_smooth_coarse.obj')

  rho0 = 1000
  t0 = 1.6

  def frame_update(t, frame_dt):
    ratio2 = 0.3
    x_pos = 0.28
    y_pos = 0.64
    if (t < t0):
      tex1 = tc.Texture(
          'mesh',
          translate=(x_pos, y_pos, 0.44),
          scale=(ratio2, ratio2, ratio2),
          filename='$mpm/cylinder_jet.obj') * 10
      mpm.add_particles(
          type='water',
          pd_source=True,
          delta_t=frame_dt,
          density_tex=tex1.id,
          initial_velocity=(0, -2, 0),
          density=rho0)
      tex2 = tc.Texture(
          'mesh',
          translate=(x_pos, y_pos, 0.56),
          scale=(ratio2, ratio2, ratio2),
          filename='$mpm/cylinder_jet.obj') * 10
      mpm.add_particles(
          type='water',
          pd_source=True,
          delta_t=frame_dt,
          density_tex=tex2.id,
          initial_velocity=(0, -2, 0),
          density=rho0)

  mpm.simulate(
      clear_output_directory=True,
      update_frequency=3,
      frame_update=frame_update,
      print_profile_info=True)
