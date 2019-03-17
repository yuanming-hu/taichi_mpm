import taichi as tc
from math import *

r = 240

if __name__ == '__main__':
  use_flip = False
  mpm = tc.dynamics.MPM(
      res=(r + 1, r + 1, r + 1),
      base_delta_t=5e-5,
      num_frames=1000,
      penalty=1e3,
      cfl=0.5,
      rpic_damping=1,)

  levelset = mpm.create_levelset()
  levelset.add_plane(tc.Vector(0, 1, 0), -0.42)
  levelset.add_plane(tc.Vector(1, 0, 0), -0.16)
  levelset.set_friction(-1)
  mpm.set_levelset(levelset, False)

  tex = tc.Texture(
      'mesh',
      resolution=(2 * r, 2 * r, 2 * r),
      translate=(0.4, 0.5, 0.55),
      scale=(0.05, 0.07, 0.07),
      adaptive=False,
      filename='$mpm/banana.obj') * 10

  mpm.add_particles(
      type='von_mises',
      pd=True,
      density_tex=tex.id,
      initial_velocity=(0, 0, 0),
      density=400,
      color=(0.8, 0.7, 1.0),
      initial_position=(0.5, 0.5, 0.5),
      youngs_modulus=4e5,
      poisson_ratio=0.4,
      yield_stress=5)

  length = 0.20
  width = 0.03
  x_pos = 0.6
  y_pos = 0.67
  z_pos = 0.5
  velo = 0.5
  period = (2 * length + 3 * width) / velo

  def position_function(t):
    i = floor(t / period)
    dis = velo * (t - period * i)
    if (dis < length):
      return tc.Vector(x_pos - width * i, y_pos - dis, z_pos)
    if (length <= dis < length + width):
      return tc.Vector(x_pos - width * i + (dis - length), y_pos - length,
                       z_pos)
    if (length + width <= dis < 2 * length + width):
      return tc.Vector(x_pos - width * i + width,
                       y_pos - (2 * length + width - dis), z_pos)
    if (dis >= 2 * length + width):
      return tc.Vector(x_pos - width * i + width - (dis - 2 * length - width),
                       y_pos, z_pos)

  def rotation_function(t):
    return tc.Vector(0, 0, 0)

  mpm.add_particles(
      type='rigid',
      density=40,
      scale=(.95, .95, .95),
      friction=0,
      scripted_position=tc.function13(position_function),
      scripted_rotation=tc.function13(rotation_function),
      codimensional=True,
      mesh_fn='$mpm/flat_cutter_customize.obj')
  mpm.simulate(clear_output_directory=True)
