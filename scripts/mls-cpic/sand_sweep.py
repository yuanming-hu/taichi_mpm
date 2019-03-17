import taichi as tc
from math import *

r = 500

if __name__ == '__main__':
  mpm = tc.dynamics.MPM(
      res=(r + 1, r + 1, r + 1),
      frame_dt=0.01,
      base_delta_t=2e-5,
      pushing_force=20000,)

  levelset = mpm.create_levelset()
  levelset.add_plane(tc.Vector(0, 1, 0), -0.4)
  levelset.add_plane(tc.Vector(1, 0, 0), -0.1)
  levelset.add_plane(tc.Vector(-1, 0, 0), 0.9)
  levelset.add_plane(tc.Vector(0, 0, 1), -0.1)
  levelset.add_plane(tc.Vector(0, 0, -1), 0.9)
  levelset.set_friction(0.4)
  mpm.set_levelset(levelset, False)

  #tex = tc.Texture('rect', bounds=(0.6, 0.01, 0.6)) * 10
  #tex = tex.translate((0, -0.1, 0))

  tex = tc.Texture(
      'mesh',
      translate=(0.5, 0.43, 0.5),
      scale=(0.7, 0.4, 0.7),
      adaptive=False,
      filename='$mpm/sand_mountain.obj') * 8

  mpm.add_particles(
      type='sand',
      pd=True,
      density_tex=tex.id,
      initial_position=(0, 0, 0),
      initial_velocity=(0, 0, 0),
      friction_angle=35,
      density=400,
      color=(0.9, 0.5, 0.6))

  def position_function(t):
    if (t < 0.1):
      return tc.Vector(0.5, 0.45, 0.3)
    else:
      return tc.Vector(0.5, 0.45, 0.3 + 0.6 * (t - 0.1))

  def rotation_function(t):
    if (t < 0.1):
      return tc.Vector(0, 0, 90)
    else:
      return tc.Vector(15 * 100 * (t - 0.1), 0, 90)

  mpm.add_particles(
      type='rigid',
      density=40,
      scale=(0.14, 0.1, 0.1),
      friction=0,
      scripted_position=tc.function13(position_function),
      scripted_rotation=tc.function13(rotation_function),
      codimensional=True,
      mesh_fn='$mpm/flat_cutter_high_res.obj')

  mpm.simulate(clear_output_directory=True)
