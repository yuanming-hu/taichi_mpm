import taichi as tc
from math import *

r = 240

if __name__ == '__main__':
  mpm = tc.dynamics.MPM(
      res=(r + 1, r + 1, r + 1),
      frame_dt=0.01,
      base_delta_t=1e-4,
      gravity=(0, -10, 0),
      pushing_force=0,
      penalty=5e4,
      rpic_damping=1,
      cfl=0.5,)

  levelset = mpm.create_levelset()
  levelset.add_plane(tc.Vector(0, 1, 0), -0.445)
  levelset.set_friction(-1)
  mpm.set_levelset(levelset, False)

  ratio = 0.5
  rho = (400, 2000, 5000)

  tex = tc.Texture('rect', bounds=(0.4, 0.1, 0.2)) * 20

  mpm.add_particles(
      type='von_mises',
      density_tex=tex.id,
      density=rho[0],
      youngs_modulus=5e4,
      poisson_ratio=0.4,
      yield_stress=10)

  period = 0.3

  def frame_update(t, frame_dt):
    id = floor(t / period)
    if (id < 3 and 0 < t <= id * period + frame_dt):
      mpm.add_particles(
          type='rigid',
          density=rho[id],
          scale=(ratio, ratio, ratio),
          friction=-1,
          initial_position=(0.375 + id * 0.125, 0.7, 0.5),
          initial_rotation=(40, 50, 0),
          codimensional=False,
          mesh_fn='$mpm/cube_smooth.obj')

  mpm.simulate(clear_output_directory=True, frame_update=frame_update)
