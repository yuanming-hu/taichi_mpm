import taichi as tc
from math import *

r = 400
damping = 3
s = 2
rho = 1
init_vel = -.5

if __name__ == '__main__':
  mpm = tc.dynamics.MPM(
      res=(r + 1, r + 1, r + 1),
      frame_dt=0.01,
      penalty=1e3,
      num_frames=240,
      base_delta_t=5e-5)

  levelset = mpm.create_levelset()
  levelset.set_friction(0.2)
  mpm.set_levelset(levelset, False)

  object1 = mpm.add_particles(
      type='rigid',
      scale=(s, s, s),
      density=rho,
      friction=0.2,
      scripted_position=tc.constant_function13(tc.Vector(0.5, 0.5, 0.5)),
      rotation_axis=(0, 0, 1),
      codimensional=True,
      angular_damping=damping,
      mesh_fn='projects/mpm/data/ww_cylinder.obj')

  object2 = mpm.add_particles(
      type='rigid',
      scale=(s, s, s),
      density=rho,
      friction=0.2,
      scripted_position=tc.constant_function13(tc.Vector(0.5, 0.5, 0.5)),
      rotation_axis=(0, 0, 1),
      codimensional=True,
      angular_damping=damping,
      mesh_fn='projects/mpm/data/ww_gears.obj')

  object3 = mpm.add_particles(
      type='rigid',
      scale=(s, s, s),
      density=rho,
      friction=0.2,
      scripted_position=tc.constant_function13(tc.Vector(0.5, 0.5, 0.5)),
      rotation_axis=(0, 0, 1),
      codimensional=True,
      angular_damping=damping,
      mesh_fn='projects/mpm/data/ww_sides.obj')

  object4 = mpm.add_particles(
      type='rigid',
      scale=(s, s, s),
      density=rho,
      friction=0.2,
      scripted_position=tc.constant_function13(tc.Vector(0.5, 0.5, 0.5)),
      rotation_axis=(0, 0, 1),
      codimensional=True,
      angular_damping=damping,
      mesh_fn='projects/mpm/data/ww_spoke.obj')

  def frame_update(t, frame_dt):
    tex = tc.Texture('ring', outer=0.05)
    tex = tc.Texture(
        'bound', tex=tex, axis=2, bounds=(0.494, 0.506), outside_val=(0, 0, 0))
    tex = tc.Texture('rotate', tex=tex, rotate_axis=0, rotate_times=1)
    tex = tex.translate((0.1, 0.4, 0.0)) * 10
    mpm.add_particles(
        density=1000,
        type='water',
        pd_source=True,
        density_tex=tex.id,
        initial_velocity=(0, init_vel, 0),
        delta_t=frame_dt,
        color=(0.9, 0.5, 0.6))

  mpm.add_articulation(type='rotation', obj0=object1, obj1=object2)
  mpm.add_articulation(type='rotation', obj0=object2, obj1=object3)
  mpm.add_articulation(type='rotation', obj0=object3, obj1=object4)

  mpm.simulate(
      clear_output_directory=True,
      frame_update=frame_update,
      print_profile_info=True)
