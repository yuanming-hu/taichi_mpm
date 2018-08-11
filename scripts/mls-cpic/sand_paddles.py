# mpm 3fa953a
# taichi 36046fc

import taichi as tc

r = 501
density = 500
friction = -2
damping = 3

positions = [
    (0.56, 0.66, 0.5),
    (0.44, 0.54, 0.5),
    (0.56, 0.42, 0.5),
    (0.44, 0.30, 0.5),
]

if __name__ == '__main__':
  res = (r, r, r)

  mpm = tc.dynamics.MPM(res=res, penalty=1e4)

  for position in positions:
    s = 0.3
    r = mpm.add_particles(
        type='rigid',
        scale=(s, s, s),
        density=density,
        scripted_position=tc.constant_function13(tc.Vector(*position)),
        codimensional=False,
        reverse_vertices=False,
        friction=friction,
        angular_damping=damping,
        mesh_fn='projects/mpm/data/paddle_complex.obj')

  def frame_update(t, frame_dt):
    tex = tc.Texture('ring', outer=0.02)
    tex = tc.Texture(
        'bound', tex=tex, axis=2, bounds=(0.494, 0.506), outside_val=(0, 0, 0))
    tex = tc.Texture('rotate', tex=tex, rotate_axis=0, rotate_times=1)
    tex = tex.translate((0, 0.4, 0.0)) * 10
    mpm.add_particles(
        type='sand',
        pd_source=True,
        density_tex=tex.id,
        initial_velocity=(0, -1, 0),
        delta_t=frame_dt,
        color=(0.9, 0.5, 0.6))

  mpm.simulate(
      clear_output_directory=True,
      frame_update=frame_update,
      print_profile_info=True)
