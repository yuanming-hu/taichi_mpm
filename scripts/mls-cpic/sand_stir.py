import taichi as tc

r = 320

if __name__ == '__main__':
  mpm = tc.dynamics.MPM(res=(r + 1, r + 1, r + 1))

  levelset = mpm.create_levelset()
  levelset.add_sphere(tc.Vector(0.5, 0.55, 0.5), 0.3, True)
  levelset.add_plane(tc.Vector(0, 1, 0), -0.35)
  levelset.set_friction(0.2)
  mpm.set_levelset(levelset, False)

  omega = -360
  start_t = 0.2
  end_t = 2.9

  def position_function(t):
    return tc.Vector(0.5, 0.45, 0.5)

  def rotation_function(t):
    if (t < start_t):
      return tc.Vector(0, 0, 0)
    if (start_t <= t < end_t):
      return tc.Vector(0, omega * (t - start_t), 0)
    if (t >= end_t):
      return tc.Vector(0, omega * (end_t - start_t), 0)

  tex0 = tc.Texture(
      'mesh',
      resolution=(2 * r, 2 * r, 2 * r),
      translate=(0.5, 0.6, 0.5),
      scale=(1, 1, 1),
      filename='$mpm/bowl_init_sand.obj') * 10

  mpm.add_particles(
      type='sand',
      density_tex=tex0.id,
      initial_velocity=(0, 0, 0),
      density=400,
      color=(0.8, 0.7, 1.0),
      friction_angle=10)

  mpm.add_particles(
      type='rigid',
      density=40,
      scale=(1, 1, 1),
      friction=0.2,
      scripted_position=tc.function13(position_function),
      scripted_rotation=tc.function13(rotation_function),
      codimensional=True,
      mesh_fn='$mpm/ellipse_cutter.obj')

  def frame_update(t, frame_dt):
    tex = tc.Texture(
        'mesh',
        resolution=(2 * r, 2 * r, 2 * r),
        translate=(0.5, 0.55, 0.22),
        scale=(0.5, 0.5, 0.5),
        filename='$mpm/sphere.obj') * 10

    if (t < end_t - start_t):
      mpm.add_particles(
          type='sand',
          pd_source=True,
          density_tex=tex.id,
          initial_velocity=(0, 0, 1),
          density=400,
          delta_t=frame_dt,
          color=(0.8, 0.7, 1.0),
          friction_angle=10)


mpm.simulate(
    clear_output_directory=True, update_frequency=1, frame_update=frame_update)
