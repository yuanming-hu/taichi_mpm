import taichi as tc

r = 300

if __name__ == '__main__':
  mpm = tc.dynamics.MPM(
      gravity=(0, -4.0),
      res=(r + 1, r + 1, r + 1),
      frame_dt=0.01,
      base_delta_t=0.0003,
      num_frames=500,)

  levelset = mpm.create_levelset()
  levelset.add_plane(tc.Vector(0.0, 1, 0), -0.2)
  levelset.set_friction(-1)
  mpm.set_levelset(levelset, False)

  tex = tc.Texture('sphere', center=(0.5, 0.45, 0.5), radius=0.04) * 10
  tex = tex.translate(tc.Vector(0.2, 0.45, 0.5))

  tex = tc.Texture(
      'mesh',
      resolution=(2 * r, 2 * r, 2 * r),
      translate=(0.5, 0.56, 0.47),
      scale=(0.4, 0.4, 0.4),
      adaptive=False,
      filename='$mpm/bunny_small.obj') * 15

  mpm.add_particles(
      type='jelly',
      pd=True,
      density_tex=tex.id,
      initial_velocity=(0, 0, 0),
      density=400,
      color=(0.8, 0.7, 1.0),
      initial_position=(0.5, 0.1, 0.5),
      E=1.5e3,
      nu=0.4)

  def position_function(_):
    return tc.Vector(0.5, 0.3, 0.5)

  def rotation_function1(_):
    return tc.Vector(90, 0, 0)

  def rotation_function2(_):
    return tc.Vector(90, 90, 0)

  mpm.add_particles(
      type='rigid',
      density=40,
      scale=(0.4, 0.2, 0.2),
      friction=0,
      scripted_position=tc.function13(position_function),
      scripted_rotation=tc.function13(rotation_function1),
      codimensional=True,
      mesh_fn='$mpm/flat_cutter_medium_res.obj')

  mpm.add_particles(
      type='rigid',
      density=40,
      scale=(0.4, 0.2, 0.2),
      friction=0,
      scripted_position=tc.function13(position_function),
      scripted_rotation=tc.function13(rotation_function2),
      codimensional=True,
      mesh_fn='$mpm/flat_cutter_medium_res.obj')

  mpm.simulate(clear_output_directory=True)
