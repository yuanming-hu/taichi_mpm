import taichi as tc

cdf = False

r = 50

if __name__ == '__main__':
  res = (r, r, r)
  mpm = tc.dynamics.MPM(
      res=res,
      base_delta_t=5e-4,
      num_frames=200,
      gravity=(0, 0, 0),
      print_energy=True,
      optimized=False)

  tex = tc.Texture(
      'mesh',
      resolution=(2 * r, 2 * r, 2 * r),
      translate=(0.5, 0.5, 0.5),
      scale=(0.4, 0.4, 0.4),
      adaptive=False,
      filename='../../data/box.obj') * 8

  mpm.add_particles(
      type='jelly',
      density_tex=tex.id,
      initial_velocity=(0, 0, 0),
      E=1e3,
      initial_dg=1.1)

  e = mpm.simulate_with_energy(
      clear_output_directory=True, print_profile_info=True)
  print(e)
