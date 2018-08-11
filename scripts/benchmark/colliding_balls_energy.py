import taichi as tc

cdf = False

r = 100

if __name__ == '__main__':
  res = (r, r, r)
  mpm = tc.dynamics.MPM(
      res=res,
      base_delta_t=3e-4,
      num_frames=60,
      gravity=(0, 0, 0),
      print_energy=True,
      optimized=False)

  tex = tc.Texture('sphere', center=(0.7, 0.45, 0.5), radius=0.08) * 8
  mpm.add_particles(
      type='jelly', density_tex=tex.id, initial_velocity=(-1, 0, 0), E=5e3)

  tex = tc.Texture('sphere', center=(0.3, 0.55, 0.53), radius=0.08) * 8
  mpm.add_particles(
      type='jelly', density_tex=tex.id, initial_velocity=(1, 0, 0), E=5e3)

  e = mpm.simulate_with_energy(
      clear_output_directory=True, print_profile_info=True)
  print(e)
