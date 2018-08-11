import taichi as tc

cdf = False

r = 100

if __name__ == '__main__':
  res = (r, r, r)
  mpm = tc.dynamics.MPM(res=res, base_delta_t=3e-4, num_frames=50)

  levelset = mpm.create_levelset()
  levelset.add_plane(tc.Vector(0.0, 1, 0), -0.2)
  mpm.set_levelset(levelset)

  tex = tc.Texture('sphere', center=(0.7, 0.45, 0.5), radius=0.08) * 3
  mpm.add_particles(
      type='linear',
      density_tex=tex.id,
      initial_velocity=(-1, 0, 0),
      E=1e4,)

  mpm.simulate(clear_output_directory=True, print_profile_info=True)
