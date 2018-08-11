import taichi as tc

cdf = False

r = 125
tc.set_gdb_trigger()

if __name__ == '__main__':
  res = (r, r, r)
  mpm = tc.dynamics.MPM(
      res=res,
      base_delta_t=1e-2,
      gravity=0,
      clean_boundary=False,
      frame_dt=1e-2,
      benchmark_resample=False,
      num_threads=1,
      optimized=True)

  mpm.add_particles(
      #benchmark=125,
      benchmark=8000,
      type='linear',
      initial_velocity=(0, 0, 0),
      E=1e2)

  mpm.simulate(clear_output_directory=True, print_profile_info=True)
