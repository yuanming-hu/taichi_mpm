import taichi as tc
import os
import math
from async_mpm import AsyncMPM

r = 400
task_id = 'slope'
Async = True

thickness = 0.012

if __name__ == '__main__':
  res = (r, r, r)

  if Async:
    mpm = AsyncMPM(
        res=res,
        particle_collision=False,
        optimized=True,
        max_units=512,
        gravity=(0, -10, 0),
        task_id=task_id,
        num_frames=120)
  else:
    mpm = tc.dynamics.MPM(
        res=res,
        particle_collision=False,
        optimized=True,
        gravity=(0, -10, 0),
        base_delta_t=1e-6 * 8,
        task_id=task_id,
        num_frames=120)

  levelset = mpm.create_levelset()
  levelset.add_slope(tc.Vector(0.7, 0.5, 0.5), 0.4, 30.0 / 180 * math.pi)
  levelset.set_friction(1)
  mpm.set_levelset(levelset, False)

  # Snow Ball
  tex_ball = tc.Texture(
      'sphere', center=(0.2, 0.38 + thickness, 0.5), radius=0.04) * 8
  mpm.add_particles(
      type='snow',
      density_tex=tex_ball.id,
      initial_velocity=(0, 0, 0),
      density=400,)

  # Snow Man
  men_h = [0.0475, 0.095 + 0.0325, 0.095 + 0.065 + 0.02]
  men_r = [0.0475, 0.0325, 0.02]
  for i in range(3):
    tex = tc.Texture(
        'sphere', center=(0.75, 0.1 + men_h[i], 0.5), radius=men_r[i])
    tex = tc.Texture(
        'bound', tex=tex, axis=1, bounds=(0.1, 0.9), outside_val=(0, 0, 0)) * 8
    mpm.add_particles(
        type='snow',
        density_tex=tex.id,
        initial_velocity=(0, 0, 0),
        youngs_modulus=6e5,
        density=400,)

  # Ground
  levelset = mpm.create_levelset()
  levelset.add_slope(tc.Vector(0.7, 0.5, 0.5), 0.4, 30.0 / 180 * math.pi)
  tex = tc.Texture(
      'levelset3d',
      levelset=levelset,
      bounds=(0, thickness / levelset.get_delta_x()))
  tex = tc.Texture(
      'bound', tex=tex, axis=2, bounds=(0.25, 0.75), outside_val=(0, 0, 0))
  tex = tc.Texture(
      'bound', tex=tex, axis=0, bounds=(0.1, 0.9), outside_val=(0, 0, 0))
  tex = tc.Texture(
      'bound', tex=tex, axis=1, bounds=(0.1, 0.9), outside_val=(0, 0, 0))
  tex2 = tc.Texture(
      'sphere', center=(0.75, 0.1 + men_h[0], 0.5), radius=men_r[0])
  tex = (tex - tex2) * 8
  mpm.add_particles(
      type='snow',
      density_tex=tex.id,
      initial_velocity=(0, 0, 0),
      density=100,
      youngs_modulus=2000,
      # Jp=1.5,
  )

  mpm.simulate()
