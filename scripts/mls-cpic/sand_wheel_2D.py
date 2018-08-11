import taichi as tc
from math import *

r = 400
damping = 2.7
use_platform = True
use_circle = True
use_blade = True
use_articulation = True
thin_shell_density = 100
FF = -2.4

if __name__ == '__main__':
  res = (r, r)

  mpm = tc.dynamics.MPM(
      res=res, penalty=1e4, pushing_force=0, base_delta_t=1e-4)

  rigids = []

  if use_circle:
    mesh = tc.SegmentMesh()
    edges = []
    segments = 60
    for i in range(segments):
      angle_start = i / segments * -2 * pi
      angle_end = (i + 1) / segments * -2 * pi
      segment = ((cos(angle_start), sin(angle_start)), (cos(angle_end),
                                                        sin(angle_end)))
      mesh.add_segment(segment)

    r = mpm.add_particles(
        type='rigid',
        codimensional=True,
        density=thin_shell_density,
        rotation_axis=(0, 0, 1),
        scripted_position=tc.constant_function((0.5, 0.35)),
        scale=(0.16, 0.16),
        angular_damping=damping,
        friction=FF,
        segment_mesh=mesh)
    rigids.append(r)

  if use_blade:
    # add blade
    mesh = tc.SegmentMesh()
    n_blades = 18
    angle = 2 * pi / n_blades
    for i in range(n_blades):
      b_angle_start = -i * angle
      b_angle_end = -(i + 1) * angle
      segment = (
          (1.7 * cos(b_angle_start), 1.7 * sin(b_angle_start)),
          (3 * cos(b_angle_end), 3 * sin(b_angle_end)),)
      mesh.add_segment(segment)

    rb = mpm.add_particles(
        type='rigid',
        codimensional=True,
        density=thin_shell_density,
        rotation_axis=(0, 0, 1),
        scripted_position=tc.constant_function((0.5, 0.35)),
        scale=(0.08, 0.08),
        friction=FF,
        angular_damping=damping,
        segment_mesh=mesh)
    rigids.append(rb)

  if use_platform:
    mesh = tc.SegmentMesh()
    segment = (
        (0, 0.1),
        (20, 0.1),)
    mesh.add_segment(segment)

    rp = mpm.add_particles(
        type='rigid',
        codimensional=True,
        density=thin_shell_density,
        rotation_axis=(0, 0, 1),
        scripted_position=tc.constant_function((0.1, 0.09)),
        scripted_rotation=tc.constant_function((0, 0)),
        friction=-1,
        scale=(0.1, 0.1),
        segment_mesh=mesh)

  if use_blade and use_circle and use_articulation:
    mpm.add_articulation(type='rotation', obj0=rigids[0], obj1=rigids[1])

  mpm.clear_output_directory()

  def frame_update(t, frame_dt):
    if (t < 2.5):
      tex = tc.Texture('ring', outer=0.05) * 4
      tex = tex.translate((0.1, 0.14)) * (frame_dt * 10)
      mpm.add_particles(
          type='sand',
          pd_source=True,
          friction_angle=45,
          cohesion=0,  #try cohesion in the range 1e-5 to 1e-4
          density_tex=tex.id,
          delta_t=frame_dt,
          initial_velocity=(-1, 0),
          density=1000,
          color=(0.9, 0.5, 0.6))

  mpm.simulate(
      clear_output_directory=True,
      update_frequency=1,
      frame_update=frame_update,
      print_profile_info=False)
