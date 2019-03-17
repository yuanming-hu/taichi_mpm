import taichi as tc
from math import *

r = 301
num_wheels = 6

body_friction = 0
leg_friction = -2
leg_vel = -25
rod_friction = 0
reversed_legs = False

assert num_wheels % 2 == 0

center_position = tc.Vector(0.22, 0.64, 0.5)
battery_position = tc.Vector(0.3, 0.64, 0.5)
battery_option = True

robot_scale = 1.6

damping = 0
thin_shell_density = 12

densities = [
    thin_shell_density, thin_shell_density, thin_shell_density,
    thin_shell_density, thin_shell_density, thin_shell_density
]
offsets = [
    tc.Vector(0.042, -0.004, 0.024) * robot_scale,
    tc.Vector(0, -0.004, 0.035) * robot_scale,
    tc.Vector(-0.042, -0.004, 0.024) * robot_scale,
    tc.Vector(0.042, -0.004, -0.024) * robot_scale,
    tc.Vector(0, -0.004, -0.035) * robot_scale,
    tc.Vector(-0.042, -0.004, -0.024) * robot_scale,
]

rod_lengths = [0.015 * robot_scale, 0.02 * robot_scale, 0.015 * robot_scale]

if __name__ == '__main__':
  res = (r, r, r)

  mpm = tc.dynamics.MPM(
      res=res,
      pushing_force=0,
      num_frames=200,
      sand_crawler=True,
      rigid_body_collision=False,
      print_rigid_body_state=False)

  ###########################################################################
  # add levelset walls
  ###########################################################################
  levelset = mpm.create_levelset()
  # levelset.add_cuboid((0.5-0.45, 0.4, 0.35), (0.5, 0.9, 0.65), True)
  levelset.add_cuboid((0.5 - 0.45, 0.465, 0.35), (0.5 + 0.45, 0.9, 0.65), True)
  levelset.set_friction(-2)
  mpm.set_levelset(levelset, False)

  ###########################################################################
  # add sand
  ###########################################################################
  # tex = tc.Texture('rect', bounds=(0.45, 0.1, 0.3)).translate((-0.225,0,0)) * 8
  tex = tc.Texture('rect', bounds=(0.9, 0.07, 0.3)) * 8

  mpm.add_particles(
      type='sand',
      pd=True,
      friction_angle=45,
      density_tex=tex.id,
      lambda_0=204057.0 * 5,
      mu_0=136038.0 * 5,
      density=2000)

  def frame_update(t, frame_dt):
    if t < 0.2 - 0.0001 or t > 0.2 + 0.0001:
      return

    ###########################################################################
    # add robot
    ###########################################################################
    rigids = []

    body = mpm.add_particles(
        type='rigid',
        codimensional=True,
        density=thin_shell_density * 12,
        initial_position=center_position,
        scale=(0.12 * robot_scale, 0.016 * robot_scale, 0.02 * robot_scale),
        angular_damping=0,
        mesh_fn='$mpm/box.obj',
        friction=body_friction)

    if battery_option:
      battery = mpm.add_particles(
          type='rigid',
          codimensional=True,
          density=thin_shell_density * 5,
          initial_position=battery_position,
          scale=(0.008 * robot_scale, 0.008 * robot_scale, 0.008 * robot_scale),
          angular_damping=0,
          mesh_fn='$mpm/sphere_small.obj',
          friction=body_friction)
      mpm.add_articulation(
          type='distance',
          obj0=body,
          obj1=battery,
          offset0=battery_position - center_position)

    legs = []
    for i in range(num_wheels):
      s = 0.01
      sign = (i % 2 * 2 - 1)
      if reversed_legs:
        wheel_offset = tc.Vector(-s * sign * 0.5, 1.5 * s * sign,
                                 0) * robot_scale
        r = mpm.add_particles(
            type='rigid',
            codimensional=True,
            density=densities[i],
            initial_position=center_position + offsets[i] + wheel_offset,
            scale=(-s * sign * robot_scale, s * robot_scale * 1.5,
                   s * robot_scale),
            mesh_fn='$mpm/robot_leg.obj',
            friction=leg_friction,
            angular_damping=0)
      else:
        wheel_offset = tc.Vector(s * sign * 0.5, 1.5 * s * sign,
                                 0) * robot_scale
        r = mpm.add_particles(
            type='rigid',
            codimensional=True,
            density=densities[i],
            initial_position=center_position + offsets[i] + wheel_offset,
            scale=(s * sign * robot_scale, s * robot_scale * 1.5,
                   s * robot_scale),
            mesh_fn='$mpm/robot_leg.obj',
            friction=leg_friction,
            angular_damping=0)

      legs.append(r)
      mpm.add_articulation(
          type='stepper',
          obj0=r,
          obj1=body,
          angular_velocity=leg_vel,
          axis=(0, 0, 1),
          offset0=-wheel_offset)

    for i in range(num_wheels // 2):
      t = 0.002 * robot_scale
      r = mpm.add_particles(
          type='rigid',
          codimensional=True,
          density=thin_shell_density,
          initial_position=center_position +
          (offsets[i] + offsets[i + num_wheels // 2]) * 0.5,
          initial_rotation=(0, 90, 0),
          scale=(rod_lengths[i], t, t),
          mesh_fn='$mpm/cylinder_horizental.obj',
          friction=rod_friction,)
      mpm.add_articulation(
          type='stepper',
          obj0=r,
          obj1=body,
          angular_velocity=leg_vel,
          axis=(0, 0, 1),
          power=1)

  mpm.simulate(
      clear_output_directory=True,
      print_profile_info=True,
      frame_update=frame_update)
