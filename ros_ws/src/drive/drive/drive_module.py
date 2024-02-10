class DriveModule:
  def __init__(self, name: str, pos_xy_from_centre: tuple, max_velocity: float, max_steering_angle: float):
    self.name = name
    self.pos_xy_from_centre = pos_xy_from_centre
    self.max_velocity = max_velocity
    self.max_steering_angle = max_steering_angle
    self.velocity = 0.0
    self.steering_angle = 0.0
        