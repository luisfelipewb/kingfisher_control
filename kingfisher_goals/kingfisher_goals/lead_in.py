"""A speed and heading controller for a differential-thrust boat, with no ROS."""

from dataclasses import dataclass


def clip(value, low, high):
    """Return value limited to [low, high]."""
    return min(max(value, low), high)


@dataclass
class LeadIn:
    """
    Hold a heading at a forward speed, with left and right thrust in [-1, 1].

    The speed setpoint rises to the target at ``max_accel``, from the speed at ``reset()``, and
    drops to it at once; the integral only runs once the setpoint is on target, so it doesn't wind
    up while the boat speeds up or turns. A positive turn is counter-clockwise: more thrust on the
    right. The defaults are tuned in sim.
    """

    k_heading: float = 1.0      # turn per rad of heading error
    k_yaw_rate: float = 1.0     # turn per rad/s, damping
    max_turn: float = 0.5
    max_accel: float = 0.1      # m/s^2, of the speed setpoint
    feedforward: float = 0.8    # thrust per m/s of setpoint
    k_speed: float = 0.5        # thrust per m/s of speed error
    k_integral: float = 0.2     # thrust per m of speed error
    max_integral: float = 0.2

    def __post_init__(self):
        self.reset(0.0)

    def reset(self, speed):
        """Start over from a forward speed, at the start of a lead-in."""
        self._integral = 0.0
        self._setpoint = speed

    def update(self, heading_error, speed_target, speed, yaw_rate, dt):
        """Return (left, right) thrust for one step of dt s."""
        turn = clip(self.k_heading * heading_error - self.k_yaw_rate * yaw_rate,
                    -self.max_turn, self.max_turn)
        self._setpoint = min(speed_target, self._setpoint + self.max_accel * dt)
        error = self._setpoint - speed
        if self._setpoint == speed_target:
            self._integral = clip(self._integral + self.k_integral * error * dt,
                                  -self.max_integral, self.max_integral)
        thrust = clip(self.feedforward * self._setpoint + self.k_speed * error + self._integral,
                      0.0, 1.0)
        return clip(thrust - turn, -1.0, 1.0), clip(thrust + turn, -1.0, 1.0)
