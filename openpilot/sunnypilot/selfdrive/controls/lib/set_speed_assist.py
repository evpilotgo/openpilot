import numpy as np

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL

MAX_BOOST = (0.7, 0.5, 0.3)  # m/s^2 above the model, by driving personality: aggressive, standard, relaxed
MIN_SPEED = 20 * CV.MPH_TO_MS
MAX_LAT_ACCEL = 1.0  # m/s^2, off in turns
MODEL_SLOWING = -0.1  # m/s^2, any model slow-down drops the boost at once
RAMP_UP, RAMP_DOWN = 0.5, 1.5  # m/s^3
GRACE = 3.  # s after an engage, see below


class SetSpeedAssist:
  """Raises only the model's accel toward the cruise ramp, which tapers to 0 at the set speed. Lead MPC and cruise stay in the min(), model braking wins."""
  def __init__(self):
    self.params, self.enabled, self.frame, self.boost = Params(), False, 0, 0.
    self.engaged, self.grace, self.floor = False, 0., 0.

  def update(self, sm, a_e2e: float, a_cruise: float) -> float:
    if self.frame % int(1. / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SetSpeedAssist")
    self.frame += 1

    v, action, engaged = sm['carState'].vEgo, sm['modelV2'].action, sm['selfdriveState'].enabled
    allowed = self.enabled and engaged and not action.shouldStop and v > MIN_SPEED and abs(action.desiredCurvature) * v ** 2 < MAX_LAT_ACCEL

    # Engage grace: the model carries a slowdown that was in progress at an engage forward. For a few seconds the cruise ramp
    # leads instead, while the model brakes no harder than the truck was then; harder braking or a stop hands back at once.
    if engaged and not self.engaged:
      self.grace, self.floor = GRACE, min(a_e2e, sm['carState'].aEgo)
    self.engaged, self.grace = engaged, self.grace - DT_MDL if allowed and self.floor <= a_e2e < MODEL_SLOWING else 0.
    if self.grace > 0:  # carried as a boost, so it hands back to the model smoothly once the braking fades out
      self.boost = max(a_cruise - a_e2e, 0.)
      return a_e2e + self.boost

    if a_e2e < MODEL_SLOWING or action.shouldStop:
      self.boost = 0.
    else:
      target = float(np.clip(a_cruise - a_e2e, 0., MAX_BOOST[sm['selfdriveState'].personality.raw])) if allowed else 0.
      self.boost = float(np.clip(target, self.boost - RAMP_DOWN * DT_MDL, self.boost + RAMP_UP * DT_MDL))
    return a_e2e + self.boost
