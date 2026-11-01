import numpy as np

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL

MAX_BOOST = (0.7, 0.5, 0.3)  # m/s^2 above the model, by driving personality: aggressive, standard, relaxed
MIN_SPEED, LEAD_MIN_SPEED = 20 * CV.MPH_TO_MS, 10 * CV.MPH_TO_MS  # a braking lead can take us below MIN_SPEED, see below
MAX_LAT_ACCEL = 1.0  # m/s^2, off in turns
MODEL_SLOWING, MODEL_BRAKING = -0.1, -0.5  # m/s^2, a model slow-down fades the boost out, braking drops it at once
RAMP_UP, RAMP_DOWN = 0.5, 1.5  # m/s^3
GRACE, GRACE_MARGIN, GRACE_JERK = 3., 0.5, 1.2  # s, m/s^2 of extra model braking it rides through, m/s^3, see below


class SetSpeedAssist:
  """Raises only the model's accel toward the cruise ramp, which tapers to 0 at the set speed. Lead MPC and cruise stay in the min(), model braking wins."""
  def __init__(self):
    self.params, self.enabled, self.frame, self.boost = Params(), False, 0, 0.
    self.engaged, self.lead, self.grace, self.floor, self.ramp, self.v_min = False, False, 0., 0., 0., MIN_SPEED

  def update(self, sm, a_e2e: float, a_cruise: float) -> float:
    if self.frame % int(1. / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SetSpeedAssist")
    self.frame += 1

    cs, action, engaged, lead = sm['carState'], sm['modelV2'].action, sm['selfdriveState'].enabled, sm['radarState'].leadOne.present
    v, a_ego = cs.vEgo, cs.aEgo
    ok = self.enabled and engaged and not action.shouldStop and abs(action.desiredCurvature) * v ** 2 < MAX_LAT_ACCEL
    allowed = ok and v > MIN_SPEED

    # Grace: the model carries a slowdown in progress at an engage, or when a lead we braked for leaves, forward and deepens it
    # at first. For a few seconds a ramp from the truck's accel up to the cruise ramp leads instead, while the model brakes no
    # harder than the truck was then plus a margin; harder braking or a stop hands back at once.
    lead_left = self.lead and not lead and a_ego < MODEL_SLOWING
    if (engaged and not self.engaged) or lead_left:
      self.grace, self.floor, self.ramp, self.v_min = GRACE, min(a_e2e, a_ego) - GRACE_MARGIN, a_ego, LEAD_MIN_SPEED if lead_left else MIN_SPEED
    self.engaged, self.lead, self.grace = engaged, lead, self.grace - DT_MDL if ok and v > self.v_min and a_e2e >= self.floor else 0.
    if self.grace > 0 and a_e2e < MODEL_SLOWING:  # carried as a boost, so it hands back to the model smoothly once the braking fades out
      self.ramp = min(self.ramp + GRACE_JERK * DT_MDL, a_cruise)
      self.boost = max(self.ramp - a_e2e, 0.)
      return a_e2e + self.boost

    if a_e2e < MODEL_BRAKING or action.shouldStop:
      self.boost = 0.
    else:
      target = float(np.clip(a_cruise - a_e2e, 0., MAX_BOOST[sm['selfdriveState'].personality.raw])) if allowed and a_e2e >= MODEL_SLOWING else 0.
      self.boost = float(np.clip(target, self.boost - RAMP_DOWN * DT_MDL, self.boost + RAMP_UP * DT_MDL))
    return a_e2e + self.boost
