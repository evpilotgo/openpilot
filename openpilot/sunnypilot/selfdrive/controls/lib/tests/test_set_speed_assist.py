from types import SimpleNamespace

import pytest

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.set_speed_assist import GRACE, MAX_BOOST, RAMP_UP, SetSpeedAssist


def run(ssa, frames, a_e2e=0.1, a_cruise=1.2, v_mph=40., should_stop=False, curvature=0., enabled=True, personality=0, a_ego=0.):
  sm = {'carState': SimpleNamespace(vEgo=v_mph * CV.MPH_TO_MS, aEgo=a_ego),
        'selfdriveState': SimpleNamespace(enabled=enabled, personality=SimpleNamespace(raw=personality)),
        'modelV2': SimpleNamespace(action=SimpleNamespace(shouldStop=should_stop, desiredCurvature=curvature))}
  return [ssa.update(sm, a_e2e, a_cruise) for _ in range(frames)][-1]


@pytest.mark.parametrize("on, personality", [(True, 0), (True, 1), (True, 2), (False, 0)])
def test_ramps_to_the_personality_cap_under_cruise(on, personality):
  Params().put_bool("SetSpeedAssist", on)
  ssa = SetSpeedAssist()
  assert run(ssa, 1, personality=personality) == pytest.approx(0.1 + RAMP_UP * DT_MDL * on)
  assert run(ssa, 100, personality=personality) == pytest.approx(0.1 + MAX_BOOST[personality] * on)
  assert run(ssa, 100, a_cruise=0.35, personality=personality) == pytest.approx(0.35 if on else 0.1)  # never above the cruise ramp


@pytest.mark.parametrize("kw, at_once", [({"a_e2e": -0.5}, True), ({"a_e2e": -0.15}, True), ({"should_stop": True}, True), ({"v_mph": 15.}, False),
                                         ({"a_cruise": -0.3}, False), ({"curvature": 0.01}, False), ({"enabled": False}, False)])
def test_backs_off(kw, at_once):
  Params().put_bool("SetSpeedAssist", True)
  ssa = SetSpeedAssist()
  run(ssa, 100)
  a_e2e = kw.get("a_e2e", 0.1)
  assert (run(ssa, 1, **kw) == pytest.approx(a_e2e)) == at_once
  assert run(ssa, 100, **kw) == pytest.approx(a_e2e)



SLOWING = {"a_e2e": -0.8, "a_cruise": 0.5, "a_ego": -1.0}  # the model carries the truck's slowdown past the engage


def engaged_ssa():
  Params().put_bool("SetSpeedAssist", True)
  ssa = SetSpeedAssist()
  run(ssa, 1, enabled=False)
  return ssa


@pytest.mark.parametrize("after", [{"a_e2e": -1.6}, {"should_stop": True}, {"v_mph": 15.}])
def test_engage_grace_hands_back_at_once(after):
  ssa = engaged_ssa()
  assert run(ssa, 10, **SLOWING) == 0.5  # the cruise ramp leads
  assert run(ssa, 10, **(SLOWING | {"a_e2e": -1.4})) == 0.5  # also while the model deepens the slowdown a little
  assert run(engaged_ssa(), 1, **SLOWING, v_mph=59.) == 0.5  # also right at the set speed, where it holds the speed
  assert run(ssa, 1, **(SLOWING | after)) == after.get("a_e2e", -0.8)  # harder braking, a stop or low speed
  assert run(ssa, 1, **SLOWING) == -0.8  # and it never re-arms


def test_engage_grace_ends_after_the_window_and_fades_out_smoothly():
  ssa = engaged_ssa()
  assert run(ssa, int(GRACE / DT_MDL) - 1, **SLOWING) == 0.5
  assert run(ssa, 2, **SLOWING) == -0.8
  ssa = engaged_ssa()
  run(ssa, 10, **SLOWING)
  assert min(run(ssa, 1, a_e2e=-0.05, a_cruise=0.5), 0.5) == 0.5  # braking faded out: no step down in the planner's min()


def test_engage_grace_holds_a_slowdown_the_model_starts_after_the_engage():
  ssa = engaged_ssa()
  assert run(ssa, 1, a_e2e=-0.05, a_cruise=0.5) == pytest.approx(-0.05 + RAMP_UP * DT_MDL)  # not slowing yet: the usual ramp
  assert run(ssa, 10, a_e2e=-0.4, a_cruise=0.5, a_ego=-0.5) == 0.5
