import json

from openpilot.cereal import log
from openpilot.system.webrtc import opview_slim


def _reader(service, build):
  ev = log.Event.new_message()
  build(ev.init(service))
  return getattr(ev.as_reader(), service)


def test_every_listed_field_exists():
  # a typo or a schema change must fail here, not leave the app without a readout
  for service in opview_slim.OPVIEW_FIELDS:
    ev = log.Event.new_message()
    ev.init(service, 1) if service == "onroadEvents" else ev.init(service)
    json.dumps(opview_slim.slim(service, getattr(ev.as_reader(), service)))


def test_keeps_only_listed_fields_and_rounds():
  def build(cs):
    cs.vEgo = 17.123456789
    cs.gearShifter = "drive"
    cs.cruiseState.enabled = True
    cs.steeringPressed = True  # not read by opview
  out = opview_slim.slim("carState", _reader("carState", build))
  assert out["vEgo"] == 17.123 and out["gearShifter"] == "drive" and out["cruiseState"] == {"enabled": True}
  assert "steeringPressed" not in out


def test_torque_bar_inputs_keep_their_precision():
  def build(cs):
    cs.curvature = 0.000351234
    cs.desiredCurvature = -0.000412345
    cs.lateralControlState.init("angleState")
  out = opview_slim.slim("controlsState", _reader("controlsState", build))
  # 3 places would make both 0
  assert out["curvature"] == 0.000351 and out["desiredCurvature"] == -0.000412
  out = opview_slim.slim("vehicleParameters", _reader("vehicleParameters", lambda vp: setattr(vp, "roll", 0.0261799)))
  assert out == {"roll": 0.02618}


def test_union_lists_and_model():
  out = opview_slim.slim("controlsState", _reader("controlsState", lambda cs: cs.lateralControlState.init("torqueState")))
  assert out["lateralControlState"] == {"torqueState": {"saturated": False}}
  ev = log.Event.new_message()
  ev.init("onroadEvents", 1)[0].overrideLongitudinal = True
  assert opview_slim.slim("onroadEvents", ev.as_reader().onroadEvents)[0]["overrideLongitudinal"] is True

  def build(m):
    m.position.x = [1.23456]
    m.init("laneLines", 1)[0].y = [0.98765]
  out = opview_slim.slim("modelV2", _reader("modelV2", build))
  assert out["position"]["x"] == [1.235] and out["laneLines"][0]["y"] == [0.988]


def test_unlisted_service_sent_whole_and_rates():
  out = opview_slim.slim("lateralDelay", _reader("lateralDelay", lambda d: setattr(d, "lateralDelay", 0.123456789)))
  assert out["lateralDelay"] == 0.123
  assert opview_slim.min_gap_s("modelV2") == 0.
  assert opview_slim.min_gap_s("longitudinalPlan") > opview_slim.min_gap_s("carState") == 0.9 / 20.


def test_gps_keeps_position_to_the_centimetre():
  def build(g):
    g.latitude, g.longitude, g.bearingDeg, g.hasFix = 32.776664912, -96.796988334, 271.23456, True
    g.altitude = 150.0  # not read by opview
  out = opview_slim.slim("gpsLocationExternal", _reader("gpsLocationExternal", build))
  assert out["latitude"] == 32.7766649 and out["longitude"] == -96.7969883
  assert out["bearingDeg"] == 271.235 and out["hasFix"] is True and "altitude" not in out


def test_lane_guard_state():
  def build(cc):
    lg = cc.laneGuard
    lg.enabled = lg.active = lg.leftLineGuarded = True
    lg.curvature = 0.000412345
    lg.source = "laneLine"
    lg.pausedReason = "none"
    lg.leftGap = 0.123456
  out = opview_slim.slim("carControlSP", _reader("carControlSP", build))
  assert set(out) == {"laneGuard"}  # no leads or params
  lg = out["laneGuard"]
  assert lg["curvature"] == 0.000412 and lg["source"] == "laneLine" and lg["pausedReason"] == "none"
  assert lg["leftLineGuarded"] is True and lg["rightEdgeGuarded"] is False and lg["leftGap"] == 0.123
  json.dumps(out)


def test_nav_route_is_thinned_and_keeps_its_ends():
  n = 5000
  def build(r):
    coords = r.init("coordinates", n)
    for i, c in enumerate(coords):
      c.latitude, c.longitude = 32.0 + i * 1e-4, -97.0 - i * 1e-4
  out = opview_slim.slim("navRoute", _reader("navRoute", build))
  pts = out["coordinates"]
  assert len(pts) == opview_slim.NAV_ROUTE_MAX_POINTS
  assert pts[0] == [32.0, -97.0] and pts[-1] == [round(32.0 + (n - 1) * 1e-4, 5), round(-97.0 - (n - 1) * 1e-4, 5)]
  assert len(json.dumps(out)) < 60_000
  assert opview_slim.slim("navRoute", _reader("navRoute", lambda r: r.init("coordinates", 3)))["coordinates"] == [[0.0, 0.0]] * 3
  assert opview_slim.slim("navRoute", _reader("navRoute", lambda r: None)) == {"coordinates": []}


def test_nav_instruction():
  def build(ni):
    ni.maneuverPrimaryText = "Main St"
    ni.maneuverDistance = 152.4567
    ni.maneuverType = "turn"
    ni.maneuverModifier = "left"
    m = ni.init("allManeuvers", 1)[0]
    m.distance, m.type, m.modifier = 152.4567, "turn", "left"
  out = opview_slim.slim("navInstruction", _reader("navInstruction", build))
  assert out["maneuverPrimaryText"] == "Main St" and out["maneuverDistance"] == 152.457
  assert out["allManeuvers"] == [{"distance": 152.457, "type": "turn", "modifier": "left"}]
