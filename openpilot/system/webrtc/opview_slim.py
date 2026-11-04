"""Telemetry for the opview app: each service cut down to the fields the app reads, floats rounded, each at its own rate.

Sent whole, webrtcd's data channel carried ~330 kB/s, mostly fields opview never reads. A service missing from
OPVIEW_FIELDS is still sent whole (rounded), so a new readout works before its fields are added here.
"""
import math
from typing import Any

import capnp

UNION = "__union__"  # keep whichever member of the union is set, as {member: fields}

_XYZ = {"x": True, "y": True, "z": True}
_LEAD = {"present": True, "radar": True, "dRel": True, "yRel": True, "vRel": True, "vLead": True}

_GPS = {
  "latitude": 7, "longitude": 7, "speed": True, "bearingDeg": True, "horizontalAccuracy": True, "bearingAccuracyDeg": True,
  "hasFix": True, "unixTimestampMillis": True,
}

# a navRoute is thinned to at most this many points: ~23 kB, well under the data channel's 64 kB message limit
NAV_ROUTE_MAX_POINTS = 1000

# a leaf is True (floats rounded to 3 places) or the number of places to keep, a struct (or a list of structs) is a dict
# of its fields
OPVIEW_FIELDS: dict[str, dict] = {
  "modelV2": {
    "position": _XYZ, "laneLines": _XYZ, "laneLineProbs": True, "roadEdges": _XYZ, "roadEdgeStds": True,
    "acceleration": {"x": True},
    "meta": {"disengagePredictions": {"brakeDisengageProbs": True, "steerOverrideProbs": True}},  # the confidence ball
  },
  "carState": {
    "vEgo": True, "vEgoCluster": True, "vCruiseCluster": True, "aEgo": True,
    "steeringAngleDeg": True, "steeringRateDeg": True, "steeringTorque": True,
    "leftBlinker": True, "rightBlinker": True, "leftBlindspot": True, "rightBlindspot": True,
    "gasPressed": True, "brakePressed": True, "gearShifter": True, "cruiseState": {"enabled": True},
  },
  "selfdriveState": {
    "enabled": True, "engageable": True, "state": True, "personality": True,
    "alertText1": True, "alertText2": True, "alertSize": True, "alertStatus": True,
  },
  "controlsState": {
    "curvature": True, "desiredCurvature": True,
    "lateralControlState": {UNION: {"saturated": True}},
  },
  "radarState": {"leadOne": _LEAD, "leadTwo": _LEAD},
  "longitudinalPlan": {"allowThrottle": True},
  "deviceState": {
    "deviceType": True, "started": True, "cpuTempC": True, "memoryUsagePercent": True,
    "freeSpacePercent": True, "networkStrength": True, "powerDrawW": True,
  },
  "narrowRoadCameraState": {"sensor": True},
  "extrinsicsCalibration": {"rpyCalib": True, "wideFromDeviceEuler": True, "calStatus": True, "calPerc": True, "height": True},
  "longitudinalPlanSP": {"speedLimit": {
    "resolver": {
      "speedLimit": True, "speedLimitLast": True, "speedLimitOffset": True, "speedLimitValid": True,
      "speedLimitLastValid": True, "speedLimitFinalLast": True, "source": True,
    },
    "assist": {"state": True},
  }},
  "liveMapDataSP": {"speedLimitAheadValid": True, "speedLimitAhead": True, "speedLimitAheadDistance": True, "roadName": True},
  "carControl": {"latActive": True, "longActive": True, "actuators": {"accel": True, "steeringAngleDeg": True}},
  "carOutput": {"actuatorsOutput": {"torque": True, "torqueOutputCan": True}},
  "selfdriveStateSP": {"mads": {"state": True, "enabled": True, "available": True}},
  "onroadEvents": {"name": True, "overrideLongitudinal": True, "overrideLateral": True},
  "driverMonitoringState": {
    "activePolicy": True, "isRHD": True, "visionPolicyState": {"faceDetected": True, "awarenessPercent": True, "pose": True},
  },
  # lane guard on the torque bar
  "carControlSP": {"laneGuard": {
    "enabled": True, "active": True, "pausedReason": True, "curvature": 6, "source": True,
    "leftLineGuarded": True, "rightLineGuarded": True, "leftEdgeGuarded": True, "rightEdgeGuarded": True,
    "leftGap": True, "rightGap": True,
  }},
  # Navigate: where the truck is, and navd's route and next turn
  "gpsLocationExternal": _GPS,
  "gpsLocation": _GPS,
  "navInstruction": {
    "maneuverPrimaryText": True, "maneuverSecondaryText": True, "maneuverDistance": True,
    "maneuverType": True, "maneuverModifier": True, "showFull": True,
    "distanceRemaining": True, "timeRemaining": True, "timeRemainingTypical": True, "speedLimit": True,
    "lanes": {"directions": True, "active": True, "activeDirection": True},
    "allManeuvers": {"distance": True, "type": True, "modifier": True},
  },
}

# messages per second. modelV2 drives the redraw and is sent whole-rate, anything unlisted gets 20 Hz
OPVIEW_RATES_HZ: dict[str, float] = {
  "carState": 20., "selfdriveState": 20., "controlsState": 20., "radarState": 20., "carOutput": 20.,
  "carControl": 10., "driverMonitoringState": 10., "selfdriveStateSP": 10., "onroadEvents": 10.,
  "longitudinalPlan": 5., "longitudinalPlanSP": 5.,
  "liveMapDataSP": 2., "deviceState": 2., "extrinsicsCalibration": 2.,
  "narrowRoadCameraState": 1.,
  "carControlSP": 10., "gpsLocationExternal": 5., "gpsLocation": 5., "navInstruction": 2.,
  "navRoute": 5.,  # navd sends it when it changes and every 10 s, all of them go through
}


def min_gap_s(service: str) -> float:
  # 0.9 of the period: the send loop wakes every ~10 ms, a full period would land under the rate
  return 0. if service == "modelV2" else 0.9 / OPVIEW_RATES_HZ.get(service, 20.)


def _plain(value: Any, places: int = 3) -> Any:
  # a capnp value as JSON-ready Python, floats rounded to the millimetre / thousandth (or to `places`)
  if isinstance(value, float):
    return round(value, places) if math.isfinite(value) else value
  if isinstance(value, capnp._DynamicStructReader):
    value = value.to_dict()
  if isinstance(value, dict):
    return {k: _plain(v, places) for k, v in value.items()}
  if isinstance(value, (list, capnp._DynamicListReader)):
    return [_plain(v, places) for v in value]
  if isinstance(value, bytes):
    return value.decode(errors="replace")
  return value if isinstance(value, (bool, int, str)) or value is None else str(value)  # str: enums


def pick(reader: Any, spec: dict) -> Any:
  if isinstance(reader, capnp._DynamicListReader):
    return [pick(item, spec) for item in reader]
  if UNION in spec:
    member = str(reader.which())
    return {member: pick(getattr(reader, member), spec[UNION])}
  return {key: _leaf(getattr(reader, key), sub) if not isinstance(sub, dict) else pick(getattr(reader, key), sub)
          for key, sub in spec.items()}


def _leaf(value: Any, sub: Any) -> Any:
  return _plain(value) if sub is True else _plain(value, sub)


def slim_nav_route(msg: Any) -> dict:
  # [[lat, lon], ...] to the metre, thinned evenly to NAV_ROUTE_MAX_POINTS keeping both ends; [] when there is no route
  coords = [(c.latitude, c.longitude) for c in msg.coordinates]
  if len(coords) > NAV_ROUTE_MAX_POINTS:
    step = (len(coords) - 1) / (NAV_ROUTE_MAX_POINTS - 1)
    coords = [coords[round(i * step)] for i in range(NAV_ROUTE_MAX_POINTS)]
  return {"coordinates": [[round(lat, 5), round(lon, 5)] for lat, lon in coords]}


def slim(service: str, msg: Any) -> Any:
  if service == "navRoute":
    return slim_nav_route(msg)
  spec = OPVIEW_FIELDS.get(service)
  return _plain(msg) if spec is None else pick(msg, spec)
