"""Navigate in the opview app: the destination it sends, written to NavDestination for navd, and read back for the app.

This is the only thing the app may change on the comma, so it is checked here and nothing else from the app is written.
"""
import math
from typing import Any


def nav_destination(params: Any) -> dict[str, float] | None:
  dest = params.get("NavDestination")
  if not isinstance(dest, dict):
    return None
  try:
    return {"latitude": float(dest["latitude"]), "longitude": float(dest["longitude"])}
  except (KeyError, TypeError, ValueError):
    return None


def set_nav_destination(params: Any, data: Any) -> str | None:
  # the one thing the opview app may change on the comma: where navd routes to. data is {latitude, longitude}, or None
  # to clear it. Returns None when done, else why not.
  if data is None:
    params.remove("NavDestination")
    return None
  if not isinstance(data, dict):
    return "not a destination"
  lat, lon = data.get("latitude"), data.get("longitude")
  if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in (lat, lon)):
    return "latitude and longitude must be numbers"
  if not (-90 <= lat <= 90 and -180 <= lon <= 180):
    return "latitude or longitude out of range"
  params.put("NavDestination", {"latitude": float(lat), "longitude": float(lon)}, block=True)
  return None
