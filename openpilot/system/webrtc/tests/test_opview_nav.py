import math

from openpilot.system.webrtc.opview_nav import nav_destination, set_nav_destination


class FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get(self, key):
    return self.values.get(key)

  def put(self, key, value, block=False):
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)


def test_sets_and_clears_the_destination():
  p = FakeParams()
  assert set_nav_destination(p, {"latitude": 32.7766649, "longitude": -96.7969883}) is None
  assert p.values["NavDestination"] == {"latitude": 32.7766649, "longitude": -96.7969883}
  assert nav_destination(p) == {"latitude": 32.7766649, "longitude": -96.7969883}
  assert set_nav_destination(p, {"latitude": 33, "longitude": -97}) is None  # whole degrees are numbers too
  assert p.values["NavDestination"] == {"latitude": 33.0, "longitude": -97.0}
  assert set_nav_destination(p, None) is None
  assert "NavDestination" not in p.values and nav_destination(p) is None


def test_refuses_anything_else():
  for bad in ("32,-96", [32, -96], {"latitude": 32}, {"latitude": "32", "longitude": "-96"},
              {"latitude": True, "longitude": False}, {"latitude": math.nan, "longitude": 0},
              {"latitude": math.inf, "longitude": 0}, {"latitude": 91, "longitude": 0}, {"latitude": 0, "longitude": -181}):
    p = FakeParams({"NavDestination": {"latitude": 1.0, "longitude": 2.0}})
    assert set_nav_destination(p, bad) is not None, bad
    assert p.values["NavDestination"] == {"latitude": 1.0, "longitude": 2.0}, bad  # the old one stays


def test_reads_back_only_a_well_formed_destination():
  assert nav_destination(FakeParams({"NavDestination": {"latitude": 1, "longitude": 2, "place_name": "x"}})) == \
    {"latitude": 1.0, "longitude": 2.0}
  assert nav_destination(FakeParams({"NavDestination": "junk"})) is None
  assert nav_destination(FakeParams({"NavDestination": {"latitude": 1}})) is None
