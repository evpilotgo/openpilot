import os
from pathlib import Path

from jetlink.openpilot.testing import check_adapter, check_fallback, check_seam
from openpilot import jetlink_adapter

MODELD = Path(__file__).parents[1] / 'modeld.py'
HOOKS = ('should_run', 'status', 'reason', 'prepare', 'handed_over', 'state', 'shutdown_step', 'runtime', 'swap_events')


def test_the_adapter():
  assert check_adapter(jetlink_adapter, heavy=('openpilot.common.params', 'openpilot.common.swaglog', 'openpilot.cereal')) == []


def test_the_modeld_seam():
  assert check_seam(MODELD) == []


def test_the_fallback(tmp_path, monkeypatch):
  monkeypatch.setenv('PARAMS_ROOT', str(tmp_path))  # link_on() reads the setting from this test's own store
  assert check_fallback(jetlink_adapter._NoJetlink('jetlink is missing'), HOOKS, link_on=jetlink_adapter.link_on) == []


def test_link_on_reads_the_setting(tmp_path, monkeypatch):
  monkeypatch.setenv('PARAMS_ROOT', str(tmp_path))
  monkeypatch.delenv('OPENPILOT_PREFIX', raising=False)
  assert not jetlink_adapter.link_on()
  (tmp_path / 'd').mkdir()
  for value, on in (('0', False), ('1', True), ('2', True)):
    (tmp_path / 'd' / 'JetlinkLink').write_text(value)
    assert jetlink_adapter.link_on() == on
  assert os.environ['PARAMS_ROOT'] == str(tmp_path)


def test_mads_counts_as_in_control():
  from types import SimpleNamespace as N
  adapter = jetlink_adapter.adapter()
  off = {'carState': N(), 'carControl': N(enabled=False, latActive=False, longActive=False),
         'selfdriveState': N(enabled=False, active=False), 'selfdriveStateSP': N(mads=N(enabled=False))}
  assert 'selfdriveStateSP' in adapter.IN_CONTROL_SERVICES
  assert not adapter.engaged(off)
  assert adapter.engaged(off | {'selfdriveStateSP': N(mads=N(enabled=True))})  # MADS armed or paused
  assert adapter.engaged(off | {'carControl': N(enabled=False, latActive=True, longActive=False)})
