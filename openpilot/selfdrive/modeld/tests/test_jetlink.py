import os
from pathlib import Path

from jetlink.openpilot.testing import check_adapter, check_fallback, check_seam
from openpilot import jetlink_adapter

MODELDS = (Path(__file__).parents[1] / 'modeld.py', Path(__file__).parents[3] / 'sunnypilot' / 'modeld_v2' / 'modeld.py')
HOOKS = ('should_run', 'status', 'reason', 'prepare', 'handed_over', 'state', 'shutdown_step', 'runtime', 'swap_events')


def test_the_adapter():
  assert check_adapter(jetlink_adapter, heavy=('openpilot.common.params', 'openpilot.common.swaglog', 'openpilot.cereal')) == []


def test_the_modeld_seams():  # stock modeld and modeld_tinygrad (a sunnypilot model) both run the hooks
  assert [p for m in MODELDS for p in check_seam(m)] == []


def test_every_modeld_always_passes_action_t():
  """The Jetlink big model is comma's and reads inputs['action_t'] on every frame. modeld_tinygrad used to add it
  only when the selected small model had one, so with GWM V9 (no action_t) every swap died with KeyError."""
  import ast
  for path in MODELDS:
    tree = ast.parse(path.read_text())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    keys = [k.value for n in ast.walk(main) if isinstance(n, ast.AnnAssign | ast.Assign) and isinstance(n.value, ast.Dict)
            and any(getattr(t, 'id', None) == 'inputs' for t in ([n.target] if isinstance(n, ast.AnnAssign) else n.targets))
            for k in n.value.keys if isinstance(k, ast.Constant)]
    assert 'action_t' in keys, path


def test_the_ui_counts_a_jetlink_cable_as_no_chestnut():
  """With Jetlink on, the cable in the USB-C port is the Jetlink host. The UI used to count it as a chestnut at the
  start of a drive, then found none and drew the orange chestnut icon (route 2e5eeb7452e81e85/0000002a)."""
  ui_state = Path(__file__).parents[2] / 'ui' / 'ui_state.py'
  src = ui_state.read_text()
  start = src.index('self.chestnut_present = self.sm["deviceState"].chestnutPresent')
  assert 'jetlink_adapter.link_on()' in src[start:src.index('\n\n', start)]


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
