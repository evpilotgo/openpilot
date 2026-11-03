"""Jetlink: the large driving model on a computer plugged into the comma's USB-C port (github.com/zoompilot/jetlink).

manager imports this to build its process list and runs it as the gadget owner, so its top level imports the
standard library only. Every hook answers as the link off when jetlink fails, so it never takes a process down.
"""
import functools
import os

API = 3              # the jetlink.openpilot API this adapter is written to
OWNER = 'jetlinkd'   # manager's name for the gadget owner


def _adapter_class():
  from jetlink.openpilot.stock import Stock

  class Adapter(Stock):
    """Stock openpilot, with MADS: lateral-only driving, armed or paused, counts as in control."""
    IN_CONTROL_SERVICES = Stock.IN_CONTROL_SERVICES + ('selfdriveStateSP',)

    def engaged(self, sm) -> bool:
      return super().engaged(sm) or sm['selfdriveStateSP'].mads.enabled

  return Adapter


def adapter():
  return _adapter_class()()


def owner_config():
  return _adapter_class().owner_config()


def main():
  _adapter_class().main()


def link_on() -> bool:
  """The Jetlink setting, read as a file, so it works with no jetlink at all."""
  prefix = os.environ.get('OPENPILOT_PREFIX', '')
  root = os.environ.get('PARAMS_ROOT') or ('/data/params' if os.path.isfile('/AGNOS') else
                                           os.path.expanduser(f'~/.comma{prefix}/params'))
  try:
    with open(os.path.join(root, os.environ.get('OPENPILOT_PREFIX', 'd'), 'JetlinkLink')) as f:
      return f.read().strip() not in ('', '0')
  except OSError:
    return False


def _cloudlog():
  from openpilot.common.swaglog import cloudlog
  return cloudlog


class _NoJetlink:
  """The hooks with no usable jetlink checkout: the link off on every one."""
  def __init__(self, why): self._why = why
  def should_run(self, started, params, CP): return False
  def status(self): return None
  def reason(self): return self._why if link_on() else None
  def prepare(self, chestnut): return False
  def handed_over(self, model): return False
  def state(self, model): return 'none'
  def shutdown_step(self, should_shutdown, reason=''): return should_shutdown
  def runtime(self): return None
  def swap_events(self, tick_s): return None


@functools.cache
def hooks():
  """What every hook line calls, one per process. Never raises."""
  try:
    from jetlink.openpilot.hooks import Hooks
    return Hooks(API, adapter, link_on=link_on, log=_cloudlog)
  except Exception as e:
    try:
      from jetlink.openpilot.hooks import Off
    except Exception:
      return _NoJetlink(f"jetlink is missing: {e}")
    return Off(str(e), link_on=link_on)


def should_run(started, params, CP) -> bool:
  return hooks().should_run(started, params, CP)
