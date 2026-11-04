from openpilot.cereal import custom
from openpilot.selfdrive.car.helpers import convert_carControlSP


def test_carControlSP_fields_the_car_ports_do_not_know_are_left_out():
  # laneGuard is for the opview app; card must still convert the message (it raised on every frame)
  msg = custom.CarControlSP.new_message()
  msg.mads.enabled = True
  msg.leadOne.dRel = 30.0
  msg.laneGuard.enabled = True
  msg.laneGuard.curvature = 0.0004
  cc_sp = convert_carControlSP(msg.as_reader())
  assert cc_sp.mads.enabled is True
  assert cc_sp.leadOne.dRel == 30.0
  assert not hasattr(cc_sp, "laneGuard")
