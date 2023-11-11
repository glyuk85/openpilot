#!/usr/bin/env python3
import capnp
import os
import importlib
import pytest
import random
import unittest
from collections import defaultdict, Counter
from typing import List, Optional, Tuple
from parameterized import parameterized_class
import hypothesis.strategies as st
from hypothesis.stateful import RuleBasedStateMachine, rule, precondition
from hypothesis import HealthCheck, Phase, assume, given, settings

from cereal import messaging, log, car
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.fingerprints import all_known_cars
from openpilot.selfdrive.car.car_helpers import FRAME_FINGERPRINT, interfaces
from openpilot.selfdrive.car.gm.values import CAR as GM
from openpilot.selfdrive.car.honda.values import CAR as HONDA, HONDA_BOSCH
from openpilot.selfdrive.car.hyundai.values import CAR as HYUNDAI
from openpilot.selfdrive.car.tests.routes import non_tested_cars, routes, CarTestRoute
from openpilot.selfdrive.controls.controlsd import Controls
from openpilot.selfdrive.test.openpilotci import get_url
from openpilot.tools.lib.logreader import LogReader
from openpilot.tools.lib.route import Route, SegmentName, RouteName

from panda.tests.libpanda import libpanda_py

EventName = car.CarEvent.EventName
PandaType = log.PandaState.PandaType
SafetyModel = car.CarParams.SafetyModel

NUM_JOBS = int(os.environ.get("NUM_JOBS", "1"))
JOB_ID = int(os.environ.get("JOB_ID", "0"))
INTERNAL_SEG_LIST = os.environ.get("INTERNAL_SEG_LIST", "")
INTERNAL_SEG_CNT = int(os.environ.get("INTERNAL_SEG_CNT", "0"))

ignore_addr_checks_valid = [
  GM.BUICK_REGAL,
  HYUNDAI.GENESIS_G70_2020,
]


def get_test_cases() -> List[Tuple[str, Optional[CarTestRoute]]]:
  # build list of test cases
  test_cases = []
  if not len(INTERNAL_SEG_LIST):
    routes_by_car = defaultdict(set)
    for r in routes:
      routes_by_car[r.car_model].add(r)

    for i, c in enumerate(sorted(all_known_cars())):
      if i % NUM_JOBS == JOB_ID:
        test_cases.extend(sorted((c, r) for r in routes_by_car.get(c, (None,))))

  else:
    with open(os.path.join(BASEDIR, INTERNAL_SEG_LIST), "r") as f:
      seg_list = f.read().splitlines()

    cnt = INTERNAL_SEG_CNT or len(seg_list)
    seg_list_iter = iter(seg_list[:cnt])

    for platform in seg_list_iter:
      platform = platform[2:]  # get rid of comment
      segment_name = SegmentName(next(seg_list_iter))
      test_cases.append((platform, CarTestRoute(segment_name.route_name.canonical_name, platform,
                                                segment=segment_name.segment_num)))
  return test_cases


init_done = False

@pytest.mark.slow
class CarModelBase(RuleBasedStateMachine):
  car_model: Optional[str] = 'TOYOTA CAMRY 2021'
  test_route: Optional[CarTestRoute] = CarTestRoute("3456ad0cd7281b24|2020-12-13--17-45-56", 'TOYOTA CAMRY 2021')
  ci: bool = True

  can_msgs: List[capnp.lib.capnp._DynamicStructReader]
  elm_frame: Optional[int]

  def __init__(self):
    super().__init__()
    global init_done
    if init_done:
      return
    init_done = True
    # if self.__name__ == 'TestCarModel' or self.__name__.endswith('Base'):
    #   raise unittest.SkipTest

    if 'FILTER' in os.environ:
      if not self.car_model.startswith(tuple(os.environ.get('FILTER').split(','))):
        raise unittest.SkipTest

    if self.test_route is None:
      if self.car_model in non_tested_cars:
        print(f"Skipping tests for {self.car_model}: missing route")
        raise unittest.SkipTest
      raise Exception(f"missing test route for {self.car_model}")

    test_segs = (2, 1, 0)
    if self.test_route.segment is not None:
      test_segs = (self.test_route.segment,)

    for seg in test_segs:
      try:
        if len(INTERNAL_SEG_LIST):
          route_name = RouteName(self.test_route.route)
          lr = LogReader(f"cd:/{route_name.dongle_id}/{route_name.time_str}/{seg}/rlog.bz2")
        elif self.ci:
          lr = LogReader(get_url(self.test_route.route, seg))
        else:
          lr = LogReader(Route(self.test_route.route).log_paths()[seg])
      except Exception:
        continue

      car_fw = []
      can_msgs = []
      self.elm_frame = None
      fingerprint = defaultdict(dict)
      experimental_long = False
      enabled_toggle = True
      dashcam_only = False
      for msg in lr:
        if msg.which() == "can":
          can_msgs.append(msg)
          if len(can_msgs) <= FRAME_FINGERPRINT:
            for m in msg.can:
              if m.src < 64:
                fingerprint[m.src][m.address] = len(m.dat)

        elif msg.which() == "carParams":
          car_fw = msg.carParams.carFw
          dashcam_only = msg.carParams.dashcamOnly
          if msg.carParams.openpilotLongitudinalControl:
            experimental_long = True
          if self.car_model is None and not self.ci:
            self.car_model = msg.carParams.carFingerprint

        elif msg.which() == 'initData':
          for param in msg.initData.params.entries:
            if param.key == 'OpenpilotEnabledToggle':
              enabled_toggle = param.value.strip(b'\x00') == b'1'

        # Log which can frame the panda safety mode left ELM327, for CAN validity checks
        if msg.which() == 'pandaStates':
          for ps in msg.pandaStates:
            if self.elm_frame is None and ps.safetyModel != SafetyModel.elm327:
              self.elm_frame = len(can_msgs)

        elif msg.which() == 'pandaStateDEPRECATED':
          if self.elm_frame is None and msg.pandaStateDEPRECATED.safetyModel != SafetyModel.elm327:
            self.elm_frame = len(can_msgs)

      if len(can_msgs) > int(50 / DT_CTRL):
        break
    else:
      raise Exception(f"Route: {repr(self.test_route.route)} with segments: {test_segs} not found or no CAN msgs found. Is it uploaded?")

    # if relay is expected to be open in the route
    self.openpilot_enabled = enabled_toggle and not dashcam_only

    self.can_msgs = sorted(can_msgs, key=lambda msg: msg.logMonoTime)

    self.CarInterface, self.CarController, self.CarState = interfaces[self.car_model]
    self.CP = self.CarInterface.get_params(self.car_model, fingerprint, car_fw, experimental_long, docs=False)
    assert self.CP
    assert self.CP.carFingerprint == self.car_model

    self.car_state_dict = {'gas_pressed': False}
    print("HERE!!!!!")

    # print('SETUP HEREHEREHEREHEREHEREHEREHEREHEREHEREHEREHEREHEREHEREHEREHEREHEREHERE')
    self.CI = self.CarInterface(self.CP.copy(), self.CarController, self.CarState)
    assert self.CI

    Params().put_bool("OpenpilotEnabledToggle", self.openpilot_enabled)

    # TODO: check safetyModel is in release panda build
    self.safety = libpanda_py.libpanda

    cfg = self.CP.safetyConfigs[-1]
    set_status = self.safety.set_safety_hooks(cfg.safetyModel.raw, cfg.safetyParam)
    assert 0 == set_status, f"failed to set safetyModel {cfg}"
    self.safety.init_tests()

  @rule(messages=st.lists(st.tuples(
    st.integers(min_value=0, max_value=2),
    st.integers(min_value=0, max_value=0x1FFFF),
    st.binary(min_size=8, max_size=8)
  )))
  def send_messages(self, messages):
    print(messages)
    for bus, address, dat in messages:
      to_send = libpanda_py.make_CANPacket(address, bus, dat)
      self.safety.safety_rx_hook(to_send)

      can = messaging.new_message('can', 1)
      can.can = [log.CanData(address=address, dat=dat, src=bus)]

      CC = car.CarControl.new_message()
      prev_car_state = self.car_state.copy()
      self.car_state = self.CI.update(CC, (can.to_bytes(),))

      # Check for state change.
      self.check_state_change(prev_car_state, self.car_state)

      # Only start asserting once there's been a state change.
      if self.has_state_changed:
        # Assertions based on your criteria here
        # For example:
        assert self.car_state['gas_pressed'] == self.initial_car_state['gas_pressed'], \
          "Mismatch in gas pressed state."

  # @settings(max_examples=100, deadline=None,
  #           # phases=(Phase.reuse, Phase.generate, Phase.shrink),
  #           suppress_health_check=[HealthCheck.filter_too_much, HealthCheck.too_slow],
  #           )
  # @given(data=st.data())
  # def test_panda_safety_carstate_fuzzy(self, data):
  #   state_has_changed = lambda prev_state, new_state: prev_state != new_state
  #   # cfg = self.CP.safetyConfigs[-1]
  #   # set_status = self.safety.set_safety_hooks(cfg.safetyModel.raw, cfg.safetyParam)
  #   # self.assertEqual(0, set_status, f"failed to set safetyModel {cfg}")
  #   # self.safety.init_tests()
  #
  #   # bus = 0  # random.randint(0, 3)
  #   # address = 0xaa  # random.randint(0x200, 0x300)
  #
  #   address = data.draw(st.integers(0x1ff, 0x250))
  #   bus = 0
  #
  #   # ORIG:
  #   # msg_strategy = st.tuples(st.integers(min_value=0, max_value=0), st.integers(min_value=0x100, max_value=0x400), st.binary(min_size=8, max_size=8))
  #
  #   msg_strategy = st.binary(min_size=8, max_size=8)
  #   msgs = data.draw(st.lists(msg_strategy, min_size=100))#, min_size=100, max_size=1000))
  #   print(len(msgs))
  #
  #   start_gas = self.safety.get_gas_pressed_prev()
  #   start_gas_int_detected = self.safety.get_gas_interceptor_detected()
  #
  #   # for bus, address, dat in msgs:
  #   for dat in msgs:
  #     to_send = libpanda_py.make_CANPacket(address, bus, dat)
  #     did_rx = self.safety.safety_rx_hook(to_send)
  #
  #     can = messaging.new_message('can', 1)
  #     can.can = [log.CanData(address=address, dat=dat, src=bus)]
  #
  #     CC = car.CarControl.new_message()
  #     CS = self.CI.update(CC, (can.to_bytes(),))
  #
  #     if self.safety.get_gas_interceptor_detected():# and state_has_changed(start_gas, self.safety.get_gas_pressed_prev()):
  #       print('get_gas_interceptor_detected!')
  #       # self.assertEqual(CS.gasPressed, self.safety.get_gas_interceptor_prev())
  #       self.assertEqual(CS.gasPressed, self.safety.get_gas_pressed_prev())
  #       # self.assertFalse(True)
  #
  #
  #     # if self.safety.get_gas_pressed_prev() and self.safety.get_cruise_engaged_prev():
  #     #   self.assertFalse(True)
  #     # self.assertFalse(self.safety.get_cruise_engaged_prev())
  #
  #     # print('gas_pressed', CS.gasPressed, self.safety.get_gas_pressed_prev())
  #     # print('wheel_speeds', CS.wheelSpeeds)
  #     # print('standstill', CS.standstill, not self.safety.get_vehicle_moving())
  #
  #     # print('did_rx', did_rx)
  #     # if did_rx:
  #     #   self.assertFalse(True, 'finally did rx: {}, {}'.format(i, dat))
  #     # self.assertTrue(CS.standstill, (not CS.standstill, self.safety.get_vehicle_moving(), CS.vEgoRaw, CS.wheelSpeeds))
  #
  #
  #     # self.assertEqual(CS.gasPressed, self.safety.get_gas_pressed_prev())
  #     # self.assertEqual(not CS.standstill, self.safety.get_vehicle_moving())
  #     # self.assertEqual(CS.brakePressed, self.safety.get_brake_pressed_prev())
  #     # self.assertEqual(CS.regenBraking, self.safety.get_regen_braking_prev())
  #     #
  #     # if self.CP.pcmCruise:
  #     #   self.assertEqual(CS.cruiseState.enabled, self.safety.get_cruise_engaged_prev())
  #     #
  #     # if self.CP.carName == "honda":
  #     #   self.assertEqual(CS.cruiseState.available, self.safety.get_acc_main_on())
  #
  #
  #   # if self.safety.get_gas_interceptor_detected():
  #   #   print('get_gas_interceptor_detected!')
  #   #   # self.assertEqual(CS.gasPressed, self.safety.get_gas_interceptor_prev())
  #   #   self.assertEqual(CS.gasPressed, self.safety.get_gas_pressed_prev())
  #   #   # self.assertFalse(True)
  #
  #   print(self.safety.get_gas_pressed_prev(), self.safety.get_brake_pressed_prev(), self.safety.get_vehicle_moving(), self.safety.get_cruise_engaged_prev())
  #   assume(state_has_changed(False, self.safety.get_gas_pressed_prev()))
  #   # assume(state_has_changed(start_gas, self.safety.get_gas_pressed_prev()))  # this just goes on forever
  #   # assume(state_has_changed(start_gas_int_detected, self.safety.get_gas_interceptor_detected()))
  #   # assume(state_has_changed(False, self.safety.get_brake_pressed_prev()))
  #   # assume(state_has_changed(False, self.safety.get_vehicle_moving()))
  #   # assume(state_has_changed(False, self.safety.get_cruise_engaged_prev()))
  #
  #   # print(msgs)
  #   # print('\nresults', self.safety.get_gas_pressed_prev(), self.safety.get_vehicle_moving(), self.safety.get_brake_pressed_prev(), self.safety.get_regen_braking_prev(), self.safety.get_cruise_engaged_prev(), self.safety.get_acc_main_on())
  #   return
  #
  #   for i in range(1000):
  #     # self.setUp()
  #     dat = os.urandom(8)
  #     to_send = libpanda_py.make_CANPacket(address, bus, dat)
  #     did_rx = self.safety.safety_rx_hook(to_send)
  #
  #     can = messaging.new_message('can', 1)
  #     can.can = [log.CanData(address=address, dat=dat, src=bus)]
  #
  #     CC = car.CarControl.new_message()
  #     CS = self.CI.update(CC, (can.to_bytes(), ))
  #
  #     print('gas_pressed', CS.gasPressed, self.safety.get_gas_pressed_prev())
  #     print('wheel_speeds', CS.wheelSpeeds)
  #     print('standstill', CS.standstill, not self.safety.get_vehicle_moving())
  #
  #     print('did_rx', did_rx)
  #     # if did_rx:
  #     #   self.assertFalse(True, 'finally did rx: {}, {}'.format(i, dat))
  #     self.assertEqual(not CS.standstill, self.safety.get_vehicle_moving())
  #
  #   print('\nresults', self.safety.get_gas_pressed_prev(), self.safety.get_vehicle_moving(), self.safety.get_brake_pressed_prev(), self.safety.get_regen_braking_prev(), self.safety.get_cruise_engaged_prev(), self.safety.get_acc_main_on())
  #
  #   # self.assertEqual(CS.gasPressed, self.safety.get_gas_pressed_prev())
  #   # self.assertEqual(not CS.standstill, self.safety.get_vehicle_moving())
  #   # self.assertEqual(CS.brakePressed, self.safety.get_brake_pressed_prev())
  #   # self.assertEqual(CS.regenBraking, self.safety.get_regen_braking_prev())
  #   #
  #   # if self.CP.pcmCruise:
  #   #   self.assertEqual(CS.cruiseState.enabled, self.safety.get_cruise_engaged_prev())
  #   #
  #   # if self.CP.carName == "honda":
  #   #   self.assertEqual(CS.cruiseState.available, self.safety.get_acc_main_on())


# @parameterized_class(('car_model', 'test_route'), get_test_cases())
# class TestCarModel(TestCarModelBase):
#   pass

TestCarModelBase = CarModelBase.TestCase

# TestCarModelBase().runTest()


if __name__ == "__main__":
  unittest.main()

