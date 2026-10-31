"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from math import gcd

from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs, apply_hysteresis
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, update_manual_button_timers

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

ALLOWED_SPEED_THRESHOLD = 1.8  # m/s, ~4 MPH
HYST_GAP = 0.0  # currently disabled; TODO-SP: might need to be brand-specific
INACTIVE_TIMER = 0.4
STEP_HISTORY_SIZE = 4  # observed set speed values used to derive the increment
MAX_SET_SPEED_STEP = 10  # display units, sanity bound for a plausible increment


SEND_BUTTONS = {
  State.increasing: SendButtonState.increase,
  State.decreasing: SendButtonState.decrease,
}


class IntelligentCruiseButtonManagement:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.v_target = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.cruise_button = SendButtonState.none
    self.state = State.inactive
    self.pre_active_timer = 0

    self.is_ready = False
    self.is_ready_prev = False
    self.v_target_ms_last = 0.0
    self.is_metric = False

    # Some cars adjust the set speed in coarse steps (e.g. JDM Nissan Leaf uses 5 km/h).
    # Learn the step from observed changes so an unreachable target doesn't cause oscillation.
    self.v_cruise_step = 1
    self.v_cruise_cluster_history: list[int] = []

    self.cruise_button_timers = CRUISE_BUTTON_TIMER

  @property
  def step_deadband(self) -> int:
    # Set speed changes are quantized to v_cruise_step, so anything within half a step
    # is the closest reachable value and must be treated as aligned.
    return self.v_cruise_step // 2

  @property
  def v_target_above(self) -> bool:
    return self.v_target > self.v_cruise_cluster + self.step_deadband

  @property
  def v_target_below(self) -> bool:
    return self.v_target < self.v_cruise_cluster - self.step_deadband

  @property
  def v_cruise_equal(self) -> bool:
    return not self.v_target_above and not self.v_target_below

  def update_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    ms_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    self.v_target_ms_last = apply_hysteresis(LP_SP.vTarget, self.v_target_ms_last, HYST_GAP * ms_conv)

    self.v_target = round(self.v_target_ms_last * speed_conv)
    self.v_cruise_min = get_minimum_set_speed(self.is_metric)

    v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)
    self._update_set_speed_step(v_cruise_cluster)
    self.v_cruise_cluster = v_cruise_cluster

  def _update_set_speed_step(self, v_cruise_cluster: int) -> None:
    if v_cruise_cluster == self.v_cruise_cluster or self.v_cruise_cluster <= 0:
      return

    # Only attribute set speed changes to ICBM while it is the one requesting buttons.
    # Changes in the opposite direction are not ours, so discard what we've learned so far.
    delta = v_cruise_cluster - self.v_cruise_cluster
    commanded = (self.cruise_button == SendButtonState.increase and delta > 0) or \
                (self.cruise_button == SendButtonState.decrease and delta < 0)
    if not commanded:
      self.v_cruise_cluster_history.clear()
      return

    self.v_cruise_cluster_history.append(v_cruise_cluster)
    del self.v_cruise_cluster_history[:-STEP_HISTORY_SIZE]

    # Require a few observations so a single missed intermediate value can't fake a larger step.
    if len(self.v_cruise_cluster_history) < 3:
      return

    # The GCD of consecutive observed values is the smallest real step, even when the car
    # starts from an arbitrary set speed (the initial SET is not aligned to the step grid).
    step = 0
    for prev, cur in zip(self.v_cruise_cluster_history, self.v_cruise_cluster_history[1:]):
      step = gcd(step, abs(cur - prev))

    if 1 < step <= MAX_SET_SPEED_STEP:
      self.v_cruise_step = step

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

    # HOLDING, ACCELERATING, DECELERATING, PRE_ACTIVE
    if self.state != State.inactive:
      if not self.is_ready:
        self.state = State.inactive

      else:
        # PRE_ACTIVE
        if self.state == State.preActive:
          if self.pre_active_timer <= 0:
            if self.v_cruise_equal:
              self.state = State.holding

            elif self.v_target_above:
              self.state = State.increasing

            elif self.v_target_below and self.v_cruise_cluster > self.v_cruise_min:
              self.state = State.decreasing

        # HOLDING
        elif self.state == State.holding:
          if not self.v_cruise_equal:
            self.state = State.preActive

        # ACCELERATING
        elif self.state == State.increasing:
          if not self.v_target_above:
            self.state = State.holding

        # DECELERATING
        elif self.state == State.decreasing:
          if not self.v_target_below or self.v_cruise_cluster <= self.v_cruise_min:
            self.state = State.holding

    # INACTIVE
    elif self.state == State.inactive:
      if self.is_ready and not self.is_ready_prev:
        self.pre_active_timer = int(INACTIVE_TIMER / DT_CTRL)
        self.state = State.preActive

    send_button = SEND_BUTTONS.get(self.state, SendButtonState.none)

    return send_button

  def update_readiness(self, CS: car.CarState, CC: car.CarControl) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)

    ready = CC.enabled and not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume
    button_pressed = any(self.cruise_button_timers[k] > 0 for k in self.cruise_button_timers)

    self.is_ready = ready and not button_pressed

  def run(self, CS: car.CarState, CC: car.CarControl, LP_SP: custom.LongitudinalPlanSP, is_metric: bool) -> None:
    if self.CP_SP.pcmCruiseSpeed:
      return

    self.is_metric = is_metric

    self.update_calculations(CS, LP_SP)
    self.update_readiness(CS, CC)

    self.cruise_button = self.update_state_machine()

    self.is_ready_prev = self.is_ready
