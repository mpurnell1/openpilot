#!/usr/bin/env python3
"""
EPS fault characterization maneuvers for Subaru STEER_RATE_LIMITED cars.

Applies controlled, sustained lateral loads to map the EPS thermal protection:
  B) Cold peak load (3.0, 4.0 m/s^2 for 5s) to test instantaneous protection
  A) Sustained escalating load (1.5 -> 2.0 -> 2.5 -> 3.0 -> 3.5 m/s^2, 30s each)
  C) Progressive ramp (0 -> 3.0 m/s^2 over 30s) to find threshold

REQUIRES: Large empty parking lot. The car WILL turn at tight radii (20-50m).
Run order matters: B runs first (needs cold EPS), then A escalates, then C.

Approximate steering angles at 20 mph:
  1.5 m/s^2 -> ~45 deg,  2.0 -> ~60 deg,  2.5 -> ~75 deg
  3.0 m/s^2 -> ~88 deg,  3.5 -> ~103 deg, 4.0 -> ~118 deg

Deploy:
  On the comma device, copy this file over the stock lateral_maneuversd.py:
    cp fault_test_maneuversd.py lateral_maneuversd.py
  Then enable "Lateral Maneuver Mode" in Settings > Developer while offroad.
  Set cruise to 20 mph on a straight, flat surface. Maneuvers auto-start.
  To restore stock maneuvers afterward: git checkout lateral_maneuversd.py
"""

import numpy as np
from dataclasses import dataclass

from cereal import messaging, car
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.drive_helpers import MIN_SPEED
from openpilot.tools.longitudinal_maneuvers.maneuversd import Action, Maneuver as _Maneuver

MAX_SPEED_DEV = 0.7
MAX_CURV = 0.002
MAX_ROLL = 0.08
TIMER = 2.0


@dataclass
class Maneuver(_Maneuver):
  _baseline_curvature: float = 0.0

  def get_accel(self, v_ego: float, lat_active: bool, curvature: float, roll: float) -> float:
    self._run_completed = False
    ready = abs(v_ego - self.initial_speed) < MAX_SPEED_DEV and lat_active and abs(curvature) < MAX_CURV and abs(roll) < MAX_ROLL
    self._ready_cnt = (self._ready_cnt + 1) if ready else max(self._ready_cnt - 1, 0)

    if self._ready_cnt > (TIMER / DT_MDL):
      if not self._active:
        self._baseline_curvature = curvature
      self._active = True

    if not self._active:
      return 0.0

    return self._step()

  def reset(self):
    super().reset()
    self._ready_cnt = 0


def _ramp_action(start_accel, end_accel, duration):
  """Linear ramp from start_accel to end_accel over duration seconds."""
  n = int(duration / DT_MDL) + 1
  a = np.linspace(start_accel, end_accel, n)
  t = np.linspace(0, duration, n)
  return Action(a.tolist(), t.tolist())


MANEUVERS = [
  # === SECTION B: Cold peak loads (run FIRST while EPS is cold) ===
  # Tests whether a single aggressive maneuver can fault from cold
  Maneuver(
    "B1: cold peak 3.0 m/s^2 5s",
    [Action([3.0], [5.0]), Action([0.0], [2.0])],
    repeat=0,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "B1: recovery 60s",
    [Action([0.0], [60.0])],
    repeat=0,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "B2: cold peak 4.0 m/s^2 5s",
    [Action([4.0], [5.0]), Action([0.0], [2.0])],
    repeat=0,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "B2: recovery 60s",
    [Action([0.0], [60.0])],
    repeat=0,
    initial_speed=20. * CV.MPH_TO_MS,
  ),

  # === SECTION A: Sustained load escalation ===
  # Each step: hold for 30s, then 60s recovery
  # Escalates until fault occurs. Faults on route 00000026 happened at 90-200 deg
  # steering angle, so we need at least 2.5-3.5 m/s^2 to reach those angles
  Maneuver(
    "A1: sustained 1.5 m/s^2 30s (~45 deg)",
    [Action([1.5], [30.0]), Action([0.0], [2.0])],
    repeat=0,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "A1: recovery 60s",
    [Action([0.0], [60.0])],
    repeat=0,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "A2: sustained 2.0 m/s^2 30s (~60 deg)",
    [Action([2.0], [30.0]), Action([0.0], [2.0])],
    repeat=0,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "A2: recovery 60s",
    [Action([0.0], [60.0])],
    repeat=0,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "A3: sustained 2.5 m/s^2 30s (~75 deg)",
    [Action([2.5], [30.0]), Action([0.0], [2.0])],
    repeat=0,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "A3: recovery 60s",
    [Action([0.0], [60.0])],
    repeat=0,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "A4: sustained 3.0 m/s^2 30s (~88 deg)",
    [Action([3.0], [30.0]), Action([0.0], [2.0])],
    repeat=0,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "A4: recovery 60s",
    [Action([0.0], [60.0])],
    repeat=0,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "A5: sustained 3.5 m/s^2 30s (~103 deg)",
    [Action([3.5], [30.0]), Action([0.0], [2.0])],
    repeat=0,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "A5: recovery 60s",
    [Action([0.0], [60.0])],
    repeat=0,
    initial_speed=20. * CV.MPH_TO_MS,
  ),

  # === SECTION C: Progressive ramp ===
  # Smooth ramp to find the transition point
  Maneuver(
    "C: ramp 0->3.0 m/s^2 30s + hold 15s",
    [_ramp_action(0.0, 3.0, 30.0), Action([3.0], [15.0]), Action([0.0], [2.0])],
    repeat=0,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
]


def main():
  params = Params()
  cloudlog.info("fault_test_maneuversd is waiting for CarParams")
  messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)

  sm = messaging.SubMaster(['carState', 'carControl', 'controlsState', 'selfdriveState', 'modelV2'], poll='modelV2')
  pm = messaging.PubMaster(['lateralManeuverPlan', 'alertDebug'])

  maneuvers = iter(MANEUVERS)
  maneuver = None
  complete_cnt = 0
  display_holdoff = 0
  prev_text = ''

  while True:
    sm.update()

    if maneuver is None:
      maneuver = next(maneuvers, None)

    alert_msg = messaging.new_message('alertDebug')
    alert_msg.valid = True

    plan_send = messaging.new_message('lateralManeuverPlan')

    accel = 0
    v_ego = max(sm['carState'].vEgo, 0)
    curvature = sm['controlsState'].desiredCurvature

    if complete_cnt > 0:
      complete_cnt -= 1
      alert_msg.alertDebug.alertText1 = 'Completed'
      alert_msg.alertDebug.alertText2 = maneuver.description
    elif maneuver is not None:
      if sm['carState'].steeringPressed or (maneuver.active and abs(v_ego - maneuver.initial_speed) > MAX_SPEED_DEV):
        maneuver.reset()

      roll = sm['carControl'].orientationNED[0] if len(sm['carControl'].orientationNED) == 3 else 0.0
      accel = maneuver.get_accel(v_ego, sm['carControl'].latActive, curvature, roll)

      if maneuver._run_completed:
        complete_cnt = int(1.0 / DT_MDL)
        alert_msg.alertDebug.alertText1 = 'Complete'
        alert_msg.alertDebug.alertText2 = maneuver.description
      elif maneuver.active:
        action_remaining = maneuver.actions[maneuver._action_index].time_bp[-1] - maneuver._action_frames * DT_MDL
        alert_msg.alertDebug.alertText1 = f'Active {accel:+.1f}m/s^2 {max(action_remaining, 0):.1f}s'
        alert_msg.alertDebug.alertText2 = maneuver.description
      elif not (abs(v_ego - maneuver.initial_speed) < MAX_SPEED_DEV and sm['carControl'].latActive):
        alert_msg.alertDebug.alertText1 = f'Set speed to {maneuver.initial_speed * CV.MS_TO_MPH:0.0f} mph'
      elif maneuver._ready_cnt > 0:
        ready_time = max(TIMER - maneuver._ready_cnt * DT_MDL, 0)
        alert_msg.alertDebug.alertText1 = f'Starting: {int(ready_time) + 1}'
        alert_msg.alertDebug.alertText2 = maneuver.description
      else:
        curv_ok = abs(curvature) < MAX_CURV
        reason = 'road not straight' if not curv_ok else 'road not flat'
        alert_msg.alertDebug.alertText1 = f'Waiting: {reason}'
        alert_msg.alertDebug.alertText2 = maneuver.description
    else:
      alert_msg.alertDebug.alertText1 = 'Maneuvers Finished'

    setup = ('Set speed', 'Starting', 'Waiting')
    text = alert_msg.alertDebug.alertText1
    same = text == prev_text or (text.startswith('Starting') and prev_text.startswith('Starting'))
    if not same and text.startswith(setup) and prev_text.startswith(setup) and display_holdoff > 0:
      alert_msg.alertDebug.alertText1 = prev_text
      display_holdoff -= 1
    else:
      prev_text = text
      display_holdoff = int(0.5 / DT_MDL) if text.startswith(setup) else 0

    pm.send('alertDebug', alert_msg)

    plan_send.valid = maneuver is not None and maneuver.active and complete_cnt == 0
    if plan_send.valid:
      plan_send.lateralManeuverPlan.desiredCurvature = maneuver._baseline_curvature + accel / max(v_ego, MIN_SPEED) ** 2
    pm.send('lateralManeuverPlan', plan_send)

    if maneuver is not None and maneuver.finished and complete_cnt == 0:
      maneuver = None


if __name__ == "__main__":
  main()
