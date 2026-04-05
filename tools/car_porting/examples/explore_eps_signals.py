#!/usr/bin/env python3
"""
Explore unused EPS CAN signals around steering fault events.

Parses message 0x118 (Steering_Torque_2) and 0x119 (Steering_Torque) from raw
CAN data in existing routes, focusing on signals that openpilot currently ignores:

  0x118: Signal1 (8-bit unknown), Steering_Active, Steering_Disabled
  0x119: Steer_Error_2

Shows each signal's behavior before, during, and after Steer_Warning events to
determine whether any of them reveal the EPS's internal state.

Usage:
  python tools/car_porting/examples/explore_eps_signals.py "dongle/route_id/segments"
"""

import copy
import sys
import numpy as np

from opendbc.can.parser import CANParser
from opendbc.car.subaru.values import CanBus, DBC

from openpilot.selfdrive.pandad import can_capnp_to_list
from openpilot.tools.lib.logreader import LogReader


PLATFORM = "SUBARU_FORESTER"
CAN_HZ = 50
DT = 1.0 / CAN_HZ

LOOKBACK_S = 10.0
LOOKFORWARD_S = 5.0
LOOKBACK = int(LOOKBACK_S / DT)
LOOKFORWARD = int(LOOKFORWARD_S / DT)


def analyze_route(route_identifier):
  print(f"\nLoading route: {route_identifier}")
  lr = LogReader(route_identifier)

  can_msgs = [msg for msg in lr if msg.which() == "can"]
  print(f"Loaded {len(can_msgs)} CAN frames")

  messages = [
    ("Steering_Torque", CAN_HZ),
    ("Steering_Torque_2", CAN_HZ),
  ]
  cp = CANParser(DBC[PLATFORM]["pt"], messages, CanBus.main)

  history = []
  for msg in can_msgs:
    cp.update(can_capnp_to_list([msg.as_builder().to_bytes()]))
    entry = {
      "Signal1": cp.vl["Steering_Torque_2"]["Signal1"],
      "Steering_Active": cp.vl["Steering_Torque_2"]["Steering_Active"],
      "Steering_Disabled": cp.vl["Steering_Torque_2"]["Steering_Disabled"],
      "Steer_Torque_Output_2": cp.vl["Steering_Torque_2"]["Steer_Torque_Output"],
      "Steer_Torque_Sensor_2": cp.vl["Steering_Torque_2"]["Steer_Torque_Sensor"],
      "Steer_Torque_Output": cp.vl["Steering_Torque"]["Steer_Torque_Output"],
      "Steer_Torque_Sensor": cp.vl["Steering_Torque"]["Steer_Torque_Sensor"],
      "Steering_Angle": cp.vl["Steering_Torque"]["Steering_Angle"],
      "Steer_Warning": cp.vl["Steering_Torque"]["Steer_Warning"],
      "Steer_Error_1": cp.vl["Steering_Torque"]["Steer_Error_1"],
      "Steer_Error_2": cp.vl["Steering_Torque"]["Steer_Error_2"],
    }
    history.append(entry)

  print(f"Parsed {len(history)} frames")

  if not history:
    print("No data!")
    return

  # Find fault events (Steer_Warning positive transitions)
  fault_events = []
  for i in range(1, len(history)):
    if history[i]["Steer_Warning"] == 1 and history[i-1]["Steer_Warning"] == 0:
      fault_events.append(i)

  print(f"Found {len(fault_events)} fault events")

  # Signal1 global statistics
  s1_values = [h["Signal1"] for h in history]
  print(f"\n{'='*100}")
  print("SIGNAL1 GLOBAL STATISTICS (full route)")
  print(f"{'='*100}")
  print(f"  Min:    {min(s1_values)}")
  print(f"  Max:    {max(s1_values)}")
  print(f"  Mean:   {np.mean(s1_values):.1f}")
  print(f"  Median: {np.median(s1_values):.0f}")
  print(f"  Std:    {np.std(s1_values):.1f}")
  unique = sorted(set(int(v) for v in s1_values))
  if len(unique) <= 20:
    print(f"  Unique values: {unique}")
  else:
    print(f"  Unique values: {len(unique)} distinct ({unique[:10]} ... {unique[-5:]})")

  # Steering_Active / Steering_Disabled global stats
  sa_values = [h["Steering_Active"] for h in history]
  sd_values = [h["Steering_Disabled"] for h in history]
  se2_values = [h["Steer_Error_2"] for h in history]
  print(f"\n  Steering_Active:   {sum(1 for v in sa_values if v == 1)}/{len(sa_values)} frames high ({100*sum(1 for v in sa_values if v == 1)/len(sa_values):.1f}%)")
  print(f"  Steering_Disabled: {sum(1 for v in sd_values if v == 1)}/{len(sd_values)} frames high ({100*sum(1 for v in sd_values if v == 1)/len(sd_values):.1f}%)")
  print(f"  Steer_Error_2:     {sum(1 for v in se2_values if v == 1)}/{len(se2_values)} frames high ({100*sum(1 for v in se2_values if v == 1)/len(se2_values):.1f}%)")

  # Correlation: Signal1 vs |Steer_Torque_Output|
  outputs = [abs(h["Steer_Torque_Output"]) for h in history]
  if np.std(s1_values) > 0 and np.std(outputs) > 0:
    corr = np.corrcoef(s1_values, outputs)[0, 1]
    print(f"\n  Correlation(Signal1, |Torque_Output|): {corr:.4f}")
  else:
    print(f"\n  Correlation: N/A (zero variance in one signal)")

  # Compare 0x118 vs 0x119 torque signals
  out_119 = [h["Steer_Torque_Output"] for h in history]
  out_118 = [h["Steer_Torque_Output_2"] for h in history]
  diff = [abs(a - b) for a, b in zip(out_119, out_118)]
  print(f"\n  0x118 vs 0x119 Steer_Torque_Output: max_diff={max(diff):.0f}, mean_diff={np.mean(diff):.1f}")

  sen_119 = [h["Steer_Torque_Sensor"] for h in history]
  sen_118 = [h["Steer_Torque_Sensor_2"] for h in history]
  diff_s = [abs(a - b) for a, b in zip(sen_119, sen_118)]
  print(f"  0x118 vs 0x119 Steer_Torque_Sensor: max_diff={max(diff_s):.0f}, mean_diff={np.mean(diff_s):.1f}")

  if not fault_events:
    print("\nNo faults in this route. Showing Signal1 distribution by torque output level:")
    _show_signal1_by_load(history)
    return

  # Per-fault analysis
  print(f"\n{'='*100}")
  print("FAULT EVENT ANALYSIS")
  print(f"{'='*100}")

  for fi, fault_idx in enumerate(fault_events):
    time_s = fault_idx * DT
    print(f"\n--- Fault #{fi+1} at frame {fault_idx} (~{time_s:.1f}s) ---")
    h = history[fault_idx]
    print(f"  Torque Output:  {h['Steer_Torque_Output']}")
    print(f"  Torque Sensor:  {h['Steer_Torque_Sensor']}")
    print(f"  Steering Angle: {h['Steering_Angle']:.1f} deg")

    # Signal1 around fault
    start = max(0, fault_idx - LOOKBACK)
    end = min(len(history), fault_idx + LOOKFORWARD)

    s1_before = [history[j]["Signal1"] for j in range(start, fault_idx)]
    s1_at = history[fault_idx]["Signal1"]
    s1_after = [history[j]["Signal1"] for j in range(fault_idx + 1, end)]

    print(f"\n  Signal1:")
    print(f"    At fault:        {s1_at}")
    if s1_before:
      print(f"    {LOOKBACK_S:.0f}s before: min={min(s1_before)}, max={max(s1_before)}, mean={np.mean(s1_before):.1f}")
    if s1_after:
      print(f"    {LOOKFORWARD_S:.0f}s after:  min={min(s1_after)}, max={max(s1_after)}, mean={np.mean(s1_after):.1f}")

    # Check if Signal1 was CHANGING leading up to fault
    if len(s1_before) >= 50:
      first_quarter = s1_before[:len(s1_before)//4]
      last_quarter = s1_before[-len(s1_before)//4:]
      print(f"    Trend: first quarter avg={np.mean(first_quarter):.1f}, last quarter avg={np.mean(last_quarter):.1f}")

    # Steering_Active / Steering_Disabled around fault
    sa_before = [history[j]["Steering_Active"] for j in range(max(0, fault_idx - 25), fault_idx)]
    sa_at = history[fault_idx]["Steering_Active"]
    sa_after = [history[j]["Steering_Active"] for j in range(fault_idx + 1, min(len(history), fault_idx + 25))]

    sd_before = [history[j]["Steering_Disabled"] for j in range(max(0, fault_idx - 25), fault_idx)]
    sd_at = history[fault_idx]["Steering_Disabled"]
    sd_after = [history[j]["Steering_Disabled"] for j in range(fault_idx + 1, min(len(history), fault_idx + 25))]

    # Find transitions
    sa_dropped = any(sa_before[i] == 1 and sa_before[i+1] == 0 for i in range(len(sa_before)-1))
    sd_rose = any(sd_before[i] == 0 and sd_before[i+1] == 1 for i in range(len(sd_before)-1))

    print(f"\n  Steering_Active:   before={''.join(str(int(v)) for v in sa_before[-10:])} | at={int(sa_at)} | after={''.join(str(int(v)) for v in sa_after[:10])}")
    print(f"  Steering_Disabled: before={''.join(str(int(v)) for v in sd_before[-10:])} | at={int(sd_at)} | after={''.join(str(int(v)) for v in sd_after[:10])}")
    if sa_dropped:
      print(f"    ** Steering_Active DROPPED in the 0.5s before fault!")
    if sd_rose:
      print(f"    ** Steering_Disabled ROSE in the 0.5s before fault!")

    # Steer_Error_2
    se2_before = [history[j]["Steer_Error_2"] for j in range(max(0, fault_idx - 25), fault_idx)]
    se2_at = history[fault_idx]["Steer_Error_2"]
    se2_after = [history[j]["Steer_Error_2"] for j in range(fault_idx + 1, min(len(history), fault_idx + 25))]
    print(f"  Steer_Error_2:     before={''.join(str(int(v)) for v in se2_before[-10:])} | at={int(se2_at)} | after={''.join(str(int(v)) for v in se2_after[:10])}")

    # Detailed timeline around fault (2s before, 1s after)
    print(f"\n  Detailed timeline (2s before -> 1s after):")
    print(f"  {'Frame':>6} {'t':>6} {'Sig1':>5} {'|Out|':>6} {'|Drv|':>5} {'Angle':>7} {'SA':>3} {'SD':>3} {'SE2':>4} {'SW':>3}")
    detail_start = max(0, fault_idx - 100)
    detail_end = min(len(history), fault_idx + 50)
    step = max(1, (detail_end - detail_start) // 40)  # ~40 rows max
    for j in range(detail_start, detail_end, step):
      hj = history[j]
      marker = " <-- FAULT" if j == fault_idx else ""
      t = (j - fault_idx) * DT
      print(f"  {j:6d} {t:+6.2f} {hj['Signal1']:5.0f} {abs(hj['Steer_Torque_Output']):6.0f} "
            f"{abs(hj['Steer_Torque_Sensor']):5.0f} {hj['Steering_Angle']:7.1f} "
            f"{int(hj['Steering_Active']):3d} {int(hj['Steering_Disabled']):3d} "
            f"{int(hj['Steer_Error_2']):4d} {int(hj['Steer_Warning']):3d}{marker}")

  # Signal1 distribution by load level
  print(f"\n{'='*100}")
  print("SIGNAL1 BY LOAD LEVEL")
  print(f"{'='*100}")
  _show_signal1_by_load(history)


def _show_signal1_by_load(history):
  brackets = [(0, 200), (200, 500), (500, 800), (800, 1200), (1200, 2000)]
  for lo, hi in brackets:
    frames = [h for h in history if lo <= abs(h["Steer_Torque_Output"]) < hi]
    if frames:
      s1 = [h["Signal1"] for h in frames]
      print(f"  |Output| {lo:>4}-{hi:<4}: {len(frames):6d} frames, "
            f"Signal1 min={min(s1):.0f} max={max(s1):.0f} mean={np.mean(s1):.1f} std={np.std(s1):.1f}")


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print("Usage: python explore_eps_signals.py <route1> [route2] ...")
    print("Example: python explore_eps_signals.py 'fca4fccad2e7b029/00000009--a662df27d9/0:9'")
    sys.exit(1)

  for route in sys.argv[1:]:
    try:
      analyze_route(route)
    except Exception as e:
      print(f"\nERROR on {route}: {e}")
      import traceback
      traceback.print_exc()
