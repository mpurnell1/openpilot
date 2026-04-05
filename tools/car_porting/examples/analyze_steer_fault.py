#!/usr/bin/env python3
"""
Analyze Subaru steering temp fault events to reverse-engineer EPS fault conditions.

Parses CAN data from a route, finds Steer_Warning transitions, and calculates
cumulative steering angle change over various sliding windows to determine what
the EPS is actually tracking.

Usage:
  python tools/car_porting/examples/analyze_steer_fault.py "fca4fccad2e7b029/00000026--232120d429/10:12"
"""

import copy
import sys
import numpy as np

from opendbc.can.parser import CANParser
from opendbc.car.subaru.values import CanBus, DBC

from openpilot.selfdrive.pandad import can_capnp_to_list
from openpilot.tools.lib.logreader import LogReader


PLATFORM = "SUBARU_FORESTER"
CAN_MSG_HZ = 50  # Steering_Torque message frequency on the bus


def analyze_route(route_identifier):
  print(f"Loading route: {route_identifier}")
  lr = LogReader(route_identifier)

  can_msgs = [msg for msg in lr if msg.which() == "can"]
  print(f"Loaded {len(can_msgs)} CAN frames")

  messages = [("Steering_Torque", CAN_MSG_HZ)]
  cp = CANParser(DBC[PLATFORM]["pt"], messages, CanBus.main)

  # Collect full history
  history = []
  for msg in can_msgs:
    cp.update(can_capnp_to_list([msg.as_builder().to_bytes()]))
    entry = copy.copy(cp.vl["Steering_Torque"])
    entry["_all_angles"] = list(cp.vl_all["Steering_Torque"]["Steering_Angle"])
    history.append(entry)

  print(f"Parsed {len(history)} steering torque messages")

  # Calculate steering rate (matching CanSignalRateCalculator logic)
  angles = [h["Steering_Angle"] for h in history]
  rates = [0.0]
  prev_angle = angles[0]
  for i in range(1, len(angles)):
    updated = len(history[i]["_all_angles"]) > 0
    if updated:
      rate = (angles[i] - prev_angle) * CAN_MSG_HZ
    else:
      rate = rates[-1]  # hold previous rate
    prev_angle = angles[i]
    rates.append(rate)

  # Also compute what the rate would be with frequency=25 (correct for 25Hz signal)
  rates_corrected = [0.0]
  prev_angle_c = angles[0]
  for i in range(1, len(angles)):
    updated = len(history[i]["_all_angles"]) > 0
    if updated:
      rate = (angles[i] - prev_angle_c) * 25  # actual 25Hz signal rate
    else:
      rate = rates_corrected[-1]
    prev_angle_c = angles[i]
    rates_corrected.append(rate)

  # Find Steer_Warning positive transitions (fault events)
  fault_events = []
  steer_warning_last = False
  for i, h in enumerate(history):
    steer_warning = h["Steer_Warning"] == 1
    if steer_warning and not steer_warning_last:
      fault_events.append(i)
    steer_warning_last = steer_warning

  print(f"\nFound {len(fault_events)} fault events (Steer_Warning transitions)")

  if not fault_events:
    print("No faults found in this route segment. Try a different segment range.")
    return

  # For each fault event, analyze the preceding conditions
  dt = 1.0 / CAN_MSG_HZ  # time per frame

  # Sliding window sizes to test (in seconds)
  window_sizes = [0.1, 0.2, 0.3, 0.4, 0.5, 0.75, 1.0, 1.5, 2.0]

  print("\n" + "=" * 100)
  print("FAULT EVENT ANALYSIS")
  print("=" * 100)

  cumulative_at_fault = {ws: [] for ws in window_sizes}

  for event_idx, fault_frame in enumerate(fault_events):
    time_s = fault_frame * dt
    print(f"\n--- Fault Event #{event_idx + 1} at frame {fault_frame} (~{time_s:.1f}s) ---")
    print(f"  Steering Angle at fault: {angles[fault_frame]:.1f} deg")
    print(f"  Steering Rate (freq=50): {rates[fault_frame]:.1f} deg/s")
    print(f"  Steering Rate (freq=25): {rates_corrected[fault_frame]:.1f} deg/s")
    print(f"  Steer Torque Output:     {history[fault_frame]['Steer_Torque_Output']}")
    print(f"  Steer Torque Sensor:     {history[fault_frame]['Steer_Torque_Sensor']}")

    # Calculate cumulative absolute angle change over various windows BEFORE the fault
    print(f"\n  Cumulative |angle change| in windows ending at fault:")
    for ws in window_sizes:
      window_frames = int(ws / dt)
      start = max(0, fault_frame - window_frames)
      cumulative_change = sum(abs(angles[j] - angles[j - 1]) for j in range(start + 1, fault_frame + 1))
      cumulative_at_fault[ws].append(cumulative_change)
      print(f"    {ws:5.2f}s ({window_frames:4d} frames): {cumulative_change:7.2f} deg total change")

    # Show the angle trajectory leading up to the fault
    lookback = min(50, fault_frame)  # 50 frames = 1s at 50Hz
    print(f"\n  Angle trajectory (last {lookback} frames before fault):")
    print(f"  {'Frame':>6} {'Angle':>8} {'Rate(50)':>9} {'Rate(25)':>9} {'|Rate(50)|':>10} {'Warning':>8}")
    for j in range(fault_frame - lookback, fault_frame + 5):
      if 0 <= j < len(history):
        marker = " <-- FAULT" if j == fault_frame else ""
        warning = history[j]["Steer_Warning"]
        print(f"  {j:6d} {angles[j]:8.1f} {rates[j]:9.1f} {rates_corrected[j]:9.1f} {abs(rates[j]):10.1f} {warning:8}{marker}")

  # Summary statistics
  print("\n" + "=" * 100)
  print("SUMMARY: Cumulative angle change at fault onset")
  print("=" * 100)
  print(f"{'Window':>8} {'Min':>8} {'Max':>8} {'Mean':>8} {'Std':>8}")
  for ws in window_sizes:
    values = cumulative_at_fault[ws]
    if values:
      print(f"  {ws:5.2f}s {min(values):8.2f} {max(values):8.2f} {np.mean(values):8.2f} {np.std(values):8.2f}")

  # Analyze: what percentage of time was the rate above various thresholds before each fault?
  print("\n" + "=" * 100)
  print("RATE ANALYSIS: Frames above threshold before each fault (1s window)")
  print("=" * 100)
  thresholds = [10, 15, 20, 25, 30, 35, 40, 50]
  window_frames = int(1.0 / dt)
  for event_idx, fault_frame in enumerate(fault_events):
    print(f"\n  Fault #{event_idx + 1}:")
    start = max(0, fault_frame - window_frames)
    for thresh in thresholds:
      frames_above = sum(1 for j in range(start, fault_frame + 1) if abs(rates[j]) > thresh)
      frames_above_corrected = sum(1 for j in range(start, fault_frame + 1) if abs(rates_corrected[j]) > thresh)
      pct = 100 * frames_above / (fault_frame - start + 1)
      pct_c = 100 * frames_above_corrected / (fault_frame - start + 1)
      print(f"    >{thresh:3d} deg/s: {frames_above:3d}/{fault_frame - start + 1} frames ({pct:5.1f}%) [corrected: {frames_above_corrected:3d} ({pct_c:5.1f}%)]")

  # Analyze sustained torque before each fault
  print("\n" + "=" * 100)
  print("TORQUE ANALYSIS: Sustained torque output before each fault")
  print("=" * 100)
  torque_thresholds = [500, 750, 1000, 1250, 1500]
  for event_idx, fault_frame in enumerate(fault_events):
    torques = [abs(history[j]["Steer_Torque_Output"]) for j in range(max(0, fault_frame - 200), fault_frame + 1)]
    driver_torques = [abs(history[j]["Steer_Torque_Sensor"]) for j in range(max(0, fault_frame - 200), fault_frame + 1)]
    print(f"\n  Fault #{event_idx + 1} (angle={angles[fault_frame]:.1f}, output={history[fault_frame]['Steer_Torque_Output']}, driver={history[fault_frame]['Steer_Torque_Sensor']}):")

    # Find how many consecutive frames of high torque before the fault
    for thresh in torque_thresholds:
      consecutive = 0
      for j in range(fault_frame, max(0, fault_frame - 200), -1):
        if abs(history[j]["Steer_Torque_Output"]) >= thresh:
          consecutive += 1
        else:
          break
      duration_s = consecutive * dt
      print(f"    |Torque Output| >= {thresh:5d}: {consecutive:4d} consecutive frames ({duration_s:.2f}s)")

    # Also show cumulative torque*time (torque-seconds)
    for ws in [0.5, 1.0, 1.5, 2.0, 3.0, 4.0]:
      window_frames = int(ws / dt)
      start = max(0, fault_frame - window_frames)
      torque_integral = sum(abs(history[j]["Steer_Torque_Output"]) * dt for j in range(start, fault_frame + 1))
      torque_mismatch_integral = sum(
        max(0, abs(history[j]["Steer_Torque_Output"]) - abs(history[j]["Steer_Torque_Sensor"])) * dt
        for j in range(start, fault_frame + 1)
      )
      print(f"    {ws:4.1f}s window: torque integral={torque_integral:8.1f}, mismatch integral={torque_mismatch_integral:8.1f}")

  # Check if there's a consistent pattern in the cumulative angle change
  print("\n" + "=" * 100)
  print("HYPOTHESIS TEST: Which window size has the most consistent cumulative angle threshold?")
  print("=" * 100)
  for ws in window_sizes:
    values = cumulative_at_fault[ws]
    if len(values) >= 2:
      cv = np.std(values) / np.mean(values) if np.mean(values) > 0 else float('inf')
      print(f"  {ws:5.2f}s window: mean={np.mean(values):7.2f} deg, CV={cv:.3f} (lower = more consistent)")
    elif len(values) == 1:
      print(f"  {ws:5.2f}s window: single value = {values[0]:.2f} deg")

  # Thermal model analysis: simulate EPS motor temperature with various time constants
  # Heat input proportional to torque^2 (I^2*R), cooling proportional to temperature
  print("\n" + "=" * 100)
  print("THERMAL MODEL ANALYSIS")
  print("=" * 100)

  # Test different time constants and input models
  time_constants = [5, 10, 15, 20, 30, 45, 60, 90, 120]
  input_models = {
    "|torque|":       lambda h: abs(h["Steer_Torque_Output"]),
    "torque^2":       lambda h: h["Steer_Torque_Output"] ** 2,
    "|mismatch|":     lambda h: max(0, abs(h["Steer_Torque_Output"]) - abs(h["Steer_Torque_Sensor"])),
    "mismatch^2":     lambda h: max(0, abs(h["Steer_Torque_Output"]) - abs(h["Steer_Torque_Sensor"])) ** 2,
  }

  best_cv = float('inf')
  best_config = ""

  for model_name, heat_fn in input_models.items():
    for tau in time_constants:
      # Exponential moving average: T += (heat - T) * dt / tau
      alpha = dt / tau
      temp = 0.0
      temps_at_fault = []
      fault_idx = 0

      for i, h in enumerate(history):
        heat = heat_fn(h)
        temp += (heat - temp) * alpha

        if fault_idx < len(fault_events) and i == fault_events[fault_idx]:
          temps_at_fault.append(temp)
          fault_idx += 1

      if len(temps_at_fault) >= 2:
        mean_t = np.mean(temps_at_fault)
        cv = np.std(temps_at_fault) / mean_t if mean_t > 0 else float('inf')
        if cv < best_cv:
          best_cv = cv
          best_config = f"{model_name}, tau={tau}s"

  # Print results for all configs, sorted by CV
  results = []
  for model_name, heat_fn in input_models.items():
    for tau in time_constants:
      alpha = dt / tau
      temp = 0.0
      temps_at_fault = []
      fault_idx = 0

      for i, h in enumerate(history):
        heat = heat_fn(h)
        temp += (heat - temp) * alpha

        if fault_idx < len(fault_events) and i == fault_events[fault_idx]:
          temps_at_fault.append(temp)
          fault_idx += 1

      if len(temps_at_fault) >= 2:
        mean_t = np.mean(temps_at_fault)
        std_t = np.std(temps_at_fault)
        cv = std_t / mean_t if mean_t > 0 else float('inf')
        results.append((cv, model_name, tau, mean_t, std_t, temps_at_fault))

  results.sort(key=lambda x: x[0])

  print(f"\n  Top 15 most consistent thermal models (lowest CV = most consistent):")
  print(f"  {'CV':>6} {'Model':>14} {'Tau':>5} {'Mean':>12} {'Std':>12}   Fault values")
  for cv, model_name, tau, mean_t, std_t, temps in results[:15]:
    fault_str = ", ".join(f"{t:.1f}" for t in temps)
    print(f"  {cv:6.3f} {model_name:>14} {tau:4d}s {mean_t:12.1f} {std_t:12.1f}   [{fault_str}]")

  print(f"\n  Best config: {best_config} (CV={best_cv:.4f})")


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print("Usage: python analyze_steer_fault.py <route_identifier>")
    print("Example: python analyze_steer_fault.py 'fca4fccad2e7b029/00000026--232120d429/10:12'")
    sys.exit(1)

  analyze_route(sys.argv[1])
