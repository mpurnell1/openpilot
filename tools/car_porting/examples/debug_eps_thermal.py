#!/usr/bin/env python3
"""
Debug EPS thermal model from recorded routes.

Shows the EPS thermal EMA timeline, identifies what driving conditions cause
high EMA values, and pinpoints fault thresholds. Run this after a drive to
understand how close you got to the fault limit and what caused it.

Usage:
  python tools/car_porting/examples/debug_eps_thermal.py <route1> [route2] ...

Suggested test drive protocol to map the thermal limits:
  1. Highway straight (2 min) — establish baseline EMA
  2. Gentle sustained curve, ~30° angle (30s) — moderate load
  3. Straight recovery (1 min) — watch EMA decay
  4. Tighter sustained curve, ~60° angle (30s) — higher load
  5. Straight recovery (1 min)
  6. Tight curve, ~100°+ angle (hold as long as comfortable) — push toward limit
  7. Straight recovery (2 min)
  8. Repeated direction changes (slalom-style) — test rapid load cycling
  Keep openpilot engaged throughout. The goal is to see what EMA values each
  condition produces and how quickly EMA rises/falls.
"""

import sys
import numpy as np

from openpilot.tools.lib.logreader import LogReader

EPS_MISMATCH_TAU = 15.0  # seconds


def analyze_route(route_identifier):
  print(f"\n{'='*100}")
  print(f"EPS THERMAL DEBUG: {route_identifier}")
  print(f"{'='*100}")

  lr = LogReader(route_identifier)

  car_states = []
  car_controls = []

  for msg in lr:
    w = msg.which()
    if w == "carState":
      car_states.append((msg.logMonoTime, msg.carState))
    elif w == "carControl":
      car_controls.append((msg.logMonoTime, msg.carControl))

  print(f"  Loaded: {len(car_states)} carState, {len(car_controls)} carControl")

  if not car_states:
    print("  ERROR: No carState messages!")
    return

  # Detect carState rate
  if len(car_states) > 10:
    dt_samples = [(car_states[i+1][0] - car_states[i][0]) / 1e9
                  for i in range(min(100, len(car_states)-1))]
    cs_hz = 1.0 / np.mean(dt_samples)
    print(f"  carState rate: ~{cs_hz:.0f}Hz")
  else:
    cs_hz = 100.0

  # Downsample to ~25Hz to match carcontroller STEER_STEP=2
  step = max(1, int(round(cs_hz / 25)))
  sampled = car_states[::step]
  print(f"  Downsampled to {len(sampled)} frames (~25Hz, step={step})")

  # Match carControl to carState for lat_active
  cc_idx = 0
  lat_active_map = {}
  for cc_time, cc in car_controls:
    lat_active_map[cc_time] = cc.latActive

  # Compute mismatch EMA timeline
  alpha = (1.0 / 25.0) / EPS_MISMATCH_TAU
  ema = 0.0
  t0 = sampled[0][0]

  timeline = []
  fault_events = []
  prev_fault = False

  for i, (ts, cs) in enumerate(sampled):
    mismatch = max(0, abs(cs.steeringTorqueEps) - abs(cs.steeringTorque))
    ema += (mismatch - ema) * alpha

    fault = cs.steerFaultTemporary
    if fault and not prev_fault:
      fault_events.append(len(timeline))
    prev_fault = fault

    # Find closest carControl for lat_active
    lat_active = False
    for cc_time, cc in car_controls:
      if cc_time >= ts:
        lat_active = cc.latActive
        break

    timeline.append({
      'time_s': (ts - t0) / 1e9,
      'ema': ema,
      'eps_output': abs(cs.steeringTorqueEps),
      'driver_torque': cs.steeringTorque,
      'angle': cs.steeringAngleDeg,
      'rate': cs.steeringRateDeg,
      'fault': cs.steerFaultTemporary,
      'lat_active': lat_active,
    })

  # Print EMA timeline at 1-second intervals
  print(f"\n  --- EMA Timeline (1s intervals, active frames only) ---")
  print(f"  {'Time':>6} {'EMA':>6} {'|EPS|':>6} {'|Drv|':>6} {'Angle':>7} {'Rate':>7} {'Fault':>6}")

  last_print_s = -1
  for t in timeline:
    sec = int(t['time_s'])
    if sec != last_print_s and t['lat_active']:
      last_print_s = sec
      fault_marker = " <--" if t['fault'] else ""
      print(f"  {t['time_s']:6.1f} {t['ema']:6.1f} {t['eps_output']:6.0f} "
            f"{abs(t['driver_torque']):6.0f} {t['angle']:7.1f} {t['rate']:7.1f} "
            f"{'YES' if t['fault'] else '':>6}{fault_marker}")

  # Fault analysis
  if fault_events:
    print(f"\n  --- Fault Events ({len(fault_events)} total) ---")
    for fi, idx in enumerate(fault_events):
      t = timeline[idx]
      print(f"\n  Fault #{fi+1} at t={t['time_s']:.1f}s:")
      print(f"    EMA at fault:     {t['ema']:.1f}")
      print(f"    |EPS output|:     {t['eps_output']:.0f}")
      print(f"    |Driver torque|:  {abs(t['driver_torque']):.0f}")
      print(f"    Steering angle:   {t['angle']:.1f}°")
      print(f"    Steering rate:    {t['rate']:.1f}°/s")

      # Show EMA trajectory leading up to fault
      lookback = min(idx, 75)  # 3s at 25Hz
      print(f"\n    EMA trajectory (3s before fault):")
      print(f"    {'Time':>6} {'EMA':>6} {'|EPS|':>6} {'Angle':>7}")
      for j in range(idx - lookback, idx + 5):
        if 0 <= j < len(timeline):
          tt = timeline[j]
          marker = " <-- FAULT" if j == idx else ""
          print(f"    {tt['time_s']:6.1f} {tt['ema']:6.1f} {tt['eps_output']:6.0f} {tt['angle']:7.1f}{marker}")
  else:
    print(f"\n  No faults in this route.")

  # EMA statistics
  active = [t for t in timeline if t['lat_active']]
  if active:
    emas = [t['ema'] for t in active]
    print(f"\n  --- EMA Statistics (active frames) ---")
    print(f"  Min EMA:  {min(emas):.1f}")
    print(f"  Max EMA:  {max(emas):.1f}")
    print(f"  Mean EMA: {np.mean(emas):.1f}")
    print(f"  Median:   {np.median(emas):.1f}")

    # Distribution
    brackets = [0, 100, 200, 250, 300, 325, 350, 375, 400, 450, 500]
    print(f"\n  EMA distribution:")
    for i in range(len(brackets) - 1):
      count = sum(1 for e in emas if brackets[i] <= e < brackets[i+1])
      pct = 100 * count / len(emas)
      bar = '#' * int(pct / 2)
      print(f"    {brackets[i]:>4}-{brackets[i+1]:<4}: {count:5d} ({pct:5.1f}%) {bar}")
    count = sum(1 for e in emas if e >= brackets[-1])
    pct = 100 * count / len(emas)
    bar = '#' * int(pct / 2)
    print(f"    {brackets[-1]:>4}+   : {count:5d} ({pct:5.1f}%) {bar}")

    # What angle ranges produce high EMA?
    print(f"\n  --- EMA by Steering Angle ---")
    angle_brackets = [(0, 15), (15, 30), (30, 50), (50, 75), (75, 100), (100, 150), (150, 200)]
    for lo, hi in angle_brackets:
      frames = [t for t in active if lo <= abs(t['angle']) < hi]
      if frames:
        avg_ema = np.mean([t['ema'] for t in frames])
        avg_eps = np.mean([t['eps_output'] for t in frames])
        print(f"    |angle| {lo:>3}-{hi:<3}°: {len(frames):5d} frames, avg EMA={avg_ema:.1f}, avg |EPS|={avg_eps:.0f}")

    # Time above various thresholds
    print(f"\n  --- Time Above EMA Thresholds ---")
    for thresh in [250, 300, 325, 350, 360, 370, 380, 400]:
      count = sum(1 for e in emas if e >= thresh)
      duration = count / 25.0
      pct = 100 * count / len(emas)
      print(f"    EMA >= {thresh}: {duration:.1f}s ({pct:.1f}%)")

    # Peak EMA moments
    print(f"\n  --- Top 5 Peak EMA Moments ---")
    sorted_by_ema = sorted(active, key=lambda t: t['ema'], reverse=True)
    seen = set()
    count = 0
    for t in sorted_by_ema:
      bucket = int(t['time_s'])
      if bucket in seen:
        continue
      seen.add(bucket)
      count += 1
      if count > 5:
        break
      print(f"    t={t['time_s']:6.1f}s: EMA={t['ema']:.1f}, |EPS|={t['eps_output']:.0f}, "
            f"angle={t['angle']:.1f}°, rate={t['rate']:.1f}°/s, |driver|={abs(t['driver_torque']):.0f}")


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print("Usage: python debug_eps_thermal.py <route1> [route2] ...")
    print()
    print("Suggested test drive protocol:")
    print("  1. Highway straight (2 min) - baseline")
    print("  2. Gentle curve ~30° (30s)")
    print("  3. Straight (1 min) - recovery")
    print("  4. Tighter curve ~60° (30s)")
    print("  5. Straight (1 min) - recovery")
    print("  6. Tight curve ~100°+ (hold as long as comfortable)")
    print("  7. Straight (2 min) - recovery")
    print("  8. Slalom / repeated direction changes")
    sys.exit(1)

  for route in sys.argv[1:]:
    try:
      analyze_route(route)
    except Exception as e:
      print(f"\nERROR on {route}: {e}")
      import traceback
      traceback.print_exc()
