#!/usr/bin/env python3
"""
Simulate the new EMA torque scaling carcontroller logic against recorded routes.

Reads carControl (desired torque) and carState (steering rate, driver torque) from
route logs, then simulates the rate check + EMA scaling to show what torque would
actually be applied and whether faults would be prevented.
"""

import sys
import numpy as np

from openpilot.tools.lib.logreader import LogReader

# CarController constants (must match carcontroller.py)
STEER_MAX = 2047
STEER_STEP = 2
STEER_DELTA_UP = 50
STEER_DELTA_DOWN = 70
STEER_DRIVER_ALLOWANCE = 60
STEER_DRIVER_MULTIPLIER = 50
STEER_DRIVER_FACTOR = 1

MAX_STEER_RATE = 20
MAX_STEER_RATE_FRAMES = 10
EPS_MISMATCH_TAU = 10.0
EPS_MISMATCH_SOFT = 400.0
EPS_MISMATCH_HARD = 600.0
EPS_EXCESS_THRESH = 800.0
EPS_EXCESS_TAU = 1.0
EPS_EXCESS_SOFT = 100.0
EPS_EXCESS_HARD = 300.0
EPS_EXCESS_MAX_RATE = 50.0
EPS_MIN_SCALE = 0.3


def common_fault_avoidance(fault_condition, request, above_limit_frames, max_above_limit_frames):
  if dilated := fault_condition or above_limit_frames > 0:
    above_limit_frames += 1
  else:
    above_limit_frames = 0

  if above_limit_frames > max_above_limit_frames:
    request = False

  if above_limit_frames >= max_above_limit_frames + 1:
    above_limit_frames = 0

  return above_limit_frames, request


def apply_driver_steer_torque_limits(new_torque, last_torque, driver_torque):
  """Simplified version of the driver torque limits."""
  # Rate limiting
  max_delta = STEER_DELTA_UP if abs(new_torque) > abs(last_torque) else STEER_DELTA_DOWN
  new_torque = np.clip(new_torque, last_torque - max_delta, last_torque + max_delta)

  # Driver torque interaction
  driver_abs = abs(driver_torque)
  if driver_abs > STEER_DRIVER_ALLOWANCE:
    # Reduce allowed torque when driver is fighting
    driver_factor = max(0, STEER_MAX - (driver_abs - STEER_DRIVER_ALLOWANCE) * STEER_DRIVER_MULTIPLIER)
    new_torque = int(np.clip(new_torque, -driver_factor, driver_factor))

  new_torque = int(np.clip(new_torque, -STEER_MAX, STEER_MAX))
  return new_torque


def simulate_route(route_identifier):
  print(f"\n{'='*100}")
  print(f"SIMULATING: {route_identifier}")
  print(f"{'='*100}")

  lr = LogReader(route_identifier)

  # Collect relevant messages
  car_controls = []
  car_states = []

  for msg in lr:
    w = msg.which()
    if w == "carControl":
      car_controls.append((msg.logMonoTime, msg.carControl))
    elif w == "carState":
      car_states.append((msg.logMonoTime, msg.carState))

  print(f"  Loaded: {len(car_controls)} carControl, {len(car_states)} carState")

  if not car_controls or not car_states:
    print("  ERROR: Missing carControl or carState messages!")
    return

  # Find actual fault events from carState.steerFaultTemporary transitions
  actual_faults = []  # list of (time_ns, index)
  for i in range(1, len(car_states)):
    curr_fault = car_states[i][1].steerFaultTemporary
    prev_fault = car_states[i-1][1].steerFaultTemporary
    if curr_fault and not prev_fault:
      actual_faults.append(car_states[i][0])  # logMonoTime

  print(f"  Actual fault events in route: {len(actual_faults)}")

  # Build time-aligned data at ~25Hz (carControl rate)
  # carControl comes at ~100Hz, we want every 4th sample (or closest to 25Hz)
  # Actually, let's just use all carControl samples and track which ones are "steer frames"

  # Match carState to carControl by timestamp
  cs_idx = 0
  sim_frames = []

  for i, (cc_time, cc) in enumerate(car_controls):
    # Find closest carState
    while cs_idx < len(car_states) - 1 and car_states[cs_idx + 1][0] <= cc_time:
      cs_idx += 1

    cs = car_states[cs_idx][1]

    sim_frames.append({
      'time_ns': cc_time,
      'desired_torque': cc.actuators.torque,  # normalized [-1, 1]
      'lat_active': cc.latActive,
      'steering_rate': cs.steeringRateDeg,
      'driver_torque': cs.steeringTorque,
      'steering_angle': cs.steeringAngleDeg,
      'eps_torque': cs.steeringTorqueEps,
    })

  print(f"  Simulation frames: {len(sim_frames)}")

  # Downsample to ~25Hz (every 4th frame if carControl is at 100Hz)
  # Detect actual rate
  if len(sim_frames) > 10:
    dt_samples = [(sim_frames[i+1]['time_ns'] - sim_frames[i]['time_ns']) / 1e9
                  for i in range(min(100, len(sim_frames)-1))]
    avg_dt = np.mean(dt_samples)
    actual_hz = 1.0 / avg_dt if avg_dt > 0 else 100
    print(f"  carControl rate: ~{actual_hz:.0f}Hz (dt={avg_dt*1000:.1f}ms)")

    # Downsample to ~25Hz
    step = max(1, int(round(actual_hz / 25)))
    sim_frames = sim_frames[::step]
    print(f"  After downsampling (step={step}): {len(sim_frames)} frames")

  # Simulate carcontroller
  apply_torque_last = 0
  steer_rate_last = 0.0
  eps_mismatch_ema = 0.0
  eps_excess_ema = 0.0
  steer_rate_counter = 0

  results = []
  for frame_idx, f in enumerate(sim_frames):
    desired = int(round(f['desired_torque'] * STEER_MAX))

    if not f['lat_active']:
      apply_torque = 0
    else:
      # Driver torque limits
      apply_torque = apply_driver_steer_torque_limits(desired, apply_torque_last, f['driver_torque'])

    if not f['lat_active']:
      apply_torque = 0

    apply_steer_req = f['lat_active']
    original_torque = apply_torque  # before EMA scaling

    # Rate check
    smoothed_rate = (abs(f['steering_rate']) + abs(steer_rate_last)) / 2
    steer_rate_last = f['steering_rate']

    steer_rate_counter, apply_steer_req = \
      common_fault_avoidance(smoothed_rate > MAX_STEER_RATE, apply_steer_req,
                             steer_rate_counter, MAX_STEER_RATE_FRAMES)

    # EPS mismatch tracking and torque scaling
    mismatch = max(0, abs(f['eps_torque']) - abs(f['driver_torque']))

    # General mismatch EMA
    alpha = (1.0 / 25.0) / EPS_MISMATCH_TAU
    eps_mismatch_ema += (mismatch - eps_mismatch_ema) * alpha
    ema_before = eps_mismatch_ema

    # Excess mismatch EMA (sustained high-torque detection, rate-gated)
    excess = max(0, mismatch - EPS_EXCESS_THRESH) if abs(f['steering_rate']) < EPS_EXCESS_MAX_RATE else 0
    alpha_excess = (1.0 / 25.0) / EPS_EXCESS_TAU
    eps_excess_ema += (excess - eps_excess_ema) * alpha_excess

    scale = 1.0
    if eps_mismatch_ema > EPS_MISMATCH_SOFT:
      scale = min(scale, 1.0 - (eps_mismatch_ema - EPS_MISMATCH_SOFT) /
                               (EPS_MISMATCH_HARD - EPS_MISMATCH_SOFT))
    if eps_excess_ema > EPS_EXCESS_SOFT:
      scale = min(scale, 1.0 - (eps_excess_ema - EPS_EXCESS_SOFT) /
                               (EPS_EXCESS_HARD - EPS_EXCESS_SOFT))
    scale = max(EPS_MIN_SCALE, scale)
    if scale < 1.0:
      apply_torque = int(round(apply_torque * scale))

    apply_torque_last = apply_torque

    results.append({
      'time_s': (f['time_ns'] - sim_frames[0]['time_ns']) / 1e9,
      'desired': desired,
      'original': original_torque,
      'applied': apply_torque,
      'scale': scale,
      'ema': ema_before,
      'excess_ema': eps_excess_ema,
      'steer_req': apply_steer_req,
      'rate': f['steering_rate'],
      'angle': f['steering_angle'],
      'lat_active': f['lat_active'],
    })

  # Analysis: find periods where EMA scaling was active
  scaling_periods = []
  in_scaling = False
  scale_start = 0
  for i, r in enumerate(results):
    if r['scale'] < 1.0 and r['lat_active']:
      if not in_scaling:
        in_scaling = True
        scale_start = i
    else:
      if in_scaling:
        in_scaling = False
        scaling_periods.append((scale_start, i))
  if in_scaling:
    scaling_periods.append((scale_start, len(results)))

  # Summary stats
  active_frames = [r for r in results if r['lat_active']]
  if active_frames:
    scaled_frames = [r for r in active_frames if r['scale'] < 1.0]
    pct_scaled = 100 * len(scaled_frames) / len(active_frames)

    avg_scale_when_active = np.mean([r['scale'] for r in active_frames])
    avg_scale_when_scaling = np.mean([r['scale'] for r in scaled_frames]) if scaled_frames else 1.0
    min_scale = min(r['scale'] for r in active_frames)
    max_ema = max(r['ema'] for r in active_frames)
    max_excess_ema = max(r['excess_ema'] for r in active_frames)

    avg_torque_reduction = np.mean([1.0 - abs(r['applied']) / max(1, abs(r['original']))
                                    for r in active_frames if abs(r['original']) > 50])

    print(f"\n  --- EMA Scaling Summary ---")
    print(f"  Active frames: {len(active_frames)}")
    print(f"  Frames with scaling: {len(scaled_frames)} ({pct_scaled:.1f}%)")
    print(f"  Avg scale (all active): {avg_scale_when_active:.3f}")
    print(f"  Avg scale (when scaling): {avg_scale_when_scaling:.3f}")
    print(f"  Min scale: {min_scale:.3f}")
    print(f"  Max general EMA: {max_ema:.1f}")
    print(f"  Max excess EMA: {max_excess_ema:.1f}")
    print(f"  Avg torque reduction: {avg_torque_reduction*100:.1f}%")
    print(f"  Number of scaling periods: {len(scaling_periods)}")

    # Show each scaling period
    if scaling_periods:
      print(f"\n  --- Scaling Periods ---")
      for sp_idx, (start, end) in enumerate(scaling_periods):
        duration = results[end-1]['time_s'] - results[start]['time_s']
        min_s = min(results[j]['scale'] for j in range(start, end))
        max_ema_period = max(results[j]['ema'] for j in range(start, end))
        avg_original = np.mean([abs(results[j]['original']) for j in range(start, end)])
        avg_applied = np.mean([abs(results[j]['applied']) for j in range(start, end)])
        print(f"  Period {sp_idx+1}: t={results[start]['time_s']:.1f}-{results[end-1]['time_s']:.1f}s "
              f"({duration:.1f}s), min_scale={min_s:.3f}, max_ema={max_ema_period:.0f}, "
              f"avg |torque| {avg_original:.0f}→{avg_applied:.0f}")

  # Check if the actual faults would have been prevented
  if actual_faults:
    print(f"\n  --- Fault Prevention Analysis ---")
    t0 = sim_frames[0]['time_ns']
    for fi, fault_time_ns in enumerate(actual_faults):
      fault_time_s = (fault_time_ns - t0) / 1e9

      # Look at the 3s window before the fault
      window_start_time = fault_time_s - 3.0
      window_frames = [r for r in results
                       if window_start_time <= r['time_s'] <= fault_time_s]

      if window_frames:
        avg_applied = np.mean([abs(r['applied']) for r in window_frames])
        avg_original = np.mean([abs(r['original']) for r in window_frames])
        avg_scale = np.mean([r['scale'] for r in window_frames])
        max_ema_window = max(r['ema'] for r in window_frames)
        pct_steer_req_cut = 100 * sum(1 for r in window_frames if not r['steer_req']) / len(window_frames)

        # Estimate 3s mismatch integral with simulated torque
        # (rough: assume EPS output ≈ apply_torque, driver ≈ 0)
        sim_integral = sum(abs(r['applied']) * (1.0/25.0) for r in window_frames)
        orig_integral = sum(abs(r['original']) * (1.0/25.0) for r in window_frames)

        print(f"\n  Fault #{fi+1} at ~{fault_time_s:.1f}s:")
        print(f"    3s window: avg |torque| {avg_original:.0f}→{avg_applied:.0f} "
              f"(avg scale={avg_scale:.3f}, max EMA={max_ema_window:.0f})")
        print(f"    Steer req cut: {pct_steer_req_cut:.1f}% of frames")
        print(f"    3s torque integral: {orig_integral:.0f}→{sim_integral:.0f} "
              f"(threshold ~4000)")
        print(f"    Predicted: {'PREVENTED' if sim_integral < 3500 else 'BORDERLINE' if sim_integral < 4000 else 'STILL FAULTS'}")
      else:
        print(f"\n  Fault #{fi+1} at ~{fault_time_s:.1f}s: (outside simulation window)")
  else:
    print(f"\n  No actual faults in this route — checking torque availability")
    if active_frames:
      # Show what percentage of torque was available
      high_torque_frames = [r for r in active_frames if abs(r['original']) > 500]
      if high_torque_frames:
        avg_retention = np.mean([abs(r['applied']) / max(1, abs(r['original']))
                                 for r in high_torque_frames])
        print(f"  High-torque frames (|cmd|>500): {len(high_torque_frames)}")
        print(f"  Avg torque retention: {avg_retention*100:.1f}%")

  # Show a few representative moments
  print(f"\n  --- Peak Torque Moments ---")
  # Find top 5 moments by desired torque
  sorted_by_torque = sorted([r for r in results if r['lat_active']],
                            key=lambda r: abs(r['original']), reverse=True)
  seen_times = set()
  count = 0
  for r in sorted_by_torque:
    t_bucket = int(r['time_s'])
    if t_bucket in seen_times:
      continue
    seen_times.add(t_bucket)
    count += 1
    if count > 5:
      break
    print(f"  t={r['time_s']:6.1f}s: desired={r['desired']:5d}, cmd={r['original']:5d}, "
          f"applied={r['applied']:5d}, scale={r['scale']:.3f}, EMA={r['ema']:.0f}, "
          f"excess={r['excess_ema']:.0f}, rate={r['rate']:.1f}°/s, angle={r['angle']:.1f}°")


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print("Usage: python simulate_ema.py <route1> [route2] ...")
    sys.exit(1)

  for route in sys.argv[1:]:
    try:
      simulate_route(route)
    except Exception as e:
      print(f"\nERROR on {route}: {e}")
      import traceback
      traceback.print_exc()
