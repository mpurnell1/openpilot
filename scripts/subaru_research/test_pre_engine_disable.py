#!/usr/bin/env python3
"""Disable EyeSight before engine start, beep when ready to start engine."""
import sys
import time
import subprocess

sys.path.insert(0, '/data/openpilot/panda')
from panda import Panda

TX, RX, BUS = 0x787, 0x78F, 2

def sr(p, payload, timeout=0.5):
  for _ in range(3):
    if not p.can_recv():
      break
    time.sleep(0.005)
  n = len(payload)
  p.can_send(TX, bytes([n]) + payload + bytes(7 - n), BUS)
  start = time.monotonic()
  while time.monotonic() - start < timeout:
    for addr, dat, bus in p.can_recv():
      if bus == BUS and addr == RX:
        return dat[1:1 + (dat[0] & 0x0F)]
    time.sleep(0.003)
  return None

def beep():
  try:
    subprocess.Popen(
      ["aplay", "-q", "/data/openpilot/system/assets/sounds/prompt.wav"],
      stderr=subprocess.DEVNULL
    )
  except Exception:
    pass

p = Panda()
p.set_safety_mode(17)
p.can_clear(0xFFFF)
time.sleep(0.5)

print("Step 1: Disabling EyeSight (engine must be OFF)...")
r = sr(p, bytes([0x10, 0x03]))
print(f"  Session: {r.hex() if r else 'TIMEOUT'}")
time.sleep(0.2)
r = sr(p, bytes([0x28, 0x03, 0x01]))

if r and r[0] == 0x68:
  print("  EyeSight DISABLED!")
  print()
  print(">>> BEEP = START THE ENGINE NOW <<<")
  time.sleep(1)
  beep()
  time.sleep(1)
  beep()
  print()
  print("Sending tester present keepalive for 60s...")
  for i in range(120):
    p.can_send(TX, bytes([0x02, 0x3E, 0x80, 0, 0, 0, 0, 0]), BUS)
    if i % 20 == 0:
      print(f"  {i * 0.5:.0f}s")
    time.sleep(0.5)
  print("Done. 60s elapsed.")
else:
  print(f"  FAILED: {r.hex() if r else 'TIMEOUT'}")

p.set_safety_mode(0)
