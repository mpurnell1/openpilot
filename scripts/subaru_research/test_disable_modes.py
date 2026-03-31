#!/usr/bin/env python3
"""
Test disable_ecu under different panda safety modes and conditions.
Single ignition cycle, tests all scenarios.
"""
import time
import sys

sys.path.insert(0, '/data/openpilot/panda')
from panda import Panda

TX = 0x787
RX = 0x78F
BUS = 2

def send_recv(p, payload, timeout=0.5):
  for _ in range(3):
    if not p.can_recv(): break
    time.sleep(0.005)
  n = len(payload)
  p.can_send(TX, bytes([n]) + payload + bytes(7 - n), BUS)
  start = time.monotonic()
  while time.monotonic() - start < timeout:
    for addr, dat, bus in p.can_recv():
      if bus == BUS and addr == RX:
        ft = (dat[0] >> 4) & 0x0F
        if ft == 0:
          return dat[1:1 + (dat[0] & 0x0F)]
        elif ft == 1:
          total = ((dat[0] & 0x0F) << 8) | dat[1]
          data = dat[2:8]
          p.can_send(TX, bytes([0x30, 0, 0x0A, 0, 0, 0, 0, 0]), BUS)
          s2 = time.monotonic()
          while len(data) < total and time.monotonic() - s2 < timeout:
            for a2, d2, b2 in p.can_recv():
              if b2 == BUS and a2 == RX and (d2[0] >> 4) == 2:
                data += d2[1:8]
            time.sleep(0.002)
          return data[:total]
    time.sleep(0.003)
  return None

def try_disable(p, label):
  print(f"\n  --- {label} ---")
  p.can_clear(0xFFFF)
  time.sleep(0.1)

  r = send_recv(p, bytes([0x10, 0x03]))
  if r is None:
    print(f"  ExtDiag: TIMEOUT (ECU not responding)")
    return "no_response"
  elif r[0] == 0x50:
    print(f"  ExtDiag: OK ({r.hex()})")
  else:
    print(f"  ExtDiag: {r.hex()}")
    return "session_fail"

  time.sleep(0.05)

  r = send_recv(p, bytes([0x28, 0x03, 0x01]))
  if r is None:
    print(f"  CommCtrl: TIMEOUT")
    return "comm_timeout"
  elif r[0] == 0x68:
    print(f"  CommCtrl: OK! EyeSight DISABLED")
    # Re-enable immediately for next test
    send_recv(p, bytes([0x10, 0x03]))
    time.sleep(0.05)
    send_recv(p, bytes([0x28, 0x00, 0x01]))
    time.sleep(0.1)
    send_recv(p, bytes([0x10, 0x01]))
    return "success"
  elif r[0] == 0x7F:
    nrc = r[2]
    names = {0x22: "conditionsNotCorrect", 0x12: "subFuncNotSupported", 0x7f: "svcNotSupportedInSession"}
    print(f"  CommCtrl: NRC 0x{nrc:02x} ({names.get(nrc, '?')})")
    send_recv(p, bytes([0x10, 0x01]))
    return f"nrc_0x{nrc:02x}"
  else:
    print(f"  CommCtrl: {r.hex()}")
    return "unexpected"

def simulate_fingerprint(p):
  """Send the same UDS queries that openpilot fingerprinting sends"""
  print("  Simulating fingerprint queries...")
  # TesterPresent
  send_recv(p, bytes([0x3E, 0x00]), timeout=0.3)
  time.sleep(0.05)
  # Read F182 (multi-frame)
  send_recv(p, bytes([0x22, 0xF1, 0x82]), timeout=0.5)
  time.sleep(0.05)
  # Read F100
  send_recv(p, bytes([0x22, 0xF1, 0x00]), timeout=0.3)
  time.sleep(0.05)
  # Enter default session
  send_recv(p, bytes([0x10, 0x01]), timeout=0.3)
  time.sleep(0.05)
  # TesterPresent again
  send_recv(p, bytes([0x3E, 0x00]), timeout=0.3)
  time.sleep(0.1)

def main():
  p = Panda()
  print(f"Panda: {p.get_serial()[0]}")

  ALLOUTPUT = 17
  ELM327 = 3  # CarParams.SafetyModel.elm327
  SUBARU = 31  # need to check this

  # Find the correct subaru safety model value
  try:
    from opendbc.car.structs import CarParams
    SUBARU = int(CarParams.SafetyModel.subaru)
    print(f"Subaru safety model: {SUBARU}")
  except:
    SUBARU = 31
    print(f"Subaru safety model (guessed): {SUBARU}")

  results = {}

  # Test 1: allOutput mode, clean state
  print("\n" + "=" * 60)
  print("TEST 1: allOutput mode, clean state")
  p.set_safety_mode(ALLOUTPUT)
  time.sleep(0.3)
  p.can_clear(0xFFFF)
  time.sleep(0.2)
  results["alloutput_clean"] = try_disable(p, "allOutput clean")

  # Test 2: elm327 mode (param=1), clean state
  print("\n" + "=" * 60)
  print("TEST 2: elm327 mode (param=1), clean state")
  p.set_safety_mode(ELM327, 1)
  time.sleep(0.3)
  p.can_clear(0xFFFF)
  time.sleep(0.2)
  results["elm327_clean"] = try_disable(p, "elm327 clean")

  # Test 3: elm327 mode after fingerprint-like queries
  print("\n" + "=" * 60)
  print("TEST 3: elm327 mode, after fingerprint queries")
  p.set_safety_mode(ELM327, 1)
  time.sleep(0.3)
  p.can_clear(0xFFFF)
  time.sleep(0.2)
  simulate_fingerprint(p)
  results["elm327_after_fp"] = try_disable(p, "elm327 after fingerprint")

  # Test 4: elm327 -> switch to subaru -> try disable
  print("\n" + "=" * 60)
  print("TEST 4: switch elm327 -> subaru, then disable")
  p.set_safety_mode(ELM327, 1)
  time.sleep(0.3)
  p.can_clear(0xFFFF)
  time.sleep(0.1)
  p.set_safety_mode(SUBARU, 11)  # param=11 = GEN2+LONG+LKAS_ANGLE
  time.sleep(0.3)
  p.can_clear(0xFFFF)
  time.sleep(0.2)
  results["subaru_mode"] = try_disable(p, "subaru mode")

  # Test 5: elm327 with fingerprint, then switch to subaru, then disable
  print("\n" + "=" * 60)
  print("TEST 5: elm327 + fingerprint -> subaru -> disable")
  p.set_safety_mode(ELM327, 1)
  time.sleep(0.3)
  p.can_clear(0xFFFF)
  time.sleep(0.2)
  simulate_fingerprint(p)
  p.set_safety_mode(SUBARU, 11)
  time.sleep(0.3)
  p.can_clear(0xFFFF)
  time.sleep(0.2)
  results["fp_then_subaru"] = try_disable(p, "fingerprint then subaru")

  # Test 6: allOutput after fingerprint
  print("\n" + "=" * 60)
  print("TEST 6: allOutput after fingerprint queries")
  p.set_safety_mode(ELM327, 1)
  time.sleep(0.3)
  p.can_clear(0xFFFF)
  time.sleep(0.2)
  simulate_fingerprint(p)
  p.set_safety_mode(ALLOUTPUT)
  time.sleep(0.3)
  p.can_clear(0xFFFF)
  time.sleep(0.2)
  results["alloutput_after_fp"] = try_disable(p, "allOutput after fingerprint")

  # Summary
  print("\n" + "=" * 60)
  print("SUMMARY")
  print("=" * 60)
  for test, result in results.items():
    status = "PASS" if result == "success" else "FAIL"
    print(f"  {test:<30} {status:>6}  ({result})")

  p.set_safety_mode(0)
  print("\nDone.")

if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    Panda().set_safety_mode(0)
