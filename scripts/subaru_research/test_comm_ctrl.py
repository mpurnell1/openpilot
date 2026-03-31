#!/usr/bin/env python3
"""Test all CommunicationControl subfunctions to find one the ECU accepts while engine running."""
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
    time.sleep(0.003)
  return None

p = Panda()
p.set_safety_mode(17)  # allOutput
p.can_clear(0xFFFF)
time.sleep(0.5)

# Enter extended diagnostic session
r = send_recv(p, bytes([0x10, 0x03]))
print(f"Session: {r.hex() if r else 'TIMEOUT'}")

tests = [
    (0x00, 0x01, "ENABLE_RX_ENABLE_TX + NORMAL"),
    (0x01, 0x01, "ENABLE_RX_DISABLE_TX + NORMAL"),
    (0x02, 0x01, "DISABLE_RX_ENABLE_TX + NORMAL"),
    (0x03, 0x01, "DISABLE_RX_DISABLE_TX + NORMAL (current)"),
    (0x00, 0x02, "ENABLE_RX_ENABLE_TX + NM"),
    (0x01, 0x02, "ENABLE_RX_DISABLE_TX + NM"),
    (0x02, 0x02, "DISABLE_RX_ENABLE_TX + NM"),
    (0x03, 0x02, "DISABLE_RX_DISABLE_TX + NM"),
    (0x00, 0x03, "ENABLE_RX_ENABLE_TX + NORMAL_AND_NM"),
    (0x01, 0x03, "ENABLE_RX_DISABLE_TX + NORMAL_AND_NM"),
    (0x03, 0x03, "DISABLE_RX_DISABLE_TX + NORMAL_AND_NM"),
    (0x80, 0x01, "ENABLE_RX_ENABLE_TX + suppress + NORMAL"),
    (0x81, 0x01, "ENABLE_RX_DISABLE_TX + suppress + NORMAL"),
    (0x83, 0x01, "DISABLE_RX_DISABLE_TX + suppress + NORMAL (Honda style)"),
]

print(f"\n{'SubFunc':>8} {'MsgType':>8} {'Description':<50} {'Result'}")
print("-" * 90)

for subfunc, msgtype, desc in tests:
    # Re-enter session each time
    send_recv(p, bytes([0x10, 0x03]))
    time.sleep(0.1)

    r = send_recv(p, bytes([0x28, subfunc, msgtype]))
    if r is None:
        result = "TIMEOUT"
    elif r[0] == 0x68:
        result = f"OK! ({r.hex()})"
        # Re-enable
        send_recv(p, bytes([0x28, 0x00, 0x01]))
        time.sleep(0.1)
    elif r[0] == 0x7F:
        nrc = r[2]
        names = {0x12: "subFuncNotSupported", 0x22: "conditionsNotCorrect",
                 0x31: "requestOutOfRange", 0x7e: "subFuncNotInSession", 0x7f: "svcNotInSession"}
        result = f"NRC 0x{nrc:02x} ({names.get(nrc, '?')})"
    else:
        result = r.hex()

    print(f"  0x{subfunc:02X}     0x{msgtype:02X}     {desc:<50} {result}")

    send_recv(p, bytes([0x10, 0x01]))
    time.sleep(0.1)

p.set_safety_mode(0)
print("\nDone.")
