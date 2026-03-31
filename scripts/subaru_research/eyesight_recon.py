#!/usr/bin/env python3
"""
EyeSight ECU Reconnaissance Script
Runs on comma 3x via SSH. Stop openpilot first: tmux kill-session -t comma

Standalone: only depends on panda package (no opendbc/numpy).
"""
import time
import struct
import sys

sys.path.insert(0, '/data/openpilot/panda')
sys.path.insert(0, '/data/openpilot')
from panda import Panda

SAFETY_ALLOUTPUT = 17
SAFETY_SILENT = 0

EYESIGHT_TX = 0x787
EYESIGHT_RX = 0x78F
BUS = 2

NRC_NAMES = {
  0x10: 'general reject',
  0x11: 'service not supported',
  0x12: 'sub-function not supported',
  0x13: 'incorrect message length or invalid format',
  0x14: 'response too long',
  0x21: 'busy repeat request',
  0x22: 'conditions not correct',
  0x24: 'request sequence error',
  0x25: 'no response from subnet component',
  0x26: 'failure prevents execution of requested action',
  0x31: 'request out of range',
  0x33: 'security access denied',
  0x35: 'invalid key',
  0x36: 'exceed number of attempts',
  0x37: 'required time delay not expired',
  0x70: 'upload download not accepted',
  0x71: 'transfer data suspended',
  0x72: 'general programming failure',
  0x73: 'wrong block sequence counter',
  0x78: 'request correctly received - response pending',
  0x7e: 'sub-function not supported in active session',
  0x7f: 'service not supported in active session',
  0x81: 'rpm too high',
  0x82: 'rpm too low',
  0x83: 'engine is running',
  0x84: 'engine is not running',
  0x85: 'engine run time too low',
  0x86: 'temperature too high',
  0x87: 'temperature too low',
  0x88: 'vehicle speed too high',
  0x89: 'vehicle speed too low',
  0x8a: 'throttle/pedal too high',
  0x8b: 'throttle/pedal too low',
  0x8c: 'transmission not in neutral',
  0x8d: 'transmission not in gear',
  0x8f: 'brake switch(es) not closed',
  0x90: 'shifter lever not in park',
  0x91: 'torque converter clutch locked',
  0x92: 'voltage too high',
  0x93: 'voltage too low',
}

def nrc_name(code):
  return NRC_NAMES.get(code, f"unknown_0x{code:02x}")


class SimpleUDS:
  """Minimal single-frame UDS client using raw panda CAN. No ISO-TP multi-frame."""

  def __init__(self, panda, tx_addr, rx_addr, bus, timeout=0.5):
    self.p = panda
    self.tx = tx_addr
    self.rx = rx_addr
    self.bus = bus
    self.timeout = timeout

  def _drain(self):
    for _ in range(10):
      msgs = self.p.can_recv()
      if not msgs:
        break
      time.sleep(0.01)

  def _send(self, payload):
    n = len(payload)
    if n <= 7:
      frame = bytes([n]) + payload + bytes(7 - n)
      self.p.can_send(self.tx, frame, self.bus)
    else:
      total_len = n
      first_data = payload[:6]
      frame = bytes([(0x10 | ((total_len >> 8) & 0x0F)), total_len & 0xFF]) + first_data
      self.p.can_send(self.tx, frame, self.bus)

      dat = self._recv_single(timeout=1.0)
      if dat is None or (dat[0] >> 4) != 3:
        return

      idx = 1
      offset = 6
      while offset < total_len:
        chunk = payload[offset:offset + 7]
        cf = bytes([0x20 | (idx & 0x0F)]) + chunk + bytes(max(0, 7 - len(chunk)))
        self.p.can_send(self.tx, cf, self.bus)
        idx += 1
        offset += 7
        time.sleep(0.01)

  def _recv_single(self, timeout=None):
    if timeout is None:
      timeout = self.timeout
    start = time.monotonic()
    while time.monotonic() - start < timeout:
      msgs = self.p.can_recv()
      for addr, dat, bus in msgs:
        if bus == self.bus and addr == self.rx:
          return dat
      time.sleep(0.005)
    return None

  def _recv_multiframe(self, first_frame_dat, timeout=None):
    if timeout is None:
      timeout = self.timeout
    total_len = ((first_frame_dat[0] & 0x0F) << 8) | first_frame_dat[1]
    data = first_frame_dat[2:8]

    fc = bytes([0x30, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00])
    self.p.can_send(self.tx, fc, self.bus)

    expected_idx = 1
    start = time.monotonic()
    while len(data) < total_len and time.monotonic() - start < timeout:
      msgs = self.p.can_recv()
      for addr, dat, bus in msgs:
        if bus == self.bus and addr == self.rx:
          frame_type = (dat[0] >> 4) & 0x0F
          if frame_type == 2:
            idx = dat[0] & 0x0F
            data += dat[1:8]
            expected_idx = (expected_idx + 1) & 0x0F
      time.sleep(0.002)
    return data[:total_len]

  def request(self, payload, timeout=None):
    self._drain()
    self._send(payload)

    response_pending_deadline = None
    effective_timeout = timeout if timeout else self.timeout

    while True:
      dat = self._recv_single(timeout=effective_timeout)
      if dat is None:
        return ("TIMEOUT", None)

      frame_type = (dat[0] >> 4) & 0x0F

      if frame_type == 0:
        length = dat[0] & 0x0F
        resp = dat[1:1 + length]
      elif frame_type == 1:
        resp = self._recv_multiframe(dat, timeout=effective_timeout)
      else:
        continue

      if len(resp) == 0:
        continue

      if resp[0] == 0x7F and len(resp) >= 3:
        error_code = resp[2]
        if error_code == 0x78:
          effective_timeout = 12
          continue
        return ("NRC", resp[2], resp)

      expected_sid = payload[0] + 0x40
      if resp[0] == expected_sid:
        return ("OK", resp)

      return ("UNEXPECTED", resp)

  def enter_session(self, session_type):
    return self.request(bytes([0x10, session_type]))

  def tester_present(self):
    return self.request(bytes([0x3E, 0x00]))

  def read_did(self, did):
    return self.request(bytes([0x22]) + struct.pack('!H', did))

  def security_access_seed(self, level):
    return self.request(bytes([0x27, level]))

  def read_memory(self, addr, size):
    data = bytes([0x14, 0x00, 0x00, 0x00]) + struct.pack('!I', addr) + struct.pack('!B', min(size, 255))
    return self.request(bytes([0x23, 0x14]) + struct.pack('!I', addr) + bytes([min(size, 255)]))


def phase1_services(uds):
  print("\n" + "=" * 70)
  print("PHASE 1A: UDS Service Scan (in Extended Diagnostic Session)")
  print("=" * 70)

  tests = [
    (bytes([0x10, 0x01]), "DiagSession - Default (0x01)"),
    (bytes([0x10, 0x02]), "DiagSession - Programming (0x02)"),
    (bytes([0x10, 0x03]), "DiagSession - Extended (0x03)"),
    (bytes([0x10, 0x04]), "DiagSession - Safety System (0x04)"),
    (bytes([0x11, 0x01]), "ECU Reset - Hard"),
    (bytes([0x11, 0x03]), "ECU Reset - Soft"),
    (bytes([0x3E, 0x00]), "Tester Present"),
    (bytes([0x27, 0x01]), "Security Access - Seed L1"),
    (bytes([0x27, 0x03]), "Security Access - Seed L2"),
    (bytes([0x27, 0x05]), "Security Access - Seed L3"),
    (bytes([0x27, 0x07]), "Security Access - Seed L4"),
    (bytes([0x27, 0x09]), "Security Access - Seed L5"),
    (bytes([0x27, 0x0B]), "Security Access - Seed L6"),
    (bytes([0x28, 0x00, 0x01]), "CommControl - Enable RX+TX"),
    (bytes([0x19, 0x01, 0xFF]), "Read DTC - Count"),
    (bytes([0x19, 0x02, 0xFF]), "Read DTC - By Status"),
    (bytes([0x19, 0x0A]), "Read DTC - Supported"),
    (bytes([0x14, 0xFF, 0xFF, 0xFF]), "Clear DTC Info"),
    (bytes([0x31, 0x01, 0xFF, 0x00]), "Routine Control - Start"),
    (bytes([0x23, 0x14, 0x00, 0x00, 0x00, 0x00, 0x10]), "ReadMemByAddr (0x0, 16B)"),
    (bytes([0x34, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x10]), "Request Download (bare)"),
    (bytes([0x35, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x10]), "Request Upload (bare)"),
    (bytes([0x85, 0x02]), "Control DTC Setting - OFF"),
    (bytes([0x83, 0x01]), "Access Timing Params"),
    (bytes([0x2E, 0xF1, 0x90, 0x00]), "WriteDataByID (bare)"),
    (bytes([0x2F, 0xF1, 0x00, 0x00]), "IO Control By ID (bare)"),
  ]

  print(f"\n{'Test':<45} {'Result':<10} {'Detail'}")
  print("-" * 95)

  for payload, label in tests:
    result = uds.request(payload)
    status = result[0]

    if status == "OK":
      resp = result[1]
      print(f"  {label:<43} {'OK':<10} {resp.hex()}")
    elif status == "NRC":
      code = result[1]
      print(f"  {label:<43} {'NRC':<10} 0x{code:02x} = {nrc_name(code)}")
    elif status == "TIMEOUT":
      print(f"  {label:<43} {'TIMEOUT':<10}")
    else:
      resp = result[1] if len(result) > 1 else None
      print(f"  {label:<43} {status:<10} {resp.hex() if resp else ''}")

    if label.startswith("DiagSession") and "0x03" in label:
      pass
    elif label.startswith("DiagSession"):
      uds.enter_session(0x03)
      time.sleep(0.05)


def phase1_dids(uds):
  print("\n" + "=" * 70)
  print("PHASE 1B: DID Scan (ReadDataByIdentifier 0x22)")
  print("=" * 70)

  dids = {}
  for v in range(0xF180, 0xF1A0):
    dids[v] = f"STD_0x{v:04X}"
  dids[0xF180] = "BOOT_SW_ID"
  dids[0xF181] = "APP_SW_ID"
  dids[0xF182] = "APP_DATA_ID"
  dids[0xF183] = "BOOT_SW_FP"
  dids[0xF184] = "APP_SW_FP"
  dids[0xF185] = "APP_DATA_FP"
  dids[0xF186] = "ACTIVE_DIAG_SESSION"
  dids[0xF187] = "SPARE_PART_NUM"
  dids[0xF188] = "ECU_SW_NUM"
  dids[0xF189] = "ECU_SW_VERSION"
  dids[0xF18A] = "SYS_SUPPLIER_ID"
  dids[0xF18B] = "MFG_DATE"
  dids[0xF18C] = "ECU_SERIAL"
  dids[0xF18D] = "SUPPORTED_FUNC_UNITS"
  dids[0xF18E] = "KIT_ASSEMBLY_PART"
  dids[0xF190] = "VIN"
  dids[0xF191] = "ECU_HW_NUM"
  dids[0xF192] = "SYS_SUPPLIER_ECU_HW"
  dids[0xF193] = "SYS_SUPPLIER_ECU_HW_VER"
  dids[0xF194] = "SYS_SUPPLIER_ECU_SW"
  dids[0xF195] = "SYS_SUPPLIER_ECU_SW_VER"
  dids[0xF196] = "TYPE_APPROVAL"
  dids[0xF197] = "SYS_NAME_ENGINE_TYPE"
  dids[0xF198] = "REPAIR_SHOP_CODE"
  dids[0xF199] = "PROGRAMMING_DATE"
  dids[0xF19A] = "CALIBRATION_SHOP_CODE"
  dids[0xF19B] = "CALIBRATION_DATE"
  dids[0xF19C] = "CALIBRATION_EQ_SW_NUM"
  dids[0xF19D] = "ECU_INSTALL_DATE"
  dids[0xF19E] = "ODX_FILE"
  dids[0xF19F] = "ENTITY"

  for v in range(0xF100, 0xF180):
    dids[v] = f"MFR_0x{v:04X}"
  dids[0xF100] = "SUBARU_ALT_VERSION"

  for v in range(0xF1A0, 0xF200):
    dids[v] = f"SUPPLIER_0x{v:04X}"

  for v in [0x1130, 0x0100, 0x0101, 0x0200, 0x0201, 0x0000, 0x0001]:
    dids[v] = f"SUBARU_0x{v:04X}"

  readable = {}
  denied = {}
  print(f"\nScanning {len(dids)} DIDs...")
  print(f"\n{'DID':<8} {'Name':<30} {'Result'}")
  print("-" * 90)

  for did_val in sorted(dids.keys()):
    did_name = dids[did_val]
    result = uds.read_did(did_val)
    status = result[0]

    if status == "OK":
      resp = result[1]
      if len(resp) >= 3:
        payload = resp[3:]
        readable[did_val] = payload
        ascii_repr = ""
        try:
          text = payload.decode('ascii', errors='replace')
          if any(c.isalnum() for c in text):
            ascii_repr = f'  "{text}"'
        except:
          pass
        print(f"  0x{did_val:04X} {did_name:<28} OK  {payload.hex()}{ascii_repr}")
    elif status == "NRC":
      code = result[1]
      if code == 0x33:
        denied[did_val] = did_name
        print(f"  0x{did_val:04X} {did_name:<28} SECURITY_DENIED")
      elif code not in (0x11, 0x31, 0x12):
        print(f"  0x{did_val:04X} {did_name:<28} NRC 0x{code:02x} = {nrc_name(code)}")
    elif status == "TIMEOUT":
      print(f"  0x{did_val:04X} {did_name:<28} TIMEOUT (multi-frame?)")

  print(f"\n--- Summary: {len(readable)} readable, {len(denied)} security-locked ---")
  if denied:
    print("Security-locked DIDs:")
    for did_val, name in sorted(denied.items()):
      print(f"  0x{did_val:04X} {name}")

  return readable, denied


def phase2_security(uds):
  print("\n" + "=" * 70)
  print("PHASE 2: Session Escalation & Security Probing")
  print("=" * 70)

  sessions = [
    (0x03, "Extended Diagnostic"),
    (0x02, "Programming"),
    (0x04, "Safety System Diagnostic"),
  ]

  for session_id, session_name in sessions:
    print(f"\n--- Session: {session_name} (0x{session_id:02x}) ---")
    result = uds.enter_session(session_id)
    if result[0] != "OK":
      detail = f"0x{result[1]:02x} = {nrc_name(result[1])}" if result[0] == "NRC" else str(result)
      print(f"  Entry: {result[0]} {detail}")
      continue
    print(f"  Entry: OK")

    for level in range(1, 0x42, 2):
      result = uds.security_access_seed(level)
      if result[0] == "OK":
        resp = result[1]
        seed = resp[2:]
        is_zero = all(b == 0 for b in seed)
        note = " << ALREADY UNLOCKED" if is_zero else ""
        print(f"  SA Level 0x{level:02x}: seed = {seed.hex()} ({len(seed)} bytes){note}")
      elif result[0] == "NRC":
        code = result[1]
        if code == 0x12:
          continue
        print(f"  SA Level 0x{level:02x}: NRC 0x{code:02x} = {nrc_name(code)}")
        if code == 0x37:
          print(f"    ^ Security lockout active, stopping SA scan")
          break
      elif result[0] == "TIMEOUT":
        print(f"  SA Level 0x{level:02x}: TIMEOUT")

    uds.enter_session(0x03)
    time.sleep(0.1)


def phase3_memory(uds):
  print("\n" + "=" * 70)
  print("PHASE 3: Memory Read Attempts (ReadMemoryByAddress 0x23)")
  print("=" * 70)

  addrs = [
    (0x00000000, 16, "Base 0x0"),
    (0x00000100, 16, "0x100"),
    (0x00001000, 16, "0x1000"),
    (0x00008000, 16, "0x8000"),
    (0x00010000, 16, "0x10000"),
    (0x00020000, 16, "0x20000"),
    (0x00080000, 16, "0x80000"),
    (0x00100000, 16, "0x100000"),
    (0x00800000, 16, "0x800000"),
    (0x08000000, 16, "0x8000000 (STM32 flash)"),
    (0x20000000, 16, "0x20000000 (SRAM)"),
    (0x40000000, 16, "0x40000000"),
    (0x80000000, 16, "0x80000000"),
    (0xFFFF0000, 16, "0xFFFF0000"),
  ]

  print(f"\n{'Address':<22} {'Label':<30} {'Result'}")
  print("-" * 90)

  for addr, size, label in addrs:
    payload = bytes([0x23, 0x14]) + struct.pack('!I', addr) + bytes([size])
    result = uds.request(payload)

    if result[0] == "OK":
      resp = result[1]
      print(f"  0x{addr:08X}     {label:<28} OK  {resp[1:].hex()}")
    elif result[0] == "NRC":
      code = result[1]
      print(f"  0x{addr:08X}     {label:<28} NRC 0x{code:02x} = {nrc_name(code)}")
    else:
      print(f"  0x{addr:08X}     {label:<28} {result[0]}")


def main():
  print("=" * 70)
  print("EyeSight ECU Reconnaissance")
  print(f"Target: TX=0x{EYESIGHT_TX:03X} RX=0x{EYESIGHT_RX:03X} Bus={BUS}")
  print("=" * 70)

  print("\nConnecting to panda...")
  p = Panda()

  p.set_safety_mode(SAFETY_ALLOUTPUT)
  print(f"Panda: {p.get_serial()[0]}")

  p.can_clear(0xFFFF)
  time.sleep(0.2)

  uds = SimpleUDS(p, EYESIGHT_TX, EYESIGHT_RX, BUS, timeout=0.5)

  print("\nEstablishing session...")
  result = uds.tester_present()
  print(f"  Tester Present: {result[0]} {result[1].hex() if result[0] == 'OK' else result[1] if result[0] == 'NRC' else ''}")

  result = uds.enter_session(0x03)
  if result[0] != "OK":
    print(f"  Extended Session: FAILED - {result}")
    print("\nECU not responding. Is the car key-on?")
    return
  print(f"  Extended Session: OK")

  phase1_services(uds)
  readable, denied = phase1_dids(uds)

  uds.enter_session(0x03)
  time.sleep(0.1)
  phase2_security(uds)

  uds.enter_session(0x03)
  time.sleep(0.1)
  phase3_memory(uds)

  print("\n" + "=" * 70)
  print("DONE")
  print("=" * 70)

  p.set_safety_mode(SAFETY_SILENT)

if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    print("\nInterrupted. Resetting panda...")
    try:
      p = Panda()
      p.set_safety_mode(SAFETY_SILENT)
    except:
      pass
