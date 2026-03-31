# Subaru ECU Research Findings

**Date:** 2026-03-30 / 2026-03-31
**Vehicle:** 2026 Subaru Forester (VIN: 4S4SLDB67T3014873) with comma 3x
**Platform:** Gen2 Global, LKAS_ANGLE

## Critical Discovery: EyeSight CommunicationControl Blocked While Engine Running

The 2026 Forester's EyeSight ECU (0x787) **rejects UDS CommunicationControl (0x28) with conditionsNotCorrect (0x22) whenever the engine is running.** This is a deliberate safety design change in newer EyeSight variants.

### Evidence

| Condition | Extended Session (0x10 0x03) | CommControl (0x28 0x03 0x01) |
|-----------|------------------------------|-------------------------------|
| Engine OFF, openpilot stopped | OK | **OK** |
| Engine OFF, openpilot running | OK | **OK** |
| Engine ON, openpilot stopped | OK | **NRC 0x22** |
| Engine ON, openpilot running | OK | **NRC 0x22** |
| Engine ON, any safety mode | OK | **NRC 0x22** |
| Engine ON, after 200ms delay | OK | **NRC 0x22** |
| Engine ON, 3 retries at 0.5s | OK | **NRC 0x22 x3** |

Tested all CommunicationControl subfunctions while engine running:
- 0x03 (DISABLE_RX_DISABLE_TX): **conditionsNotCorrect (0x22)** - only implemented option
- 0x83 (same + suppress response): **conditionsNotCorrect (0x22)**
- 0x01 (ENABLE_RX_DISABLE_TX): subFuncNotSupported (0x12)
- 0x00, 0x02: subFuncNotSupported (0x12)
- All NM/NORMAL_AND_NM variants: not supported

### What This Means for Alpha Long

The existing openpilot approach (disable_ecu via UDS CommunicationControl) **cannot work** on the 2026 Forester while the engine is running. This is specific to LKAS_ANGLE EyeSight variants - older Gen2 Subarus accept the disable command with engine running.

Without UDS disable, relay blocking triggers `relayMalfunction` because the panda detects both stock EyeSight messages and openpilot replacement messages on the same bus.

### Approaches That Won't Work
1. **Timing delays** (12s sleep, 200ms gap) - the condition is engine state, not timing
2. **Retry loops** - consistently fails while engine running
3. **Different safety modes** (allOutput, elm327, subaru) - fails in all modes
4. **Relay-only blocking** - triggers relayMalfunction detection
5. **Different CommunicationControl subfunctions** - only 0x03 is implemented

### Remaining Options
1. **Hardware relay isolation** - physically block EyeSight, modify relayMalfunction detection
2. **Work WITH EyeSight** - send ACC target commands to EyeSight instead of replacing it (stretch goal)
3. **Pre-engine disable** - modify startup flow to disable EyeSight before engine starts (requires changes to card.py fingerprinting flow)

## Other Findings

### Panda Safety Mode and Bus 2

- **elm327 mode**: bus 0 <-> bus 2 forwarding is ACTIVE. UDS on bus 2 works.
- **subaru mode**: ECU goes completely SILENT on bus 2. No UDS responses at all.
- **allOutput (param=0)**: forwarding DISABLED. UDS on bus 2 works.
- **allOutput (param=1)**: forwarding ENABLED. UDS on bus 2 works.

The panda's subaru safety mode prevents receiving UDS responses on bus 2. This means disable_ecu MUST run before the panda switches to subaru mode.

### card.py Init Ordering

```
init() runs -> disable_ecu attempts -> ControlsReady set -> pandad switches safety mode
```

`init()` and `ControlsReady` happen in the same card.py cycle. The safety mode switch happens ~150ms later when pandad processes the param. The TODO in card.py acknowledges this: "this can make us miss at least a few cycles when doing an ECU knockout."

### SSM4 Database Decryption (Cracked)

Fully cracked the SSM4 XML database encryption:
- **Algorithm:** AES-256-CBC
- **Key derivation:** SHA1(mixed_copyright_strings) -> HMAC expansion to 32 bytes
- **Password:** XOR/ADD mix of "Copyright (C) DENSO CORPORATION..." and "Copyright (C) Fuji Heavy Industries Ltd..."
- **IV:** `a8b0c8c96f9bafb8bec2c2a08985b48c` (from modified buffer)
- **Universal:** same key/IV works across all SSM4 versions

Extracted 32 AES security access keys from SSM4 26.6. Keys are variant-specific and the 2026 Forester's variants are not in the 2021 database. SSM5 database needed for newer variants.

### ECU Map (Bus 1)

| Addr | ECU | System | DID 0x1020 | Part |
|------|-----|--------|-----------|------|
| 0x730 | Unknown | - | - | - |
| 0x764 | Unknown | - | - | - |
| 0x785 | Monocular Camera | SLS10 | ffff0001 | 87542SL00A |
| 0x7A2 | Engine | 2.5 DOHC | 50000019 | 22768AA29A |
| 0x7A3 | CVT Transmission | CVT | 00000001 | 30921AA170 |
| 0x7B0 | ABS/VDC | VDC/Parking Brake | 9ffff801 | 27522SL00A |
| 0x7E0 | Engine (legacy) | - | - | - |

### EyeSight ECU (Bus 2)

| Property | Value |
|----------|-------|
| Address | 0x787 (RX: 0x78F) |
| Part | 87501SL00A |
| F100 | 20020e |
| F182 | 1821080142132108013D |
| F197 | EyeSight |
| DID 0x1020 | 00000001 |
| Security Access | 3 levels (0x01, 0x03, 0x05), 128-bit seeds, strong RNG |
| ReadMemoryByAddress | NOT SUPPORTED (0x11) |
| Routine Control | No valid IDs found |

### Security Access Algorithm

SSM4/SSM5 era: `key = AES-128-ECB(seed, per_variant_aes_key)`
- Variant determined by DID 0xF182
- Key table in SSM4 `CVsmRsKeyTableAES` (encrypted XML database)
- 2026 Forester variants not in SSM4 26.6 (too old)
- ECU locks out completely (no UDS response) after 1 wrong key attempt

### Files and Scripts

**On comma at `/data/openpilot/`:**
- `eyesight_recon.py` - Full service/DID/security scan
- `eyesight_seed_collect.py` - Seed collection and analysis
- `bus0_ecu_scan.py` - Bus 0/1 ECU discovery
- `probe_7a3.py` - CVT deep probe
- `probe_eyesight_timing.py` - Startup timing test
- `test_disable_modes.py` - Safety mode comparison
- `test_fwd_disable.py` - Forwarding vs disable test
- `test_comm_ctrl.py` - CommunicationControl subfunction scan
- `clear_all_dtcs.py` / `fix_faults.py` - DTC clearing
- `read_dids.py` - Variant DID reader
- `try_keys.py` - AES key attempt tool

**On host:**
- `subaru_ecu_research.md` - This file
- `eyesight_recon_findings.md` - EyeSight recon details
- `unlockECU_comment.md` - Draft GitHub comment for UnlockECU Issue #25
- `~/Documents/Development/op/UnlockECU/` - UnlockECU repo with VsmDataLib DLLs
- `~/Documents/Development/op/ssm4_vm/` - SSM4 26.6 VM image
- `~/Documents/Development/op/subaru_service_manuals/` - 2019 Legacy/Outback service manual (370MB)

**Subaru techinfo.subaru.com** - 72hr subscription active until ~2026-04-03. Service manual for 2019 Legacy/Outback downloaded.
