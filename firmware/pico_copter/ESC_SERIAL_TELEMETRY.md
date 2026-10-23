# MAD BLHeli_32 ESC — separate TX-pad UART telemetry

This feature is **experimental / hardware-unverified** and exists only in
`pico-copter-fc-esc-telemetry`. It does not replace or modify the PID, EKF,
400 Hz motor update, 100 Hz attitude control, CH5 RAM flight log, or SBUS
failsafe. It **does not** use serial telemetry for feedback or motor stopping.

## Wiring

- Each ESC's DShot signal input stays on its existing Pico motor GPIO:
  FR=15, FL=14, RR=3, RL=2.
- Connect ESC signal grounds and Pico ground together.
- ESC **dedicated TX / telemetry pad**, not motor signal pad, goes to
  Pico **GPIO5 (UART1 RX; physical pin 7)**.
- Four separate BLHeli_32 ESC TX wires may share the one RX line **only if**
  the particular ESCs have compatible shared-line telemetry electrical behavior.
  This is a common BLHeli_32 configuration, but confirm the output electrical
  specification. If in doubt, initially connect **one ESC TX only**; use
  appropriate isolation/multiplexing if the particular outputs cannot share.
- **Pico GPIO is NOT 5-V tolerant.** Measure the ESC TX idle and high
  voltage before connecting. Provide a compatible level shifter/divider or
  buffer when it exceeds Pico's 3.3-V GPIO domain. Do not connect TX to GPIO0
  or GPIO1, which are used by SBUS.
- No external wiring is needed from Pico UART1 TX (GPIO4).

## Acquisition and output

The standard build now uses **bidirectional DShot300** on the motor-control
lines. Its PIO RX path returns eRPM on the *same* DShot signal wire.
A channel is considered **connected** only after a GCR+checksum-valid eRPM
response (even eRPM=0 is valid). Its connected state expires if another
valid eRPM response is not seen within **250 ms**. CRC errors, GCR errors,
missing replies, UART garbage, and command transmission by themselves do
not mark a channel as connected.

**Continuous receive and automatic framing (2026-10-20 revision)**

The UART1 IRQ continuously scans every incoming byte, even while the motor is
stopped and even if the Pico has not sent a telemetry-request bit. A 2.5 ms
inter-byte gap clears partial data; a sliding 10-byte window resynchronizes
after shifted, dropped or extra bytes. A candidate passes the KISS CRC8
(poly 0x07), broad 8S ESC plausible bounds (3–50 V, temperature <=150 C,
current <=300 A, eRPM <=1,500,000), and **two successive consistent frames**
before the first measurement is exposed. Established frames must stay
consistent with recent voltage (within 3.5 V) and temperature (within 20 C).
This rejects obviously invalid values such as 250 C / 74.27 V from prior
logs. The limits are diagnostic heuristics, not proof of correct measurements.

**Exactly one bidirectional-DShot-detected ESC:** Incoming frames can be
assigned to that motor even without any DShot request bit. This is the
current **one-ESC RL=GPIO2 test**. While CRC-valid unsolicited UART packets
are present, the Pico **does not send redundant telemetry requests**.
If none has arrived for 250 ms and the sole motor has a nonzero command,
the 40-ms request-bit scheduler falls back to polling (for Auto=Off firmware).
DShot motor output still runs at 400 Hz without blocking in either case.

**Two or more detected ESCs:** KISS TX packets do not contain a motor ID.
Unsolicited packets on a shared UART wire cannot be attributed safely;
they are counted as `unassigned` and never treated as per-motor values.
When unsolicited packets are arriving, requests are paused instead of
possibly assigning the wrong ESC's packet. When no unsolicited stream has
arrived recently, the previous rotating 40-ms request scheduler is used.
The electrical compatibility of joining multiple ESC TX output pins is
NOT established; use a suitable multiplexer/separate UARTs if required.
**Do not connect multiple TX outputs together without checking specifications.**

UART1 at 115200 baud, 8N1 decodes the following fields:

- Temperature in °C
- Voltage in 0.01 V
- Current in 0.01 A
- Consumption in mAh
- Electrical RPM in multiples of 100 (mechanical RPM depends on pole pairs)

The IRQ never prints over USB and never waits for a response. Requests
time out in 8 ms; loss of ESC telemetry does **not** inhibit motor output.
Data older than 500 ms is marked invalid and excluded from current totals.

Enable/disable: set `esc_serial::kEnabled` in `esc_serial_uart.hpp`.
USB reports `#DShot300 FR/FL/RR/RL ... connected=0/1`,
`#ESC_UART FR/FL/RR/RL detected=0/1 ... auto_rx=... req_rx=... confirm=...
src=auto|request|none`, and `#ESC_UART SUM mode=stream|poll_or_wait
mask=... bytes=... plausible=... crc_windows=... range_reject=...
unassigned=... gaps=... no_esc_bytes=...`. The old `crc` field
counted one requested 10-byte block per request; `crc_windows` now counts
ALL scanned/shifted invalid 10-byte windows, so it is **not** a request failure
rate. For stable streaming, `auto_rx` should grow at about 30 frames/s.
`confirm` means a candidate was held for corroboration, not reported.
`range_reject` means a packet passed CRC but failed physical bounds.
`valid=0` means no fresh confirmed measurement; it is NOT zero.
An unpowered ESC or a nonresponding/unsupported TX terminal produces
timeouts/invalid telemetry rather than a change to flight-control output.

**CH5 recorder remains exactly 38 fields** and is not modified in this first
telemetry version. The displayed current is ESC-reported, not independently
calibrated. Consumption is each ESC's own cumulative mAh (not watts).

## Initial safety tests

1. Remove propellers. Verify ESC TX signal high voltage <=3.3 V and a common
   ground; first connect only **one** ESC TX terminal to Pico GPIO5.
2. Flash the new RP2040 UF2. Bidirectional DShot mode and motor control GPIOs
   remain unchanged.
3. Before arming, look for `#DShot300 RL connected=1` and
   `#ESC_UART RL auto_rx` growing. Some BLHeli proto firmware may
   automatically transmit a frame every ~32 ms; this hypothesis remains
   subject to experimental confirmation.
4. `#ESC_UART SUM mode=stream` indicates recent unsolicited valid data. In
   this mode `requests=0` is expected, not an error. Two consistent packets
   are needed before `valid=1`; when idle, `erpm=0` is valid.
5. Gently spin the motor and compare UART temperature, voltage, current and
   eRPM with bidirectional DShot eRPM. `valid` should stay 1 when healthy.
6. If no unsolicited stream exists, the request scheduler starts sending
   telemetry bits once a detected motor has a nonzero throttle command.
7. Check `bytes`, `auto_rx`, `req_rx`, `crc_windows`,
   `range_reject`, `unassigned`, `gaps` and `no_esc_bytes` separately.
   `crc_windows` is *not* the ratio of lost request replies.
8. Do NOT connect multiple TX outputs together without verifying their
   electrical behavior and preventing automatic-transmission collisions.
9. Verify SBUS failsafe still sends DShot STOP with no propellers.

The supplied RL experiment log showed genuine-looking 7.7–7.8 V / 0.4 A
packets mixed with abnormal 250 C / 74.27 V packets. Firmware build and
host parsing tests are not proof of reliable hardware reception. Obtain
a new log before evaluating this revision's success.


## Pico 2 hardware variant (experimental, October 2026)

The RP2040/Pico driver and its UART1 GP5 configuration remain unchanged.
For `PICO_BOARD=pico2`, a separate `pwm_pico2.cpp` driver is selected:

- SN74LVC245APWR A side=Pico2, B side=ESC; **GP18=DIR**, **GP19=/OE**.
  An external pull-up from /OE to 3.3 V is mandatory for safe boot/reset.
  Remove previous hardwired DIR and /OE straps before connecting MCU outputs.
- DShot channels: GP15, GP14, GP3, GP2, GP20, GP21. Existing flight mixer
  controls only the first four. **GP20/GP21 transmit STOP only**, not flight
  throttle, until a six-motor mixer and failsafe are developed.
- ESC dedicated TX-pad telemetry goes through its **own protected input**
  to GP17 (PIO2 UART RX at 115200 8N1), **not** through the shared DIR/OE
  transceiver. This input requires voltage compatibility / buffering as needed.
- Pico2 defaults to one-way DShot300; unlike the RP2040 build, it can request
  dedicated ESC TX-pad telemetry **without** eRPM/presence confirmation. It
  requests only for nonzero motor commands, serializes requests, and never
  gates motor output on telemetry.
- To test the shared-SN74LVC245A **bidirectional DShot300** path, build Pico2
  with `-DDSHOT_BIDIRECTIONAL=1`. PIO0+PIO1 motor streams and the PIO2
  DIR/OE controller are synchronously restarted. The PIO UART receiver remains
  continuously active. **Do not operate this mode before verifying DIR/OE
  waveforms, signal voltage, and bus contention on a scope without ESCs.**
  Rates other than 300 kbit/s are deliberately rejected for this experimental
  Pico2 bidirectional controller.
- UART TX-pad frames have no source ID. When multiple ESCs auto-transmit on
  the same wire, frame attribution is impossible and push-pull TX connections
  can electrically conflict. Do **not** tie unrelated TX outputs together
  without a suitable mux/isolation and an explicit source-identification
  scheme. `kFixedUartSourceMotor` may be configured only for a physically
  single-source telemetry connection. Request-scoped attribution assumes
  Auto Telemetry is off on all shared-line ESCs.

These changes are **source/build checks, not hardware validation**. No flight
with Pico2 should be attempted until protected single-ESC tests and
six-channel timing verification have passed.


### Pico 2: request-driven ESC TX telemetry (no bidirectional DShot required)

The target ESC sends a dedicated TX-pad KISS UART reply when the DShot
telemetry-request bit is set. Pico 2 therefore uses **request-only**
telemetry: the default one-way DShot300 sends requests to one active motor
(FR/FL/RR/RL) at a time, at least 40 ms apart. Only a complete reply
received within the 8 ms request window may be assigned to that motor.
Two consistent CRC-valid replies are needed before the UART data is shown
as fresh and the ESC is marked detected. RP2350 does not use bidirectional
DShot eRPM for this presence decision. The RP2040 path stays unchanged.

GP17 receives the UART TX-pad signal through its own protected input; it
does not pass through the common DIR/OE-controlled SN74LVC245A. GPIO20/21
remain STOP-only until a six-motor flight mixer is implemented.

**Electrical caution:** one-at-a-time DShot requests do not automatically
make several push-pull ESC TX pins safe to wire together. Verify the ESC
TX electrical output topology first; add per-ESC input isolation or a
controlled multiplexer if needed. Disable unsolicited/auto ESC telemetry
for request-window-based motor identification.
