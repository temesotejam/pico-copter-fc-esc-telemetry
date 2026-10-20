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

Only **connected motors with a nonzero throttle command** are selected for
TX-pad telemetry requests, in FR, FL, RR, RL rotation. Disconnected motors
still receive their original control/STOP commands at 400 Hz, and SBUS
failsafe behavior is unaffected. A serial telemetry request uses the DShot
telemetry request bit while preserving that motor's throttle command.
The next request can occur no sooner than 40 ms after the previous one
(25 responses/s maximum total). One connected ESC can be queried at up to
roughly 25 Hz; four connected ESCs roughly 6.25 Hz each. UART1 at 115200 bps, 8N1 receives the 10-byte KISS/BLHeli_32
serial reply, checks CRC8 (poly 0x07), and stores:

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
`#ESC_UART FR/FL/RR/RL detected=0/1 ...`, and a `#ESC_UART SUM` line.
`valid=0` means no fresh data; it is **not** a zero measurement.
An unpowered ESC or a nonresponding/unsupported TX terminal produces
timeouts/invalid telemetry rather than a change to flight-control output.

**CH5 recorder remains exactly 38 fields** and is not modified in this first
telemetry version. The displayed current is ESC-reported, not independently
calibrated. Consumption is each ESC's own cumulative mAh (not watts).

## Initial safety tests

1. Remove propellers; first connect just one verified-level TX terminal.
2. Build and flash the Pico UF2, verify original DShot motor operation.
3. Check `#DShot300 <motor> mode=bidir connected=1 valid=1` first.
   An unplugged ESC must remain `connected=0`. A recognized ESC with
   fresh reply should show eRPM even if its motor is not spinning yet.
4. Gently run the connected motor. Check `#ESC_UART <motor> detected=1`;
   then verify `requests`, `rx`, and `valid=1`. The other three ESCs,
   if not connected, should continue to show `requests=0`.
   If bidirectional DShot never reaches `connected=1`, check signal
   wiring/levels and ESC bidirectional support *before* diagnosing UART.
   If DShot is detected but UART fails, inspect TX wiring, voltage and
   `timeout/crc/stray` counters.
5. Only after successful one-ESC operation, verify electrical compatibility
   and connect multiple ESC TX lines. Test each individually and then in
   combination, checking mapping, current, and temperature.
6. Verify that SBUS failsafe still issues DShot STOP with props removed.

Sources: BLHeli_32 KISS telemetry framing (Betaflight esc_sensor.c),
Betaflight's DShot ESC telemetry documentation, and the MAD product
specification. Hardware behavior of this exact ESC TX pad must be validated. Current output
logs from the earlier one-way firmware showed no successful UART packets,
so no hardware success is claimed for this new bidirectional revision.
