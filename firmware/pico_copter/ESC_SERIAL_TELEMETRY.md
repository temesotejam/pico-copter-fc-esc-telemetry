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

While motors are running, each ESC is requested in order FR, FL, RR, RL.
One DShot throttle frame per request sets its telemetry request bit, while
preserving the commanded throttle; other motors get normal DShot frames.
The request rotates every 40 ms (25 responses/s maximum total, about 6.25 Hz
per motor). UART1 at 115200 bps, 8N1 receives the 10-byte KISS/BLHeli_32
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
USB reports `#ESC_UART FR/FL/RR/RL ...` and a `#ESC_UART SUM` line.
`valid=0` means no fresh data; it is **not** a zero measurement.
An unpowered ESC or a nonresponding/unsupported TX terminal produces
timeouts/invalid telemetry rather than a change to flight-control output.

**CH5 recorder remains exactly 38 fields** and is not modified in this first
telemetry version. The displayed current is ESC-reported, not independently
calibrated. Consumption is each ESC's own cumulative mAh (not watts).

## Initial safety tests

1. Remove propellers; first connect just one verified-level TX terminal.
2. Build and flash the Pico UF2, verify original DShot motor operation.
3. Gently run the connected motor and inspect `#ESC_UART` on USB.
4. Confirm `requests` increments, `received` increments, and `valid=1`.
   No response: inspect TX wiring, UART voltage, ESC telemetry firmware,
   and `timeouts/crc` without altering control logic.
5. Only after successful one-ESC operation, verify electrical compatibility
   and connect multiple ESC TX lines. Test each individually and then in
   combination, checking mapping, current, and temperature.
6. Verify that SBUS failsafe still issues DShot STOP with props removed.

Sources: BLHeli_32 KISS telemetry framing (Betaflight esc_sensor.c),
Betaflight's DShot ESC telemetry documentation, and the MAD product
specification. Hardware behavior of this exact ESC TX pad must be validated.
