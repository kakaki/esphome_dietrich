#!/usr/bin/env python3
"""Detect which De Dietrich / Remeha serial protocol a boiler speaks.

Sends the "sample" request of both protocol variants supported by the ESPHome
component (``mcr3`` and ``calenta_v1_p5``) over a USB-TTL serial port, prints
the raw reply, validates it and decodes a few values so they can be compared
with what Recom shows.

Wiring
------
* ESP8266 board used as a USB-serial bridge (RST tied to GND): wire it exactly
  like for Recom - board TX -> boiler TX, board RX -> boiler RX, GND -> GND.
* Separate USB-TTL adapter: cross the data lines - adapter TX -> boiler RXD
  (RJ10 pin 2), adapter RX <- boiler TXD (pin 3), GND -> GND (pin 4).
Close Recom first: only one program can open the COM port.

Usage
-----
    pip install pyserial
    python tools/detect_protocol.py COM3
    python tools/detect_protocol.py COM3 --repeat 5 --interval 3
    python tools/detect_protocol.py COM3 --listen 10   # only print what arrives
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is missing - install it with: pip install pyserial")


# --------------------------------------------------------------------------- #
# Checksums
# --------------------------------------------------------------------------- #
def crc16_modbus(data: bytes) -> int:
    """CRC16, poly 0xA001, init 0xFFFF (used by the mcr3 variant)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def xor_checksum(data: bytes) -> int:
    """XOR of all bytes (used by the calenta_v1_p5 variant)."""
    x = 0
    for b in data:
        x ^= b
    return x


def mcr3_frame_ok(f: bytes) -> bool:
    # STX ... CRC_L CRC_H ETX, CRC over bytes 1..n-4
    if len(f) < 5 or f[0] != 0x02 or f[-1] != 0x03:
        return False
    return crc16_modbus(f[1:-3]) == (f[-3] | (f[-2] << 8))


def calenta_frame_ok(f: bytes) -> bool:
    # STX ... XOR ETX, XOR over bytes 1..n-3
    if len(f) < 4 or f[0] != 0x02 or f[-1] != 0x03:
        return False
    return xor_checksum(f[1:-2]) == f[-2]


# --------------------------------------------------------------------------- #
# Protocol definitions (same requests as components/dietrich/dietrich.cpp)
# --------------------------------------------------------------------------- #
PROTOCOLS = {
    "mcr3": {
        "request": bytes([0x02, 0xFE, 0x01, 0x05, 0x08, 0x02, 0x01, 0x69, 0xAB, 0x03]),
        "valid": mcr3_frame_ok,
        "offset": 0,
    },
    "calenta_v1_p5": {
        "request": bytes([0x02, 0x52, 0x05, 0x06, 0x02, 0x00, 0x53, 0x03]),
        "valid": calenta_frame_ok,
        "offset": -1,
    },
}

# Sanity check of the request frames themselves
assert mcr3_frame_ok(PROTOCOLS["mcr3"]["request"])
assert calenta_frame_ok(PROTOCOLS["calenta_v1_p5"]["request"])


# --------------------------------------------------------------------------- #
# Serial helpers
# --------------------------------------------------------------------------- #
def hexs(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def read_reply(port: serial.Serial, timeout: float = 1.0, gap: float = 0.1) -> bytes:
    """Read until no byte arrived for `gap` seconds (or `timeout` overall)."""
    buf = bytearray()
    start = last = time.monotonic()
    while True:
        n = port.in_waiting
        if n:
            buf += port.read(n)
            last = time.monotonic()
        now = time.monotonic()
        if buf and now - last > gap:
            break
        if now - start > timeout:
            break
        time.sleep(0.005)
    return bytes(buf)


def query(port: serial.Serial, request: bytes) -> bytes:
    port.reset_input_buffer()
    port.write(request)
    port.flush()
    reply = read_reply(port)
    # A bridge that echoes our own TX back (half-duplex wiring) shows up as the
    # request at the start of the reply - strip it so validation still works.
    if reply.startswith(request):
        print("    (request echoed back - TX and RX are tied together?)")
        reply = reply[len(request):]
    return reply


# --------------------------------------------------------------------------- #
# Decoding (offsets from components/dietrich/dietrich.cpp)
# --------------------------------------------------------------------------- #
def s16(lo: int, hi: int) -> int:
    v = lo | (hi << 8)
    return v - 0x10000 if v >= 0x8000 else v


def temp(f: bytes, i: int) -> str:
    if i + 1 >= len(f):
        return "n/a"
    raw = f[i] | (f[i + 1] << 8)
    if raw == 0x8000:
        return "open"
    return f"{s16(f[i], f[i + 1]) * 0.01:.2f} °C"


def byte(f: bytes, i: int) -> str:
    return str(f[i]) if 0 <= i < len(f) else "n/a"


def decode(f: bytes, o: int) -> None:
    print("    Decoded values (compare with Recom):")
    print(f"      flow temp        : {temp(f, 7 + o)}")
    print(f"      return temp      : {temp(f, 9 + o)}")
    print(f"      DHW in temp      : {temp(f, 11 + o)}")
    print(f"      outside temp     : {temp(f, 13 + o)}")
    print(f"      calorifier temp  : {temp(f, 15 + o)}")
    print(f"      boiler ctrl temp : {temp(f, 19 + o)}")
    print(f"      room temp        : {temp(f, 21 + o)}")
    print(f"      CH setpoint      : {temp(f, 23 + o)}")
    print(f"      DHW setpoint     : {temp(f, 25 + o)}")
    if 32 + o < len(f):
        print(f"      fan speed        : {s16(f[31 + o], f[32 + o])} rpm")
    if 33 + o < len(f):
        print(f"      ionisation       : {f[33 + o] * 0.1:.1f} uA")
    print(f"      state            : {byte(f, 47 + o)}")
    print(f"      lockout          : {byte(f, 48 + o)}")
    print(f"      blocking         : {byte(f, 49 + o)}")
    print(f"      sub state        : {byte(f, 50 + o)}")
    if 56 < len(f):
        print(f"      water pressure   : {f[56] * 0.1:.1f} bar")


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def listen(port: serial.Serial, seconds: float) -> None:
    print(f"Listening for {seconds:.0f} s without sending anything...")
    end = time.monotonic() + seconds
    got = False
    while time.monotonic() < end:
        data = read_reply(port, timeout=0.5)
        if data:
            got = True
            print(f"  <<< {hexs(data)}")
    if not got:
        print("  nothing received")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="serial port, e.g. COM3 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--repeat", type=int, default=1, help="query each protocol N times")
    ap.add_argument("--interval", type=float, default=2.0, help="seconds between rounds")
    ap.add_argument("--listen", type=float, metavar="SECONDS", help="only listen, do not send")
    args = ap.parse_args()

    try:
        port = serial.Serial(args.port, args.baud, bytesize=8, parity="N", stopbits=1, timeout=0)
    except serial.SerialException as e:
        print(f"Cannot open {args.port}: {e}")
        print("Is Recom (or another program) still using the port?")
        return 2

    with port:
        time.sleep(0.2)  # let the adapter settle
        port.reset_input_buffer()

        if args.listen:
            listen(port, args.listen)
            return 0

        results = {name: 0 for name in PROTOCOLS}
        any_bytes = False

        for rnd in range(args.repeat):
            if args.repeat > 1:
                print(f"=== round {rnd + 1}/{args.repeat} ===")
            for name, proto in PROTOCOLS.items():
                print(f"[{name}]")
                print(f"  >>> {hexs(proto['request'])}")
                reply = query(port, proto["request"])
                if not reply:
                    print("  <<< (no reply)")
                    continue
                any_bytes = True
                print(f"  <<< {hexs(reply)}  ({len(reply)} bytes)")
                if proto["valid"](reply):
                    results[name] += 1
                    print("    checksum OK")
                    decode(reply, proto["offset"])
                else:
                    print("    checksum/frame INVALID")
                time.sleep(0.3)
            if rnd + 1 < args.repeat:
                time.sleep(args.interval)

    print()
    print("=== Result ===")
    for name, ok in results.items():
        print(f"  {name:15s}: {ok}/{args.repeat} valid replies")

    working = [n for n, ok in results.items() if ok]
    if working:
        print(f"\nUse in ESPHome:  variant: {working[0]}" + ("  (mcr3 is the default)" if working[0] == "mcr3" else ""))
        return 0
    if any_bytes:
        print("\nThe boiler answers, but not with a frame either variant understands.")
        print("Paste the hex above - the protocol may need a new variant.")
    else:
        print("\nNo byte received at all. Check:")
        print("  - TX/RX: ESP-as-bridge = straight (like Recom); separate adapter = crossed")
        print("  - GND connected between adapter and boiler")
        print("  - Recom closed, correct COM port")
    return 1


if __name__ == "__main__":
    sys.exit(main())
