"""
servo_test.py - first test for Hiwonder HTD-45H bus servos through a BusLinker board.

Setup (once):   pip3 install pyserial
Run:            python3 servo_test.py                 (auto-finds the board)
                python3 servo_test.py /dev/cu.XXXX    (or give the port yourself)

What it does: finds servos with IDs 1-3, prints position / voltage / temperature,
then gently wiggles each one about 24 degrees and back.
Before running: set the servo IDs to 1, 2, 3 in Bus Servo Terminal, nothing attached to the horns.
"""
import glob
import sys
import time

import serial

BAUD = 115200

# Hiwonder / LewanSoul bus servo command numbers
MOVE_TIME_WRITE = 1   # params: position (0-1000), time (ms)
TEMP_READ = 26        # reply: temperature in C
VIN_READ = 27         # reply: voltage in mV
POS_READ = 28         # reply: position (0-1000, signed)
LOAD_UNLOAD = 31      # param: 0 = motor off (limp), 1 = motor on (holds position)

UNITS_PER_DEG = 1000 / 240  # 0-1000 maps to 0-240 degrees


def checksum(body):
    return (~sum(body)) & 0xFF


class ServoBus:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.05)

    def _send(self, sid, cmd, params=()):
        body = [sid, len(params) + 3, cmd, *params]
        self.ser.reset_input_buffer()
        self.ser.write(bytes([0x55, 0x55, *body, checksum(body)]))

    def _read_reply(self, sid, cmd, timeout=0.2):
        """Scan incoming bytes for a valid reply frame from servo `sid` for `cmd`.
        Skips the echo of our own request, which the single-wire bus can send back."""
        buf = b""
        end = time.time() + timeout
        while time.time() < end:
            buf += self.ser.read(64)
            i = 0
            while i + 6 <= len(buf):
                if buf[i] == 0x55 and buf[i + 1] == 0x55:
                    length = buf[i + 3]
                    frame_end = i + 2 + length  # index of the checksum byte
                    if frame_end >= len(buf):
                        break  # wait for the rest of the frame
                    body = list(buf[i + 2:frame_end])
                    ok = checksum(body) == buf[frame_end]
                    if ok and (sid is None or body[0] == sid) and body[2] == cmd and length > 3:
                        return body[3:]  # parameter bytes
                    i = frame_end + 1 if ok else i + 1
                else:
                    i += 1
        return None

    def move(self, sid, pos, time_ms=1000):
        pos = max(0, min(1000, int(pos)))
        t = max(0, min(30000, int(time_ms)))
        self._send(sid, MOVE_TIME_WRITE, [pos & 0xFF, pos >> 8, t & 0xFF, t >> 8])

    def read_pos(self, sid):
        self._send(sid, POS_READ)
        p = self._read_reply(sid, POS_READ)
        if p is None:
            return None
        val = p[0] | (p[1] << 8)
        return val - 65536 if val > 32767 else val

    def read_vin(self, sid):
        self._send(sid, VIN_READ)
        p = self._read_reply(sid, VIN_READ)
        return None if p is None else (p[0] | (p[1] << 8)) / 1000.0

    def read_temp(self, sid):
        self._send(sid, TEMP_READ)
        p = self._read_reply(sid, TEMP_READ)
        return None if p is None else p[0]

    def motor(self, sid, on):
        self._send(sid, LOAD_UNLOAD, [1 if on else 0])

    def release(self, ids=(1, 2, 3)):
        """Motors off so the servos go limp and stop drawing current. Call this when a script is done
        (project rule: never leave servos holding after a run; a limp servo re-engages on the next move)."""
        for sid in ids:
            self.motor(sid, False)
        print("Motors off (servos limp).")


def find_port():
    candidates = (glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.wchusbserial*")
                  + glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyUSB*"))
    if not candidates:
        sys.exit("No board found. Check the USB cable (data, not charge-only) and run: ls /dev/cu.*")
    return candidates[0]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    print(f"Using {port}")
    bus = ServoBus(port)

    found = []
    for sid in (1, 2, 3):
        pos = bus.read_pos(sid)
        if pos is None:
            print(f"ID {sid}: no reply")
            continue
        found.append((sid, pos))
        print(f"ID {sid}: position {pos} (~{pos / UNITS_PER_DEG:.0f} deg), "
              f"{bus.read_vin(sid)} V, {bus.read_temp(sid)} C")

    if not found:
        sys.exit("No servos answered. Is the 12 V supply on? Are the IDs set to 1-3?")

    input("\nPress Enter to wiggle each servo ~24 degrees and back (keep fingers clear)...")
    for sid, start in found:
        target = start + 100 if start < 900 else start - 100
        print(f"Moving ID {sid}: {start} -> {target} -> {start}")
        bus.move(sid, target, 1000)
        time.sleep(1.3)
        bus.move(sid, start, 1000)
        time.sleep(1.3)
        print(f"  now at {bus.read_pos(sid)}, {bus.read_temp(sid)} C")

    bus.release([sid for sid, _ in found])
    print("\nDone.")


if __name__ == "__main__":
    main()
