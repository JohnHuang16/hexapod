"""
profile_demo.py - move all three servos at the same time with smooth motion profiles.

Run:  python3 profile_demo.py            (default: 60 deg swings, 0.5 s each)
      python3 profile_demo.py 90 0.4     (bigger and faster: degrees, seconds)
Keep servo_test.py in the same folder (this reuses it). Nothing attached to the horns yet.

What it shows:
  1. All three joints moving together, starting and stopping on the same instant.
  2. The same move done three ways so you can see/feel the difference:
       linear  - constant speed, abrupt start/stop
       smooth  - "smoothstep" ease in/out (3u^2 - 2u^3)
       minjerk - minimum-jerk ease (10u^3 - 15u^4 + 6u^5), the smoothest
"""
import time

import servo_test as st

IDS = (1, 2, 3)
RATE_HZ = 50                 # setpoints per second
DT = 1.0 / RATE_HZ
MOVE_TIME_WAIT_WRITE = 7     # load a move but don't start it yet
MOVE_START = 11              # start all loaded moves
BROADCAST = 254


# ---- motion profiles: u goes 0 -> 1 over the move, s(u) goes 0 -> 1 ----
def linear(u):
    return u


def smoothstep(u):
    return 3 * u**2 - 2 * u**3


def minjerk(u):
    return 10 * u**3 - 15 * u**4 + 6 * u**5


PROFILES = {"linear": linear, "smooth": smoothstep, "minjerk": minjerk}


class SyncBus(st.ServoBus):
    def load_move(self, sid, pos, time_ms):
        pos = max(0, min(1000, int(round(pos))))
        t = max(0, int(time_ms))
        self._send(sid, MOVE_TIME_WAIT_WRITE, [pos & 0xFF, pos >> 8, t & 0xFF, t >> 8])

    def start_all(self):
        self._send(BROADCAST, MOVE_START)

    def move_together(self, targets, duration_s, profile=minjerk):
        """targets: {id: goal position}. Streams a synchronized profiled move."""
        starts = {sid: self.read_pos(sid) for sid in targets}
        steps = max(1, int(duration_s * RATE_HZ))
        t0 = time.perf_counter()
        for k in range(1, steps + 1):
            s = profile(k / steps)
            for sid, goal in targets.items():
                self.load_move(sid, starts[sid] + (goal - starts[sid]) * s, DT * 1000)
            self.start_all()
            # wait until the next tick (keeps a steady 50 Hz)
            next_tick = t0 + k * DT
            while time.perf_counter() < next_tick:
                pass


def main():
    import sys
    amp_deg = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0   # swing size
    move_s = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5     # time per swing
    amp_deg = max(5.0, min(90.0, amp_deg))
    move_s = max(0.3, move_s)  # faster than this exceeds the servo's top speed
    print(f"Swing: {amp_deg:.0f} deg in {move_s} s each way")
    bus = SyncBus(st.find_port())
    home = {}
    for sid in IDS:
        p = bus.read_pos(sid)
        if p is None:
            raise SystemExit(f"ID {sid} not answering. Check power and chain.")
        home[sid] = p
    print("Start positions:", home)

    input("Press Enter for a synchronized move with each profile (keep fingers clear)...")
    offset = int(amp_deg * st.UNITS_PER_DEG)
    out = {sid: min(1000, max(0, home[sid] + offset)) for sid in IDS}
    back = {sid: min(1000, max(0, home[sid] - offset)) for sid in IDS}

    for name, prof in PROFILES.items():
        input(f"\nPress Enter for {name.upper()}: 3 fast back-and-forth swings, no pauses...")
        for _ in range(3):
            bus.move_together(out, move_s, prof)
            bus.move_together(back, move_s, prof)
        bus.move_together(home, move_s, prof)

    input("\nPress Enter for a wave (joints swing in opposite directions together)...")
    w = int(offset * 0.8)
    for _ in range(4):
        bus.move_together({1: home[1] + w, 2: home[2] - w, 3: home[3] + w}, move_s)
        bus.move_together({1: home[1] - w, 2: home[2] + w, 3: home[3] - w}, move_s)
    bus.move_together(home, move_s)

    print("Final:", {sid: bus.read_pos(sid) for sid in IDS})
    print("Done. Unplug the 12 V to relax the servos.")


if __name__ == "__main__":
    main()
