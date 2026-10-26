"""
camera_demo.py - a choreographed servo show for filming.

Run everything:        python3 camera_demo.py
Run one segment only:  python3 camera_demo.py 4      (handy for retakes)

Segments:
  1 Wake up      - servos rise one after another, like a ripple
  2 In sync      - all three swing together, big and smooth
  3 Face-off     - same move, three profiles at once: 1 = linear, 2 = smooth, 3 = min-jerk
  4 Wave         - continuous wave travelling across the three servos
  5 Speed ramp   - swinging faster and faster, then easing off
  6 Robot mode   - sharp snaps between poses (stop-motion look)
  7 Bow + sleep  - slow bow, then motors switch off and go limp

Keep servo_test.py in the same folder. Tape the servos down: fast moves make them hop.
"""
import math
import sys
import time

import servo_test as st

IDS = (1, 2, 3)
RATE_HZ = 50
DT = 1.0 / RATE_HZ
CENTER = 500
DEG = st.UNITS_PER_DEG          # position units per degree
MOVE_TIME_WAIT_WRITE = 7
MOVE_START = 11
BROADCAST = 254


# ---------- motion profiles: u 0 -> 1, returns 0 -> 1 ----------
def linear(u):
    return u


def smooth(u):
    return 3 * u**2 - 2 * u**3


def minjerk(u):
    return 10 * u**3 - 15 * u**4 + 6 * u**5


class Bus(st.ServoBus):
    def load(self, sid, pos, time_ms):
        pos = max(0, min(1000, int(round(pos))))
        t = max(0, int(time_ms))
        self._send(sid, MOVE_TIME_WAIT_WRITE, [pos & 0xFF, pos >> 8, t & 0xFF, t >> 8])

    def go(self):
        self._send(BROADCAST, MOVE_START)

    def frame(self, positions, time_ms=DT * 1000):
        for sid, p in positions.items():
            self.load(sid, p, time_ms)
        self.go()


pose = {}  # last commanded position of each servo


def stream(bus, fn, duration):
    """Call fn(t) -> {id: position} 50 times a second for `duration` seconds."""
    t0 = time.perf_counter()
    k = 0
    while True:
        k += 1
        t = k * DT
        if t > duration + 1e-9:
            break
        target = fn(t)
        bus.frame(target)
        pose.update(target)
        while time.perf_counter() < t0 + t:
            pass


def glide(bus, goal, duration, profile=minjerk):
    start = dict(pose)
    stream(bus, lambda t: {i: start[i] + (goal[i] - start[i]) * profile(min(1.0, t / duration))
                           for i in goal}, duration)


def offsets(d1, d2, d3):
    """Pose from degree offsets around center."""
    return {1: CENTER + d1 * DEG, 2: CENTER + d2 * DEG, 3: CENTER + d3 * DEG}


HOME = offsets(0, 0, 0)


# ---------------------------- segments ----------------------------
def wake_up(bus):
    """Each servo lifts and settles, staggered, twice."""
    amp, length, stagger = 60 * DEG, 1.2, 0.35

    def bump(u):
        return math.sin(math.pi * u) ** 2 if 0 <= u <= 1 else 0.0

    for _ in range(2):
        stream(bus, lambda t: {i: CENTER + amp * bump((t - (i - 1) * stagger) / length) for i in IDS},
               length + 2 * stagger)


def in_sync(bus):
    for _ in range(3):
        glide(bus, offsets(70, 70, 70), 1.0)
        glide(bus, offsets(-70, -70, -70), 1.0)
    glide(bus, HOME, 1.0)


def face_off(bus):
    print("   servo 1 = LINEAR   servo 2 = SMOOTH   servo 3 = MIN-JERK")
    profiles = {1: linear, 2: smooth, 3: minjerk}
    for goal_deg in (70, -70, 70, -70, 0):
        start = dict(pose)
        goal = CENTER + goal_deg * DEG
        stream(bus, lambda t: {i: start[i] + (goal - start[i]) * profiles[i](min(1.0, t / 1.0))
                               for i in IDS}, 1.0)
        time.sleep(0.25)


def wave(bus):
    amp, period, cycles = 60 * DEG, 2.0, 4
    total = period * cycles

    def env(t):  # fade in and out so it starts and ends at center
        return min(1.0, t / period, (total - t) / period)

    stream(bus, lambda t: {i: CENTER + amp * env(t) *
                           math.sin(2 * math.pi * t / period - (i - 1) * 2 * math.pi / 3)
                           for i in IDS}, total)


def speed_ramp(bus):
    amp = 45 * DEG
    state = {"phase": 0.0}
    ramp_up, ramp_down = 6.0, 3.0

    def freq(t):
        if t < ramp_up:
            return 0.3 + 0.5 * (t / ramp_up)          # 0.3 Hz -> 0.8 Hz
        return 0.8 - 0.5 * ((t - ramp_up) / ramp_down)  # back down to 0.3 Hz

    def fn(t):
        state["phase"] += 2 * math.pi * freq(t) * DT
        p = CENTER + amp * math.sin(state["phase"])
        return {i: p for i in IDS}

    stream(bus, fn, ramp_up + ramp_down)
    glide(bus, HOME, 0.6)


def robot_mode(bus):
    poses = [(30, -30, 45), (-30, 30, -45), (0, 60, 0), (60, 0, -60),
             (-45, -45, -45), (45, 45, 45), (0, 0, 0)]
    for d in poses:
        target = offsets(*d)
        bus.frame(target, time_ms=80)   # as fast as the servo can snap
        pose.update(target)
        time.sleep(0.45)


def bow_and_sleep(bus):
    glide(bus, offsets(80, 80, 80), 1.5)
    time.sleep(0.6)
    glide(bus, offsets(-20, -20, -20), 2.0)
    time.sleep(0.4)
    for sid in IDS:
        bus._send(sid, st.LOAD_UNLOAD, [0])   # motors off: horns go limp
        time.sleep(0.15)


SEGMENTS = [
    ("Wake up", wake_up),
    ("In sync", in_sync),
    ("Face-off: linear vs smooth vs min-jerk", face_off),
    ("Wave", wave),
    ("Speed ramp", speed_ramp),
    ("Robot mode", robot_mode),
    ("Bow + sleep", bow_and_sleep),
]


def main():
    only = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else None
    bus = Bus(st.find_port())

    for sid in IDS:
        p = bus.read_pos(sid)
        if p is None:
            raise SystemExit(f"ID {sid} not answering. Check power and chain.")
        pose[sid] = p
        bus.motor(sid, True)

    print("Moving to center...")
    glide(bus, HOME, 2.0)
    input("Ready. Start recording, then press Enter...")
    for n in (3, 2, 1):
        print(n)
        time.sleep(1)

    show_start = time.perf_counter()
    for num, (name, fn) in enumerate(SEGMENTS, start=1):
        if only and num != only:
            continue
        print(f"[{time.perf_counter() - show_start:5.1f}s] {num}. {name}")
        fn(bus)
        if name != "Bow + sleep":
            glide(bus, HOME, 0.8)
        time.sleep(1.0)

    print(f"[{time.perf_counter() - show_start:5.1f}s] Done.")


if __name__ == "__main__":
    main()
