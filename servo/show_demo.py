"""
show_demo.py - exaggerated motion-profile show for the three bare servos (horns on, no leg).

Run:  python3 show_demo.py [loops]        default 3 loops, ~30 s each
Stop: Ctrl-C (re-centers, then motors off)

Profiles are deliberately cartoonish: long slow wind-ups, a whip in the middle,
elastic overshoot, bounces, and a rippling sine wave that speeds up. Everything is
centered on the reference positions in leg_cal.json and stays within about +-100 deg of them (overshoot included).
Tape the servos down first.
"""
import math
import sys
import time

import profile_demo as pd
import servo_test as st
from leg_ik import load_cal, UNITS_PER_DEG

RATE = 50
DT = 1 / RATE
MAX_DEG_S = 300        # just under the servo's ~330 deg/s; durations stretch to respect it


# ---- profiles: u in 0..1 -> s (fraction of the move, may overshoot) ----
def extreme_ease(u):
    """Very slow start and end, sudden whip through the middle."""
    a, b = u ** 4, (1 - u) ** 4
    return a / (a + b)

def anticipate(u):
    """Pull back 20% first, then whip forward (cartoon anticipation)."""
    if u < 0.3:
        return -0.2 * math.sin(math.pi * u / 0.3 / 2)
    v = (u - 0.3) / 0.7
    return -0.2 + 1.2 * extreme_ease(v)

def elastic(u):
    """Overshoot and ring down like a spring."""
    if u >= 1:
        return 1.0
    return 1 - math.exp(-6 * u) * math.cos(2 * math.pi * 1.5 * u)

def bounce(u):
    """Drop and bounce three times."""
    n1, d1 = 7.5625, 2.75
    if u < 1 / d1:
        return n1 * u * u
    if u < 2 / d1:
        u -= 1.5 / d1
        return n1 * u * u + 0.75
    if u < 2.5 / d1:
        u -= 2.25 / d1
        return n1 * u * u + 0.9375
    u -= 2.625 / d1
    return n1 * u * u + 0.984375

def crawl_then_slam(u):
    """70% of the time crawls through the first 25%, then slams the rest."""
    if u < 0.7:
        return 0.25 * (u / 0.7)
    return 0.25 + 0.75 * extreme_ease((u - 0.7) / 0.3)


def safe_duration(profile, travel_deg, wanted_s):
    """Stretch the move so the steepest part of the profile stays under MAX_DEG_S."""
    peak = max(abs(profile((k + 1) / 400) - profile(k / 400)) * 400 for k in range(400))
    return max(wanted_s, peak * abs(travel_deg) / MAX_DEG_S)


class Show:
    def __init__(self, bus, ref):
        self.bus, self.ref = bus, ref          # ref: {sid: center position}
        self.deg = {sid: 0.0 for sid in ref}   # current offset from center, deg

    def _send(self, offsets):
        for sid, d in offsets.items():
            self.bus.load_move(sid, self.ref[sid] + d * UNITS_PER_DEG, DT * 1000)
        self.bus.start_all()

    def move(self, targets, secs, profile, label):
        """targets: {sid: offset deg from center}. Streams a profiled move."""
        travel = max(abs(targets[s] - self.deg[s]) for s in targets)
        secs = safe_duration(profile, travel, secs)
        print(f"  {label:28s} {travel:4.0f} deg in {secs:.2f} s")
        starts = dict(self.deg)
        steps = max(1, int(secs * RATE))
        t0 = time.perf_counter()
        for k in range(1, steps + 1):
            s = profile(k / steps)
            self._send({sid: starts[sid] + (targets[sid] - starts[sid]) * s for sid in targets})
            while time.perf_counter() < t0 + k * DT:
                pass
        self.deg.update(targets)

    def ripple(self, amp, f_start, f_end, secs, label):
        """Sine wave on all servos, 120 deg phase apart, frequency ramping up."""
        print(f"  {label:28s} +-{amp:.0f} deg, {f_start}->{f_end} Hz for {secs} s")
        sids = sorted(self.ref)
        t0 = time.perf_counter()
        phase = 0.0
        k = 0
        while True:
            t = k * DT
            if t > secs:
                break
            f = f_start + (f_end - f_start) * t / secs
            phase += 2 * math.pi * f * DT
            self._send({sid: amp * math.sin(phase + i * 2 * math.pi / 3) for i, sid in enumerate(sids)})
            k += 1
            while time.perf_counter() < t0 + k * DT:
                pass
        # settle back to center smoothly
        self.deg = {sid: amp * math.sin(phase + i * 2 * math.pi / 3) for i, sid in enumerate(sids)}
        self.move({sid: 0 for sid in sids}, 1.0, pd.minjerk, "settle")

    def hold(self, secs):
        time.sleep(secs)


def main():
    loops = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    cal = load_cal()
    ref = {j.sid: j.ref_pos for j in cal.values()}
    bus = pd.SyncBus(st.find_port())
    for sid in ref:
        if bus.read_pos(sid) is None:
            sys.exit(f"ID {sid} not answering. Is the 12 V on?")
    show = Show(bus, ref)
    all_ = lambda d: {sid: d for sid in ref}

    print("centering...")
    show.deg = {sid: (bus.read_pos(sid) - ref[sid]) / UNITS_PER_DEG for sid in ref}
    show.move(all_(0), 1.5, pd.minjerk, "center")
    show.hold(0.5)

    try:
        for n in range(loops):
            print(f"\n--- loop {n + 1}/{loops} ---")
            show.move(all_(+75), 2.5, anticipate, "anticipation whip +75")
            show.hold(0.6)
            show.move(all_(-75), 2.5, elastic, "elastic snap to -75")
            show.hold(0.6)
            show.move(all_(+75), 4.0, crawl_then_slam, "crawl... then SLAM")
            show.hold(0.8)
            show.move(all_(-75), 3.0, bounce, "drop and bounce")
            show.hold(0.6)
            show.move({1: +75, 2: -75, 3: +75}, 2.0, extreme_ease, "opposite whip")
            show.hold(0.4)
            show.move({1: -75, 2: +75, 3: -75}, 2.0, extreme_ease, "and back")
            show.hold(0.6)
            show.move(all_(0), 1.0, pd.minjerk, "center")
            show.ripple(80, 0.15, 0.45, 8.0, "ripple wave, speeding up")
            show.hold(0.8)
    except KeyboardInterrupt:
        print("\nstopped")
    show.move(all_(0), 1.5, pd.minjerk, "center")
    bus.release(ref)
    print("done")


if __name__ == "__main__":
    main()
