"""
leg_demo.py - choreographed demo for the assembled leg, in joint angles, inside leg_cal.json limits.

Run:  python3 leg_demo.py [loops]      default 1 loop (~60 s). Ctrl-C returns to standing, then motors off.

Every target goes through the calibration, so a move outside a joint's soft limits raises
RangeError before anything is powered. Angles: coxa 0 = straight out (+ toward the front),
femur 0 = horizontal (+ up), knee 0 = straight (+ folded). Joints without anything attached
are still driven, which is fine.
"""
import math
import sys
import time

import profile_demo as pd
import servo_test as st
from leg_ik import load_cal, LegAngles, RangeError, ik, STANDING

RATE, DT = 50, 0.02
STAND = ik(*STANDING)      # coxa 0, femur 21, knee 92


class Leg:
    def __init__(self, bus, cal):
        self.bus, self.cal = bus, cal
        self.ang = self.read()

    def read(self):
        pos = {j.sid: self.bus.read_pos(j.sid) for j in self.cal.values()}
        return LegAngles(*(self.cal[k].to_angle(pos[self.cal[k].sid]) for k in ("coxa", "femur", "knee")))

    def positions(self, a):
        return {self.cal[k].sid: self.cal[k].to_pos(getattr(a, k)) for k in ("coxa", "femur", "knee")}

    def send(self, a, ms=DT * 1000):
        for sid, p in self.positions(a).items():
            self.bus.load_move(sid, p, ms)
        self.bus.start_all()

    def move(self, coxa=None, femur=None, knee=None, secs=1.0, profile=pd.minjerk, label=""):
        target = LegAngles(self.ang.coxa if coxa is None else coxa,
                           self.ang.femur if femur is None else femur,
                           self.ang.knee if knee is None else knee)
        self.positions(target)                       # range check before moving
        if label:
            print(f"  {label}")
        start, steps = self.ang, max(1, int(secs * RATE))
        t0 = time.perf_counter()
        for k in range(1, steps + 1):
            s = profile(k / steps)
            self.send(LegAngles(start.coxa + (target.coxa - start.coxa) * s,
                                start.femur + (target.femur - start.femur) * s,
                                start.knee + (target.knee - start.knee) * s))
            while time.perf_counter() < t0 + k * DT:
                pass
        self.ang = target

    def stream(self, fn, secs, label=""):
        """fn(t) -> LegAngles, streamed at 50 Hz. Must start near the current pose."""
        if label:
            print(f"  {label}")
        t0 = time.perf_counter()
        k = 0
        while (t := k * DT) <= secs:
            a = fn(t)
            self.send(a)
            k += 1
            while time.perf_counter() < t0 + k * DT:
                pass
        self.ang = a

    def hold(self, secs):
        time.sleep(secs)


def routine(leg):
    S = STAND
    leg.move(S.coxa, S.femur, S.knee, 2.0, label="stand")
    leg.hold(0.8)

    # 1. look around: coxa sweeps while the femur lifts
    leg.move(femur=45, secs=1.5, label="lift")
    leg.move(coxa=+40, secs=1.2, label="look front")
    leg.hold(0.4)
    leg.move(coxa=-40, secs=1.8, label="look back")
    leg.hold(0.4)
    leg.move(coxa=0, femur=S.femur, secs=1.5, label="back to stand")
    leg.hold(0.6)

    # 2. stomps, each faster than the last
    for secs in (1.2, 0.9, 0.7, 0.5):
        leg.move(femur=55, secs=secs, profile=pd.smoothstep, label=f"stomp up ({secs} s)")
        leg.move(femur=-15, secs=secs, profile=pd.minjerk)
        leg.hold(0.25)
    leg.move(femur=S.femur, secs=1.0, label="recover")
    leg.hold(0.6)

    # 3. walking stride: coxa and femur 90 deg out of phase -> foot traces an ellipse. Speeds up.
    A_C, A_F, F0, F1, T = 28.0, 22.0, 0.25, 0.7, 12.0
    phase = [0.0]
    def stride(t):
        f = F0 + (F1 - F0) * t / T
        phase[0] += 2 * math.pi * f * DT
        ramp = min(1.0, t / 1.5)                      # ease the amplitude in
        return LegAngles(ramp * A_C * math.sin(phase[0]),
                         S.femur + ramp * A_F * max(0.0, math.sin(phase[0] + math.pi / 2)) - 4,
                         S.knee)
    leg.stream(stride, T, label="walking stride, speeding up")
    leg.move(0, S.femur, S.knee, 1.2, label="settle")
    leg.hold(0.6)

    # 4. bow
    leg.move(femur=-18, secs=2.2, label="bow")
    leg.hold(1.0)
    leg.move(femur=S.femur, secs=1.8, label="rise")
    leg.hold(0.5)


def main():
    loops = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    cal = load_cal()
    bus = pd.SyncBus(st.find_port())
    ids = [j.sid for j in cal.values()]
    missing = [sid for sid in ids if bus.read_pos(sid) is None]
    if missing:
        sys.exit(f"servos not answering: {missing}")
    leg = Leg(bus, cal)
    print(f"start: {leg.ang}")
    try:
        for n in range(loops):
            print(f"--- loop {n + 1}/{loops} ---")
            routine(leg)
    except KeyboardInterrupt:
        print("\nstopped")
    except RangeError as e:
        print(f"\nREFUSED: {e}")
    S = STAND
    leg.move(S.coxa, S.femur, S.knee, 2.0, label="stand")
    temps = {sid: bus.read_temp(sid) for sid in ids}
    print(f"end: {leg.read()}  temps: {temps}")
    bus.release(ids)


if __name__ == "__main__":
    main()
