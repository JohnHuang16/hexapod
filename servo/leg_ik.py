"""
leg_ik.py - inverse kinematics for one hexapod leg: foot (x, y, z) -> joint angles -> servo positions.

Run (no hardware needed, just prints):
    python3 leg_ik.py 180 0 -100          foot target in mm -> angles and servo positions
    python3 leg_ik.py --fk 0 21 -70.8     forward check: coxa, femur, tibia angles (deg) -> foot (mm)
    python3 leg_ik.py --test              self-test against docs/kinematics.md, round trips, reach limits
Run (moves the real servos, synchronized minimum-jerk move):
    python3 leg_ik.py --move 180 0 -100 [--time 1.5] [--hold]
        (motors are switched off after the move unless --hold is given; a limp leg will drop)
    python3 leg_ik.py --move --stand     shortcut for the standing pose

Leg frame (right-handed, mm):
    origin  = coxa (hip) vertical axis, at the height of the femur axis
    +x      = straight out from the body through the hip
    +y      = sideways, counter-clockwise seen from above (= positive coxa angle)
    +z      = up, so the foot is at negative z when standing (standing pose: 180, 0, -100)

Angles (degrees) match docs/kinematics.md:
    coxa  theta1 = atan2(y, x)                 0 = straight out
    femur theta2 = angle above horizontal      + = femur tilted up
    knee         = 180 - gamma                 0 = tibia in line with femur, + = folded down
    tibia phi    = theta2 - knee               angle of the tibia from horizontal (for FK only)

Servo mapping (per joint, see CAL below and leg_cal.json):
    pos = ref_pos + sign * (angle - ref_angle) * 1000/240
    ref_angle is the joint angle the leg is physically in when the servo reads ref_pos.
    Default assembly: with every servo at 500, the leg is in the STANDING pose. That puts
    +-120 deg of travel around the working pose. If you fit a horn differently, edit the JSON.
    sign is +1/-1 depending on which way each servo is mounted; find it with a small --move
    and flip it in the JSON if the joint goes the wrong way.
"""
import json
import math
import os
import sys
from dataclasses import dataclass, asdict

# ---- geometry, mm, axis to axis (docs/kinematics.md) ----
L1 = 50.0    # coxa
L2 = 90.0    # femur
L3 = 140.0   # tibia
D_MIN = abs(L2 - L3)   # 50 mm: fully folded
D_MAX = L2 + L3        # 230 mm: fully stretched
STANDING = (L1 + 130.0, 0.0, -100.0)   # foot 130 mm out from the femur axis, 100 mm below it

UNITS_PER_DEG = 1000 / 240
CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "leg_cal.json")


class ReachError(ValueError):
    """Foot target is outside what the leg can reach."""


class RangeError(ValueError):
    """Joint angle maps to a servo position outside its allowed range."""


@dataclass
class LegAngles:
    coxa: float    # theta1
    femur: float   # theta2
    knee: float    # 180 - gamma

    @property
    def tibia(self):
        """phi: tibia angle from horizontal (negative = pointing down)."""
        return self.femur - self.knee

    def __str__(self):
        return (f"coxa {self.coxa:7.2f}  femur {self.femur:7.2f}  knee {self.knee:7.2f}"
                f"  (tibia from horizontal {self.tibia:7.2f})")


# ---------------------------------------------------------------- kinematics
def ik(x, y, z):
    """Foot (x, y, z) in the leg frame -> LegAngles (degrees). Raises ReachError."""
    theta1 = math.atan2(y, x)
    r = math.hypot(x, y) - L1          # horizontal reach from the femur axis
    h = -z                             # femur axis height above the foot (+ when foot is below)
    d = math.hypot(r, h)
    if r < 0:
        raise ReachError(f"foot is inside the coxa radius (sqrt(x^2+y^2) = {r + L1:.1f} < L1 = {L1:.0f} mm)")
    if d < D_MIN - 1e-9:
        raise ReachError(f"too close: d = {d:.1f} mm, minimum is {D_MIN:.0f} mm")
    if d > D_MAX + 1e-9:
        raise ReachError(f"too far: d = {d:.1f} mm, maximum is {D_MAX:.0f} mm")
    d = min(max(d, D_MIN), D_MAX)      # clean up float noise at the exact limits

    alpha = math.atan2(h, r)                                        # d below horizontal
    beta = math.acos((L2**2 + d**2 - L3**2) / (2 * L2 * d))         # femur above d
    gamma = math.acos((L2**2 + L3**2 - d**2) / (2 * L2 * L3))       # knee interior angle
    theta2 = beta - alpha
    return LegAngles(math.degrees(theta1), math.degrees(theta2), 180.0 - math.degrees(gamma))


def fk(angles):
    """LegAngles (degrees) -> foot (x, y, z) in the leg frame."""
    t1, t2, phi = (math.radians(a) for a in (angles.coxa, angles.femur, angles.tibia))
    r = L2 * math.cos(t2) + L3 * math.cos(phi)
    z = L2 * math.sin(t2) + L3 * math.sin(phi)
    return ((L1 + r) * math.cos(t1), (L1 + r) * math.sin(t1), z)


# ---------------------------------------------------------------- servo calibration
@dataclass
class JointCal:
    sid: int
    ref_angle: float      # joint angle (deg) when the servo reads ref_pos
    ref_pos: int = 500
    sign: int = 1         # +1 or -1: which way the servo turns for a positive joint angle
    min_pos: int = 0      # soft limits, tighten these once the leg is assembled
    max_pos: int = 1000

    def to_pos(self, angle):
        pos = self.ref_pos + self.sign * (angle - self.ref_angle) * UNITS_PER_DEG
        if not self.min_pos <= pos <= self.max_pos:
            raise RangeError(f"servo {self.sid}: angle {angle:.1f} deg -> position {pos:.0f}, "
                             f"allowed {self.min_pos}..{self.max_pos}")
        return int(round(pos))

    def to_angle(self, pos):
        return self.ref_angle + self.sign * (pos - self.ref_pos) / UNITS_PER_DEG


_stand = ik(*STANDING)
DEFAULT_CAL = {
    "coxa": JointCal(1, round(_stand.coxa, 2)),
    "femur": JointCal(2, round(_stand.femur, 2)),
    "knee": JointCal(3, round(_stand.knee, 2)),
}


def load_cal(path=CAL_FILE):
    """Calibration from leg_cal.json if it exists, otherwise the standing-pose defaults."""
    cal = {k: JointCal(**asdict(v)) for k, v in DEFAULT_CAL.items()}
    if os.path.exists(path):
        with open(path) as f:
            for joint, fields in json.load(f).items():
                cal[joint] = JointCal(**{**asdict(cal[joint]), **fields})
    return cal


def save_cal(cal, path=CAL_FILE):
    with open(path, "w") as f:
        json.dump({k: asdict(v) for k, v in cal.items()}, f, indent=2)
        f.write("\n")


def angles_to_positions(angles, cal):
    """LegAngles -> {servo id: position 0-1000}. Raises RangeError."""
    return {cal["coxa"].sid: cal["coxa"].to_pos(angles.coxa),
            cal["femur"].sid: cal["femur"].to_pos(angles.femur),
            cal["knee"].sid: cal["knee"].to_pos(angles.knee)}


def positions_to_angles(positions, cal):
    return LegAngles(cal["coxa"].to_angle(positions[cal["coxa"].sid]),
                     cal["femur"].to_angle(positions[cal["femur"].sid]),
                     cal["knee"].to_angle(positions[cal["knee"].sid]))


def foot_to_positions(x, y, z, cal=None):
    """The whole chain: foot target -> servo positions. Raises ReachError / RangeError."""
    return angles_to_positions(ik(x, y, z), cal or load_cal())


# ---------------------------------------------------------------- self test
def _close(a, b, tol):
    return all(abs(p - q) <= tol for p, q in zip(a, b))


def self_test():
    # 1. numbers from docs/kinematics.md for the standing pose
    a = ik(*STANDING)
    assert _close((a.coxa, a.femur, 180 - a.knee, a.tibia), (0.0, 21.0, 88.2, -70.8), 0.05), a
    print(f"standing pose:  {a}")
    print(f"servo positions (default cal): {angles_to_positions(a, DEFAULT_CAL)}")

    # 2. IK -> FK round trip over a grid of reachable targets
    n = worst = 0
    for x in range(60, 290, 10):
        for y in range(-150, 151, 15):
            for z in range(-200, 61, 10):
                try:
                    ang = ik(x, y, z)
                except ReachError:
                    continue
                err = max(abs(p - q) for p, q in zip(fk(ang), (x, y, z)))
                worst = max(worst, err)
                n += 1
    assert worst < 1e-6, worst
    print(f"round trip IK->FK on {n} reachable targets, worst error {worst:.2e} mm")

    # 3. reach limits
    for bad in [(L1 + 231, 0, 0), (L1 + 30, 0, 0), (20, 0, -100), (L1 + 100, 0, -210)]:
        try:
            ik(*bad)
            raise AssertionError(f"{bad} should be unreachable")
        except ReachError as e:
            print(f"rejected {bad}: {e}")
    print("reachable edge cases:", ik(L1 + 230, 0, 0), "|", ik(L1 + 50, 0, 0), sep="\n  ")

    # 4. calibration round trip
    cal = DEFAULT_CAL
    for ang in (a, ik(200, 80, -60), ik(120, -40, -150)):
        back = positions_to_angles(angles_to_positions(ang, cal), cal)
        assert _close((back.coxa, back.femur, back.knee), (ang.coxa, ang.femur, ang.knee), 0.15), (ang, back)
    print("servo position round trip OK (within 0.15 deg = half a unit)")
    print("ALL TESTS PASSED")


# ---------------------------------------------------------------- CLI
def _print_target(x, y, z, cal):
    ang = ik(x, y, z)
    pos = angles_to_positions(ang, cal)
    r = math.hypot(x, y) - L1
    print(f"foot ({x:.1f}, {y:.1f}, {z:.1f}) mm   r = {r:.1f}  h = {-z:.1f}  d = {math.hypot(r, z):.1f}")
    print(f"angles: {ang}")
    print("servo:  " + "  ".join(f"ID{sid} -> {p}" for sid, p in sorted(pos.items())))
    return ang, pos


def main(argv):
    args = list(argv)
    if "--test" in args:
        return self_test()
    cal = load_cal()
    move = "--move" in args
    if move:
        args.remove("--move")
    hold = "--hold" in args
    if hold:
        args.remove("--hold")
    secs = 1.5
    if "--time" in args:
        i = args.index("--time")
        secs = float(args[i + 1])
        del args[i:i + 2]
    if "--stand" in args:
        args = [str(v) for v in STANDING]
    if "--fk" in args:
        c, f, t = (float(v) for v in args[args.index("--fk") + 1:][:3])
        ang = LegAngles(c, f, f - t)
        x, y, z = fk(ang)
        print(f"angles: {ang}\nfoot: ({x:.1f}, {y:.1f}, {z:.1f}) mm")
        return
    if len(args) != 3:
        sys.exit(__doc__)
    x, y, z = (float(v) for v in args)
    try:
        ang, pos = _print_target(x, y, z, cal)
    except (ReachError, RangeError) as e:
        sys.exit(f"ERROR: {e}")
    if not move:
        return

    import servo_test as st
    import profile_demo as pd
    bus = pd.SyncBus(st.find_port())
    now = {}
    for sid in pos:
        p = bus.read_pos(sid)
        if p is None:
            sys.exit(f"ID {sid} not answering. Check power and chain.")
        now[sid] = p
    biggest = max(abs(pos[s] - now[s]) for s in pos) / UNITS_PER_DEG
    secs = max(secs, biggest / 150.0)    # cap at 150 deg/s, well under the servo's ~330 deg/s
    print(f"now at {positions_to_angles(now, cal)}")
    print(f"moving {biggest:.0f} deg max in {secs:.2f} s")
    input("Press Enter to move (keep fingers clear)...")
    bus.move_together(pos, secs, pd.minjerk)
    final = {sid: bus.read_pos(sid) for sid in pos}
    print("landed:", final, "->", positions_to_angles(final, cal))
    if not hold:
        bus.release(pos)


if __name__ == "__main__":
    main(sys.argv[1:])
