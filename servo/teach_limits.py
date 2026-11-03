"""
teach_limits.py - learn a joint's safe range by moving it by hand.

Run:  python3 teach_limits.py [ids...] [--secs N]      default: ids 1 2 3, 60 s
Releases the motors, then logs each servo's position while you move the joints by
hand. Prints the min/max seen every 5 s and a summary at the end. Use the summary to
set min_pos/max_pos (and check sign) in leg_cal.json. Motors stay off afterwards.
"""
import sys
import time

import servo_test as st


def main(argv):
    secs = 60.0
    if "--secs" in argv:
        i = argv.index("--secs")
        secs = float(argv[i + 1])
        del argv[i:i + 2]
    ids = [int(a) for a in argv] or [1, 2, 3]
    bus = st.ServoBus(st.find_port())
    ids = [sid for sid in ids if bus.read_pos(sid) is not None]
    if not ids:
        sys.exit("no servos answering")
    bus.release(ids)
    lo = {sid: 9999 for sid in ids}
    hi = {sid: -9999 for sid in ids}
    first = {sid: bus.read_pos(sid) for sid in ids}
    print(f"start positions: {first}  -- move the joints by hand now ({secs:.0f} s)")
    t0 = time.perf_counter()
    next_print = 5.0
    while (t := time.perf_counter() - t0) < secs:
        for sid in ids:
            p = bus.read_pos(sid)
            if p is None:
                continue
            lo[sid] = min(lo[sid], p)
            hi[sid] = max(hi[sid], p)
        if t >= next_print:
            print(f"{t:4.0f}s  " + "  ".join(f"ID{sid}: now {bus.read_pos(sid)} range {lo[sid]}..{hi[sid]}" for sid in ids), flush=True)
            next_print += 5.0
        time.sleep(0.02)
    print("\nsummary (position units, 4.17 per degree):")
    for sid in ids:
        print(f"  ID{sid}: start {first[sid]}  min {lo[sid]}  max {hi[sid]}  "
              f"span {(hi[sid] - lo[sid]) / st.UNITS_PER_DEG:.0f} deg  end {bus.read_pos(sid)}")
    bus.release(ids)


if __name__ == "__main__":
    main(sys.argv[1:])
