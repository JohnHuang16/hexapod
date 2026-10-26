"""
set_id.py - give ONE connected servo a new ID.

Connect ONLY the servo you want to change, then run:
    python3 set_id.py 2        (sets that servo to ID 2)

Uses the broadcast address, so it works whatever the servo's current ID is.
Keep servo_test.py in the same folder (this script reuses it).
"""
import sys
import time

import servo_test as st

ID_WRITE = 13
ID_READ = 14
BROADCAST = 254


def main():
    if len(sys.argv) < 2 or not sys.argv[1].isdigit() or not 0 <= int(sys.argv[1]) <= 253:
        sys.exit("Usage: python3 set_id.py NEW_ID   (0-253, e.g. 2)")
    new_id = int(sys.argv[1])

    bus = st.ServoBus(st.find_port())

    bus._send(BROADCAST, ID_READ)
    reply = bus._read_reply(None, ID_READ)  # the servo answers with its real ID
    print(f"Current ID: {reply[0] if reply else 'unknown (no reply)'}")

    input(f"Make sure ONLY ONE servo is connected. Press Enter to set it to ID {new_id}...")
    bus._send(BROADCAST, ID_WRITE, [new_id])
    time.sleep(0.2)

    pos = bus.read_pos(new_id)
    if pos is None:
        sys.exit("No reply at the new ID. Check power and wiring, then try again.")
    print(f"Done: servo now answers as ID {new_id} (position {pos}). Label it!")


if __name__ == "__main__":
    main()
