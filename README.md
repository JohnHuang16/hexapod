# Hexapod

A 2 ft (610 mm) six-legged walking robot built around Hiwonder HTD-45H serial bus servos.
Built in public: daily progress on TikTok.

**Status:** Phase 1 (one-leg prototype). Three servos are wired on one bus and controlled from Python, with synchronized, motion-profiled moves. Leg CAD is in progress.

## Hardware

| Part | Notes |
|---|---|
| Hiwonder HTD-45H bus servo ×3 (×18 for the full robot) | 45 kg·cm stall at 11.1 V, 9–12.6 V, 25T spline, position/voltage/temperature feedback |
| LewanSoul BusLinker V3.0 | USB to the servo bus. Shows up on macOS as `/dev/cu.usbmodem…` |
| 12 V 5 A supply (desk) | Barrel jack into the BusLinker. Phase 2 moves to a 3S LiPo |
| Bearings per joint | 6804-2RS (20×32×7) on the drive side, F625ZZ + M5 shoulder bolt on the idle side |

Full parts list with prices: [`bom/Hexapod_BOM.xlsx`](bom/Hexapod_BOM.xlsx)

## Leg geometry

| | Value |
|---|---|
| Coxa / femur / tibia (axis to axis) | 50 / 90 / 140 mm |
| Body hip radius | 125 mm, legs 60° apart |
| Standing pose | foot 130 mm out, 100 mm below the femur axis |
| Span | ≈ 610 mm |

Kinematics, torque, and stability math: [`docs/kinematics.md`](docs/kinematics.md)
Joint and bearing design notes: [`docs/joint-design.md`](docs/joint-design.md)

## Servo code (`servo/`)

Setup once: `pip3 install pyserial`. Run scripts from the `servo/` folder; they share `servo_test.py`.

| Script | What it does |
|---|---|
| `servo_test.py` | Finds the BusLinker, reads position / voltage / temperature for IDs 1–3, and wiggles each servo. Also the shared `ServoBus` driver |
| `set_id.py N` | Sets the ID of the **one** connected servo to `N` |
| `profile_demo.py [deg] [s]` | Moves all three servos together, comparing linear, smoothstep, and minimum-jerk profiles |
| `camera_demo.py [segment]` | Choreographed ~1 minute show for filming (wave, face-off, speed ramp, robot mode, bow) |
| `leg_ik.py x y z` | Inverse kinematics: foot target (mm, leg frame) → joint angles → servo positions, with a reach check. `--move` sends the leg there (synchronized min-jerk), `--fk` checks the other direction, `--test` runs the self-test. Per-joint zero offsets, direction signs and soft limits live in `leg_cal.json` |

Joint IDs: **1 = coxa, 2 = femur, 3 = tibia**. Chain: BusLinker → 1 → 2 → 3.

Leg frame for `leg_ik.py`: origin at the coxa axis at femur-axis height, +x straight out from the body, +y counter-clockwise seen from above, +z up. Standing pose is (180, 0, −100). Assembly convention: fit every horn with the servo at position 500 and the leg in the standing pose, so each joint has ±120° of travel around where it works.

### How the bus works

Each servo has an ID and listens on one shared wire. Packets are:

```
0x55 0x55 | ID | length | command | params… | checksum
checksum = ~(ID + length + command + params) & 0xFF
```

Commands used: move with time (1), move-and-wait (7), start all (11), read temperature (26), voltage (27), position (28), ID read/write (14/13), motor on/off (31). Position 0–1000 maps to 0–240°.

Synchronized motion: every 20 ms each servo gets a "move-and-wait" to its next point on the profile curve, then a single broadcast "start" makes all of them move on the same tick.

### Safety

- Power off before plugging or unplugging servo cables.
- Set IDs with only one servo connected.
- Tape servos down before fast demos.
- The Hiwonder "Bus Servo Terminal" app crashed when moving servos on macOS. Use it only for reading, or use these scripts.

## CAD

`cad/HTD-45H_placeholder.step` is a rough servo model from the drawing (body 40 × 20.14 × 40 mm, output axis at the origin). A more detailed HTD-35H model (same case) is used in Onshape. Verify fits against the real servo before printing.

## Roadmap

1. **Phase 1:** one leg. CAD, print, IK, step motion, and a strength test holding ~1.2 kg at the foot.
2. **Phase 2:** six legs, body, LiPo power, stand up, tripod gait.
3. **Phase 3:** controller, camera, gaits, terrain, personality.
