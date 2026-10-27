# CLAUDE.md: Hexapod project context

Handoff notes so Claude Code can pick up where earlier work left off (as of Oct 25, 2026).

Repo: https://github.com/JohnHuang16/hexapod (public, `main`). **Commit and push after every meaningful change** (John's standing instruction, Oct 26, 2026); don't wait to be asked. Working copy: `~/Desktop/Projects/hexapod`. A duplicate copy exists at `~/Downloads/hexapod`; treat it as stale.

## Who / why
- John Huang, UC Berkeley ECE freshman. Background in FTC robotics, CAD (Onshape), mechatronics, Python/C++.
- Building a **2 ft hexapod** as a **build-in-public TikTok series** (daily progress videos) and a portfolio project.
- Wants it to look and feel **big, sturdy, and deliberate**, not flimsy. Smooth, slow motion matters for that.
- Cost-conscious: chose a prototype-first plan (one leg, then the full robot) instead of committing ~$1k upfront.
- Prefers step-by-step explanations with diagrams and concrete numbers. Don't tell him to go to sleep.

## Plan (phases and gates)
1. **Phase 1, one leg (current):** CAD → print → servo control → FK/IK → trace shapes → step motion → **strength test** (gate: hold ~1.2 kg at the foot in standing pose without sag or overheating).
2. **Phase 2, full robot:** 15 more servos, body plates, Pi 5 (2GB), 3S LiPo + fuse + kill switch + 5 V buck, stand up, tripod gait, turning.
3. **Phase 3:** controller, camera/vision, more gaits, terrain, armor shell. Possible v2: 3 ft with brushless CAN actuators (~$6k+).

Build plan doc (Claude Docs): https://claude.ai/artifact/U4NFLFAUziTXhzSJmVreEE

## Geometry (design values, axis to axis)
- Coxa L1 = 50 mm, femur L2 = 90 mm, tibia L3 = 140 mm. Body hip radius R = 125 mm, hexagonal, legs 60° apart.
- Standing pose: foot r = 130 mm out and h = 100 mm below the femur axis (d = 164 mm). Span ≈ 610 mm.
- Reach limits: 50 ≤ d ≤ 230 mm. Joint IDs: **1 = coxa, 2 = femur, 3 = tibia** (full robot later: 11/12/13, 21/22/23, …).
- Full equations: `docs/kinematics.md`. Femur torque in standing pose ≈ 15 kg·cm (30 kg·cm with 2× safety) vs 45 kg·cm servo stall.

## Hardware on hand
- 3× **Hiwonder HTD-45H** bus servos (IDs 1, 2, 3 set and labeled). 9–12.6 V, 25T spline (counted on CAD model; confirm on part), body 40 × 20.14 × 40 mm, ~51 mm tip to tip incl. spline and rear support shaft. Case holes are **unthreaded**. Came with **plastic Ø20 mm main + assistant horns** and very short self-tapping screws.
- **LewanSoul BusLinker V3.0** (USB-C, works with a C-to-C cable). macOS port: `/dev/cu.usbmodem5C4C1267531`. 115200 baud.
- 12 V 5 A barrel supply (center-positive, reads ~12.1 V at the servos).
- Nichicon 1000 µF 25 V capacitors (not yet installed; striped leg to −).
- Preciva M3 heat-set insert kit + 60 W iron, YIHUA stand, Loctite 242, JOREST screwdriver set, silicone mat.
- **Calipers ordered, not yet arrived.** Multimeter: confirm whether he has one.
- 3D printer: undecided (considering Bambu Lab P2S, ~$549; can print at Jacobs Hall makerspace). Check dorm rules.

## Software (`servo/`, run with `python3`, needs `pyserial`)
- `servo_test.py`: `ServoBus` driver (Hiwonder/LewanSoul protocol: `0x55 0x55 | ID | len | cmd | params | ~sum`). Reads pos/voltage/temp, moves, motor on/off. Auto-finds `/dev/cu.usbmodem*`, `usbserial*`, `wchusbserial*`.
- `set_id.py N`: sets the ID of the single connected servo (broadcast 254).
- `profile_demo.py [deg] [s]`: synchronized moves (cmd 7 move-and-wait per servo + cmd 11 broadcast start, 50 Hz), comparing linear / smoothstep / min-jerk.
- `camera_demo.py [segment]`: ~1 min choreographed filming demo.
- `leg_ik.py x y z`: IK (foot mm → `LegAngles(coxa, femur, knee)` → servo positions) with reach check (50 ≤ d ≤ 230, foot outside coxa radius) and servo range check. `--move` uses `profile_demo.SyncBus` (min-jerk, speed capped at 150°/s), `--fk`, `--test` (self-test, all passing). Angle conventions: coxa 0 = straight out, femur 0 = horizontal (+ up), knee 0 = tibia in line with femur (+ folded). Knee-up solution only.
- `leg_cal.json`: per-joint `sid`, `ref_pos` (500), `ref_angle` (joint angle when servo reads ref_pos), `sign` (±1, flip if a joint runs backwards), `min_pos`/`max_pos` soft limits. Defaults = standing pose at 500 for all three (femur 20.99°, knee 91.82°). **Signs are unverified until the leg is assembled.**
- All verified on hardware except `camera_demo.py`, the updated `profile_demo.py`, and `leg_ik.py --move` (only simulated / self-tested). Position 0–1000 = 0–240°, about 4.17 units/°. Servos land within ~0–2 units of target.
- **Hiwonder "Bus Servo Terminal" Mac app crashes when commanding moves.** Use the scripts.
- **Forcing a powered servo by hand heats it fast** (Oct 26, 2026): servo 2 read 59 °C after `show_demo.py` vs 50/51 for the others, and stayed at 52 °C while holding unloaded until released. John had pulled/twisted its horn by hand while it was holding, which drives stall current. Not a bad unit; all three read 25 °C once cooled. Rule: motor off (`bus.motor(sid, False)`) or 12 V off before moving a joint by hand. Turn motors off when idle after demos.
- **Thermal baseline (Oct 26, 2026, bare horns):** identical ±60° sine sweep, 8 s period, 2 min: all three climbed 30 → 57/61/58 °C (servo 2 ~3 °C warmer, same curve shape, no binding). John had also pulled hard on servo 2's horn axially; by hand all three feel the same. Expect ~25 °C rise from a few minutes of continuous unloaded motion; the loaded strength test must log temperature.

## Mechanical decisions so far
- **Servo layout:** coxa servo vertical in the body, between top and bottom plates. Femur servo and tibia servo both live **inside the femur box**, facing each other, outputs at each end (~29 mm of each servo inward, ~32 mm gap).
- **Servo mounting:** pocket (~0.2–0.3 mm clearance/side) + **clamp plate** bolted into M3 heat-set inserts. Not screwing into the servo's unthreaded holes.
- **All joints fully decoupled** (servo carries torque only): frame plates clamp the servo, **6804-2RS** (20 × 32 × 7) on the drive side with a printed hub, **F625ZZ** (flanged 5 × 16 × 5) + **M5 shoulder bolt** on the idle side, coaxial via the same mate connector. Details and retention numbers: `docs/joint-design.md`.
- Unflanged 6804 retention: Ø32 pocket + Ø~29.5 lip (outer ring only); Ø20 journal + Ø~23 shoulder (inner ring only); journal ~6.8 mm (0.2 mm shorter than bearing); Ø~23 × 2 mm clamp washer squeezed by the horn screws.
- **Horns:** keeping the plastic horns for now (torque only in this design); concern about the short screws. Options: capture the horn in a hub pocket with pegs + longer M2 screws, or switch to **25T aluminum clamp-style horns** with M3 holes → M3 inserts (recommended).
- Print hubs upright. Test coupons for fits: journals 19.80–20.00 mm, pockets 32.0–32.2 mm.
- **Horn/assembly convention (Oct 25, 2026):** fit every horn with the servo at position **500 and the leg in the standing pose** (coxa straight out, femur 21° up, knee bent 92°). This centers the 240° servo range on the working pose. `leg_cal.json` encodes it; if a horn ends up on a different tooth, change `ref_angle` instead of re-fitting.
- **Horns fitted (Oct 26, 2026):** new horns pressed on with the servos holding at **coxa 429, femur 459, tibia 500** (John nudged 1 and 2 off center by eye). Those are now `ref_pos` in `leg_cal.json`; the servos read 430/460/498 afterwards. Decreasing position turned the horn clockwise as John viewed it on the bench. Joint `sign` values are still unverified.

## CAD status (Onshape document "hexapod")
- Imported an **HTD-35H SolidWorks model** (same case as the 45H) as the servo reference. Measured 50.90 mm tip to tip (drawing says 51.1). Use an open composite to keep colors. Add mate connectors on the spline (at the horn-seat face) and on the rear shaft.
- Made a Ø20 horn model. Leg parts (coxa bracket, femur box, tibia, hubs, plates) **not done yet**.
- `cad/HTD-45H_placeholder.step` is a rough backup model; prefer the imported HTD-35H model.

## Open items / next steps
1. **When calipers arrive:** measure the spline Ø, horn hole circle and hole size, rear shaft Ø/length, collar height, servo body along the axis (~43 mm?), and the spline axis offset from the body end (~10–11 mm). Confirm 25T.
2. **Order:** 6804-2RS (10-pack), F625ZZ (10-pack), M5 shoulder bolts sized to plate + arm thickness, maybe 25T aluminum clamp horns. Optional: M2 hardware if keeping plastic horns.
3. **CAD the joint module once**, test-print fit coupons, then the coxa → femur box → tibia → foot (rounded, rubber cap) → desk test stand.
4. ~~Write the IK script~~ **Done** (`servo/leg_ik.py`). Remaining: once the leg is assembled, verify each joint's `sign` in `leg_cal.json` with small `--move`s, set `min_pos`/`max_pos` soft limits from the real hard stops, then write the shape-tracing / step-motion script on top of `foot_to_positions()`.
5. Install the 1000 µF capacitor across the BusLinker power input.
6. Strength test → Phase 2 decision.

## Safety rules
- **Always switch motors off when a run is done** (`bus.release()`; John's rule, Oct 26, 2026). Every script does this at exit; `leg_ik.py --move` keeps holding only with `--hold`. Never leave servos holding between runs.
- Power off before plugging or unplugging servo cables. Set IDs with one servo connected. Tape servos down for fast demos.
- Keep moves within limits: the servo top speed is ~0.18 s/60° at 11.1 V. Scripts cap speeds (~264°/s max in demos).
- Later: fuse + kill switch on the LiPo line, charge LiPos in a safety bag.
