# Joint design notes

## Servo layout
- Coxa (hip) servo: in the body, vertical, clamped between the top and bottom plates.
- Femur servo: inside the femur at the hip end. Its horn drives the coxa bracket, so the servo body rotates with the femur.
- Tibia servo: inside the femur at the knee end, facing the femur servo (about 29 mm of each servo extends inward from its axis, leaving about 32 mm between them).

## Servo mounting
The HTD-45H case holes are unthreaded. Servos are held by a **pocket plus a clamp plate** bolted into M3 heat-set inserts, not by screws into the servo. Pocket clearance is about 0.2–0.3 mm per side.

## Fully decoupled joints (all joints)
The servo only supplies torque. Leg load goes link → bearings → frame.

- **Frame:** two plates clamp the servo body (belongs to the previous link).
- **Drive side:** 6804-2RS (20 × 32 × 7) in the frame plate. A printed hub journal rides in it and bolts to the Ø20 horn.
- **Idle side:** F625ZZ (5 × 16 × 5, flange 18 × 1) in the other plate, with an M5 shoulder bolt into the link's far arm, on the same axis.
- Both bearing positions come from the same mate connector on the servo axis.
- A plastic horn's slight flex absorbs small misalignment.

### Retaining an unflanged 6804
- Plate pocket Ø32 (press fit, tuned by test print) with a lip about Ø29.5. The lip touches only the outer ring.
- Hub journal Ø20 (tuned), shoulder about Ø23. The shoulder touches only the inner ring.
- Journal about 0.2 mm **shorter** than the bearing (about 6.8 mm), so the clamp washer (about Ø23, 2 mm) presses on the inner ring.
- The horn screws pass through the washer into the hub and clamp: shoulder → inner ring → washer.
- Rule: fixed parts touch only the outer ring, rotating parts only the inner ring.
- Shoulder step at least 1 mm radially, at least 2–3 mm thick axially. Fillet at the journal corner no larger than 0.3 mm.

### Printing fits
- Print hubs upright. Seam aligned away from bearing surfaces.
- Test coupon for journals: 19.80 / 19.85 / 19.90 / 19.95 / 20.00 mm. Test pockets: 32.0–32.2 mm.
- Chamfer the journal entry about 0.5 mm, and watch for elephant's foot.
- If too loose: crush ribs, or retaining compound.

## Horns
- Spline: 25T (counted on the CAD model; confirm on the servo).
- Stock horns are plastic, Ø20. Upgrade path: 25T aluminum clamp-style horns with M3 holes into M3 heat-set inserts.
- Set the servo to 500 (center) before fitting the horn. Blue threadlocker on the center screw (metal threads only).
