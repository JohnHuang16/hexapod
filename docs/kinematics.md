# Kinematics and physics reference

## Constants
- L1 = 50 mm (coxa), L2 = 90 mm (femur), L3 = 140 mm (tibia), all axis to axis
- R = 125 mm (body center to hip axis), legs 60° apart
- M ≈ 3.5 kg, g = 9.81 m/s²

r, h, d and all joint angles are **variables**: recomputed for every foot target. The standing pose (r = 130, h = 100) is just the design default.

## Inverse kinematics (foot target → joint angles)
Foot target (x, y) in the leg frame (x = straight out from the hip). h = height of the femur axis above the foot.

```
θ1 = atan2(y, x)                                coxa angle
r  = √(x² + y²) − L1                            horizontal reach from femur axis
d  = √(r² + h²)                                 femur axis → foot
reach check: |L2 − L3| ≤ d ≤ L2 + L3            (50 ≤ d ≤ 230 mm)

α  = atan2(h, r)                                angle of d below horizontal
β  = acos((L2² + d² − L3²) / (2·L2·d))          law of cosines, side L3 opposite
θ2 = β − α                                      femur angle above horizontal

γ  = acos((L2² + L3² − d²) / (2·L2·L3))         knee interior angle, side d opposite
knee bend = 180° − γ
φ  = θ2 − (180° − γ)                            tibia angle from horizontal
```

Check (standing pose, r = 130, h = 100): d = 164.0, α = 37.6°, β = 58.6°, θ2 = 21.0°, γ = 88.2°, φ = −70.8°.
Always check the reach before calling `acos`, or it returns NaN.

Why the top view shows 180 mm, not 280: hip to foot = L1 + L2·cos θ2 + L3·cos φ = 50 + 84 + 46.

## Forward kinematics (check)
```
r = L2·cos θ2 + L3·cos φ
z = L2·sin θ2 + L3·sin φ          (z = −h)
x = (L1 + r)·cos θ1,  y = (L1 + r)·sin θ1
```

## Body frame → leg frame
Hip i at angle ψᵢ:
```
dx = X − R·cos ψᵢ,   dy = Y − R·sin ψᵢ
x  =  dx·cos ψᵢ + dy·sin ψᵢ
y  = −dx·sin ψᵢ + dy·cos ψᵢ
```

## Torque
```
τ = F · r⊥,   F_leg = M·g / 3 (tripod gait),   1 kg·cm = 0.0981 N·m
τ_femur = F_leg · r                → 11.4 N × 0.13 m = 1.49 N·m (15.2 kg·cm), ×2 safety = 30.4 kg·cm
τ_tibia = F_leg · |L3·cos φ|       → 0.52 N·m (5.4 kg·cm)
```
HTD-45H: 45 kg·cm stall. The femur is the critical joint.

## Stability
The center of mass projected to the ground must stay inside the support triangle of the planted feet. Stability margin = shortest distance from that point to a triangle edge.

## Gait and motion profiles
```
v = S / (β_d · T)          tripod: β_d = 0.5
swing height: z(t) = H·sin(π·t / t_swing)
smoothstep:   s(u) = 3u² − 2u³
minimum jerk: s(u) = 10u³ − 15u⁴ + 6u⁵      u = t / duration
```

## Servo command
```
pos = 500 + (θ − θ_center) · 1000/240        ≈ 4.17 units per degree
```

## Electrical
```
P = V·I,   V_drop = I·R,   ΔV ≈ I·Δt / C,   runtime ≈ 0.8·capacity / I_avg
```
