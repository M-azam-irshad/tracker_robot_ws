# Component inventory (real part -> simulation)
| Real part (STEP file) | In simulation | Source |
|---|---|---|
| `internal_chassis`, `side_plates`, `bigger_plate` (chassis, plates) | open camo box 52 x 26.5 x 16 cm (one solid collision box) + 2 side plates (visual) | table + STEP |
| `Motor` (80 mm x 169 mm) x2 | 2 visual cylinders inside the chassis; mass lumped into chassis | STEP |
| `bigger_wheel_back`, `bigger_wheel_bearing`, `bigger_wheel_hex_nut` (drive sprocket) | `*_sprocket_wheel` link, r=9.5 cm path, 14 teeth, visual only, spins with the drive | STEP |
| `smaller_wheel` (+bearing, nut) (front/rear rollers) | `*_front/rear_roller_wheel`, collision r=8.45 cm, POWERED (traction) | STEP + table |
| `track__updated` (belt) | `*_track_belt`: procedural visual mesh (same profile/lugs) + ramp/skid collisions | STEP |
| RPLIDAR / Kinect / IMU / gas / mic | unchanged sensors, repositioned on the new chassis | estimate |

Bearings and hex nuts are intentionally not modelled (tiny; they only add triangles).

## Table values used
| Table | Value | Used as |
|---|---|---|
| Ground contact length 13.5 in | 34.3 cm | consistent with STEP roller spacing 30.6 cm + curvature |
| Track width | 11.5 cm | `track_width` |
| Track height 11.3 in | 28.7 cm | STEP: 30.0 cm incl. lugs - STEP profile used |
| Ground to centre axis | 17 cm | STEP shows 18.8 cm to lug tips (17 cm + 1.9 cm teeth) -> 18.8 used as contact height |
| Track mid to mid | 47 cm | `track_y_offset` = 23.5 cm, `wheel_separation` = 0.47 |
| Track end to end | 59 cm | overall width (47 + 11.5 = 58.5), not used directly |
| Chassis 52 x 26.5, clearance 12.5 | | exact |
| Front roller OD 14.5 cm | | STEP gives 13.5 cm path dia + 1.7 cm belt/lug = 16.9 cm effective -> 8.45 cm radius used |
| Front/back offsets 2.5 / 4.5 cm | | NOT used (tracks centred on chassis) - tell me if the chassis is really offset |
