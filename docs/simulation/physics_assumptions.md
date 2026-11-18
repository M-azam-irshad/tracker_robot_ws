# Physics assumptions
* Engine gz-sim DART, step 3 ms, RTF 1.
* Total mass 25 kg (given): chassis 13 + per side 2 rollers (0.9) + sprocket (1.2) + belt/plate (2.5) + sensors ~1.
* Motor: 10 N*m stall per side (assumed at the sprocket). Pull = T / 11.2 cm = 89 N per side; shared over the 2 rollers
  -> 3.77 N*m effort limit per roller joint (`roller_effort` in robot_properties.xacro). Max thrust ~179 N (36 deg, traction-limited).
* Rollers mu = 1.0 (rolling) / 0.5 (lateral, `mu_lateral`); ramps + skid mu = 0.05 so rocks slide up the belt slope.
* Tracks are NOT simulated link-by-link. Traction = 2 powered rollers per side; the sloped belt runs (42.6 deg) are passive ramps.
* Obstacle limit of this geometry: ~ roller radius (8.5 cm). Rocks in the world are <= 8 cm; taller obstacles act as walls.
  Real belts grip better over edges, so the sim is slightly pessimistic.
* Turning: `wheel_separation_multiplier` 1.3 (tracked skid-steer). Calibrate: command a 360 deg turn, compare odom.

* Tread lugs are visual-only (no collision, no mass); they never affect physics.
