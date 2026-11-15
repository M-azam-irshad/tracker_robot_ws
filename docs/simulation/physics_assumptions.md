# Physics assumptions
* Engine: gz-sim DART, step 3 ms, RTF 1.
* Ground friction mu=1. Wheels mu=0.9 (rolling), mu2=0.5 (lateral) -> skid-steer slip.
* Tracks = wheels, collision radius = outer track radius (7 cm).
* Obstacles are static boxes/slabs <= 22 cm; robot ground clearance 4 cm, wheel radius 7 cm, so
  large rubble is intentionally a traversability challenge.
* Odometry from wheels drifts in turns; tune `wheel_separation_multiplier`.
