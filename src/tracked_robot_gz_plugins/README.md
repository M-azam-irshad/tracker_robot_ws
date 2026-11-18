# tracked_robot_gz_plugins  (OPTIONAL)

`TrackAnimator` - a Gazebo Harmonic system plugin. It makes the tread lugs of the tracks visibly circulate:
every step it reads the drive-roller joint angle, converts it to belt travel (s = R * theta) and moves each lug
visual (`<prefix>_lug_<i>`) along the rounded-triangle belt path. It changes VISUALS only - physics is untouched.

Requires the dev headers: `sudo apt install libgz-sim8-dev` (normally already pulled in by ros-jazzy-ros-gz).

If it does not build, the rest of the workspace still works (lugs just stay static):
```bash
touch src/tracked_robot_gz_plugins/COLCON_IGNORE      # skip it
```
The path maths (`BuildPath`) mirrors `build_path()` in tracked_robot_description/scripts/generate_meshes.py -
keep the two in sync if you ever change the track geometry.
