# deploy_auto: automatic crawler alignment (ROS1 Melodic)

## Select yaw feedback in YAML

Edit `config/deploy_auto.yaml`:

```yaml
yaw_source: aruco
```

- `aruco`: office testing. Uses the orientation in `/aruco_pose` as relative
  heading feedback. MAVROS is not subscribed to or required. The marker must be
  fixed in the environment and remain visible during alignment. This is a
  relative heading reference, not an absolute vehicle heading.
- `mavros`: full experiment. Uses `/mavros/local_position/pose` for vehicle yaw.

The default is `aruco`, as requested for office testing. Change only this YAML
value to switch modes.

## Launch

```bash
roslaunch deploy_auto deploy_auto.launch
```

The launch starts the existing deployment stack, the ArUco detector/T265, the
end-effector controller, and the automatic alignment node. Press `f` in the
end-effector keyboard terminal to trigger alignment. The `enable_motors` launch
argument defaults to `true`; use `enable_motors:=false` to inhibit motion.

## Four alignment stages

1. Reverse (`b`) until the marker is in front and within `max_bearing_deg`.
2. Rotate to zero bearing: positive bearing uses clockwise (`c`); negative
   bearing uses counterclockwise (`d`).
3. Move forward (`a`) until both marker x and y are within `position_tolerance_m`.
4. Restore the initial heading using the selected `yaw_source`.

The configured bearing limit remains 85 degrees. Commands are sent repeatedly
as a heartbeat, and `e` is sent to stop the tracks at stage transitions and on
abort. The emergency-stop topic is `/deploy_auto/emergency_stop`
(`std_msgs/Bool`): publish `data: true` to abort.

## Office-mode limitation

In `aruco` mode, the yaw reference is the marker orientation relative to the
camera. It is valid for heading restoration only while the marker is stationary
and visible. If the marker is lost or its orientation estimate becomes stale,
the controller stops rather than continuing without feedback.

## Build

```bash
cd ~/piloting_ws
catkin_make
source devel/setup.bash
```