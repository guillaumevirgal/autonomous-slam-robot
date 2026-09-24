# Hardware bringup log

Canonical reproducer log for Idefix bringup steps. Logged as they happen,
with exact commands, version strings, and measured values -- not
reconstructed after the fact.

## 2026-09-23: EKF deployment + full-rotation diagnostic (left wheel bolt)

### Context

First real-hardware run of the `robot_localization` EKF (`idefix_bringup
ekf.launch.py`, fusing `/odom` + `/imu/data_raw`). The EKF commit existed
only in the WSL2 local repo, never on the Pi's checkout (the Pi's
`jazzy-port` branch had independently progressed with RPLiDAR/SLAM work).
Fixed by copying `ekf.yaml`/`ekf.launch.py` onto the Pi's checkout directly
(not merged into git history -- still needs proper reconciliation) and
rebuilding `idefix_bringup`.

### micro-ROS agent baud rate (stale doc)

`pins.h`'s `UROS_UART_BAUDRATE` is 921600, raised from 460800 at some point
after the bringup notes below were first written. Agent must be started
with the matching baud:

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/idefix-esp32 -b 921600
```

Symptom of a baud mismatch: total silence (no session-established log, no
garbled bytes either) -- easy to mistake for a dead board or bad wiring.
Also worth noting: ttyACM0/ttyACM1 enumeration order for the two USB ports
is not fixed across reboots -- always address the micro-ROS UART port by
its udev symlink (`/dev/idefix-esp32`), never a literal `ttyACM*` number.

### systemd-logind reaping background processes

The Pi's user session had `Linger=no`, so `systemd-logind`'s
`KillUserProcesses` reaped tmux servers and background jobs between
non-interactive `ssh idefix 'cmd'` invocations, with no reboot involved.
Fixed with `sudo loginctl enable-linger idefix` (persists across reboots).

### Rotation test methodology

One-shot full 360 deg in-place turn at 0.5 rad/s (12.566 s), measuring
`/odom` and `/odometry/filtered` by integrating unwrapped yaw deltas
throughout the run (comparing start/end orientation alone reads ~0 deg for
a perfect full turn). Script: `docs/rotation_test.py` (copied onto the Pi
at `~/autonomous-slam-robot/docs/rotation_test.py`, run directly with
`python3`, no colcon build needed).

Gate the start of each run on measured `/odom` angular velocity actually
reaching ~0 (not a fixed sleep) -- otherwise PID coast-down from a prior
run can bleed into the next run's window.

### Finding 1: left wheel (Motor A) intermittent under-speed

Symptom: clean, reproducible plateau at ~50% of commanded rotation
(measured `/odom` wz steady at ~0.248 rad/s vs 0.5 commanded setpoint,
tight variance, odom/filtered agreeing with each other almost exactly).
Occurred in roughly 2/3 of runs before the fix.

Diagnosis: for a pure in-place rotation, both wheels contribute equally
and oppositely to net yaw rate (`dtheta = (ds_right - ds_left) /
WHEEL_BASE_M`). Ruled out a kinematics/encoder-constant scaling bug by
inspection (`counts_to_rad_s` PID feedback and `kin_counts_to_metres`
odom conversion share the same `ENCODER_COUNTS_PER_OUTPUT_REV` constant,
so a PID at steady state should make `/odom` track the commanded setpoint
regardless of that constant's calibration). Losing one wheel's
contribution while the other still delivers full speed exactly halves the
net yaw rate, matching the measured plateau precisely. Confirmed visually:
left wheel visibly turning slower than the right during affected runs.

Fix: left wheel mounting bolt was loose; user tightened it. Verified
after: 4/5 rotation test runs at 99.5-99.7% of commanded (vs. the prior
~50% plateau in most runs), disagreement <0.2 deg.

Validation rosbag recorded post-fix:
`~/robot_ws/bags/idefix_ekf_rotation_2026-09-23` (topics matching the
Sep 14 straight-line validation bag: `/odom`, `/odometry/filtered`,
`/imu/data_raw`, `/tf`, plus `/cmd_vel`; `/tf_static` not captured -- it's
transient-local and the recorder started listening after the EKF's one-shot
publish). Captured run: 99.9%/99.9% of commanded, 0.13 deg disagreement, no
velocity spikes -- pairs with the existing good straight-line result.

### Finding 2 (still open): torn-read race in `odom_timer_callback`

Separate from Finding 1, unaffected by the bolt fix -- still reproduces
after it (1/5 runs in the verification batch). `main.c`'s
`odom_timer_callback` reads `g_snapshot_counts_a`, `g_snapshot_counts_b`,
`g_snapshot_time_us` as three separate atomic loads (comment acknowledges
this is not one transaction, assumes "at worst ~10ms of temporal skew,
invisible to Nav2"). Observed effect is larger than assumed: occasional
single-sample `/odom` angular velocity spikes of ~200-290 rad/s (physically
impossible), which then feed a large single-step yaw delta into any
integrator. Ruled out serial transmission corruption first (no CRC/framing
errors in the micro-ROS agent log during affected runs; battery voltage
also flat at 10.85-10.87V throughout, ruling out voltage-sag/PID-clamp
theories too). Not yet fixed -- needs the three atomic loads to become one
consistent snapshot (e.g. pack into a struct behind a single atomic, or a
seqlock) before trusting `/odom` angular velocity for anything beyond this
kind of diagnostic.

## 2026-09-24: Online SLAM + Nav2 on hardware (navigate while mapping)

### Context

Goal: run Nav2 against slam_toolbox's live `/map` (mapping mode) instead of
a saved map + localization, so the robot can navigate in rooms it has never
mapped. New launch file `idefix_bringup/launch/slam_nav.launch.py`: includes
`slam_mapping.launch.py` unchanged and adds the Nav2 servers without
`map_server` (global costmap `static_layer` subscribes to the live `/map`).

### Nav2 was not installed on the Pi

Nav2 had only ever run in Gazebo on WSL2. Installed on the Pi with
`sudo apt install ros-jazzy-navigation2` (1.3.13-1noble.20260907.041002).

Side issue: on the phone-hotspot network (`192.168.250.0/24`) the Pi had no
usable internet (TCP handshakes completed but no data came back, DNS over
UDP and TCP both timed out). Workaround used to confirm the package
servers were reachable: an SSH reverse SOCKS tunnel through the dev machine
(`ssh -R 1080 idefix`, then `apt -o
Acquire::http::Proxy=socks5h://localhost:1080 ...`). The install itself
completed after the Pi was rebooted and the network recovered.

### Bringup

```
# tmux window 1
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/idefix-esp32 -b 921600
# tmux window 2
ros2 launch idefix_bringup slam_nav.launch.py
```

Verified: `/odom` 30 Hz, `/scan` 10 Hz, `map -> odom -> base_footprint ->
base_link` resolves, lifecycle manager "Managed nodes are active", global
costmap log `StaticLayer: Resizing costmap to 97 X 62` (sized from the live
`/map`). Pi 5 load ~3.3 (4 cores), ~760 MB RAM with the full stack.

First goal (~0.5-1 m, sent from RViz2 over `ssh -X`) succeeded in 9 s, but
the robot stopped briefly several times on the way (see open issue below).

### Bug: every Nav2 node ran on sim time

A second goal failed (RPP "collision ahead", then NavFn could not plan), and
the recovery spin failed with `behavior_server: Costmap is not available`.
The BT then hung in the wait recovery and ignored every later goal.

Diagnosis: `ros2 param get <node> use_sim_time` returned `True` on all Nav2
nodes, including the costmap sub-nodes, even though the launch file set
`SetParameter(use_sim_time=false)`. `nav2_params.yaml` sets
`use_sim_time: True` per node (Gazebo default), and those node-scoped
values override a launch-level `SetParameter`. With no `/clock`, ROS time
stayed at 0, so the costmaps' time-gated publish never fired
(`ros2 topic hz /global_costmap/costmap` -> no messages, while
`/local_costmap/published_footprint` ran at 5 Hz). The first goal worked
only because the controller reads its costmap in-process.

Fix: `slam_nav.launch.py` passes the params through
`nav2_common.launch.RewrittenYaml(param_rewrites={'use_sim_time': ...})`,
the same way `nav2_bringup` does. Verified after: `use_sim_time` False on
all Nav2 nodes, `/global_costmap/costmap` and `/local_costmap/costmap_raw`
publishing. `nav2.launch.py` (saved-map path) has the same latent bug when
launched with `use_sim_time:=false`.

### Open: stalls during goal following

Stalls during the first goal are not explained by the Nav2 log (no
recovery, no collision stop, no progress-checker failure). Candidates: the
whole short path fell inside RPP's approach slow-down zone
(`approach_velocity_scaling_dist: 0.6`, down to 0.05 m/s, possibly below
motor stiction with the 0.24 duty clamp), rotate-to-heading on each 1 Hz
replan, or cost-regulated scaling near inflated obstacles. Recording
`/cmd_vel_nav`, `/cmd_vel`, `/odom`, `/plan`, `/tf` to `~/bags/goal_run_*`
for the next goal to separate "Nav2 commands low speed" from "motors don't
follow the command". Note: the first run happened with the sim-time bug
active, so it may not be representative.
