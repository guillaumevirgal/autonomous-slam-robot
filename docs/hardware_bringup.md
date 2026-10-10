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

### Root cause of the ~200-290 rad/s /odom spike: PCNT overflow counted twice

Found from a 12-minute `slam_nav` bag (`~/bags/goal_run_190907`): two
`/odom` samples with `vx=16.53 m/s, wz=286.4 rad/s` and `vx=12.34,
wz=-215.5`, each followed by a permanent 0.50 m / 132 deg odom pose jump.
slam_toolbox then corrected `map -> odom` in 21 deg steps (the edge of its
angular search window) and never recovered, ending 42 deg off, giving ghost
walls and planner failures.

Both spikes match exactly 30000 encoder counts on a single wheel within one
odom period: vx/wz = L/2 (one wheel only; 16.53/286.4 = 0.0577 m, L = 0.115
m); 30000 / 4200 counts/rev x 2 pi x 0.022 m = 0.987 m of wheel travel;
0.987 / 0.115 = 8.585 rad = 491.9 deg -> 131.9 deg wrapped (measured 132.3
and 133.8); body displacement 0.494 m (measured 0.496, 0.494); dt =
8.585 / 286.4 = 30 ms (one odom period).

Cause: `encoder.c` set `unit_cfg.flags.accum_count = true`, so the ESP-IDF
5.3 PCNT driver already adds +/-30000 to its internal `accum_value` on each
watch-point hit and `pcnt_unit_get_count()` returns `hw + accum_value`
(`components/esp_driver_pcnt/src/pulse_cnt.c`). The firmware's own
`on_reach` callback also added `watch_point_value` to a software
accumulator, and `encoder_read_counts()` returned `accumulator +
get_count()`, so every overflow was counted twice. This is deterministic,
not a race: it fires each time a wheel's net count crosses +/-30000
(~0.99 m of net wheel travel). A 360 deg in-place rotation moves each wheel
~11000 counts, which is why it showed up in roughly 1 of 3 rotation runs.
Supersedes the torn-read hypothesis in "Finding 2" above (the portMUX fixes
from 849f7e7 remain correct, but did not address this).

Fix: removed the `on_reach` callback and the software accumulator; kept
`accum_count` and the watch points (the driver needs them), and
`encoder_read_counts()` now returns the driver's extended count directly.
Flashed from the Pi over the UART port (app image only, esptool 4.7.0,
`--no-stub`), then reset via the native USB-JTAG port so the micro-ROS
handshake hit a listening agent:

```
esptool --chip esp32s3 --port /dev/idefix-esp32 -b 460800 --no-stub \
  write_flash --flash_mode dio --flash_freq 80m --flash_size 8MB \
  0x10000 idefix_firmware.bin
esptool --chip esp32s3 --port /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*-if00 \
  --no-stub --after hard_reset chip_id
```

Verified: two back-to-back 4-turn in-place spins at 0.8 rad/s, the second
crossing the next overflow (~5.5 turns since boot): odom yaw 1428.7 deg for
1440 deg commanded (99.2%), max |wz| 0.887 rad/s, largest single-sample yaw
step 2.02 deg (vs 132 deg before), no spikes.
(2026-09-28: the spike fix holds, but this 99.2% is not evidence of correct
rotation. Odom and the commanded rate share the same wrong `WHEEL_BASE_M`,
so they agree regardless of what the robot physically does. See "Wheel
base was half the real value" below.)

Side finding (not fixed): integrated `/imu/data_raw` gyro z over the same
spin gave 710 deg, almost exactly half of odom. IMU stamps are a clean
100 Hz, so the likely cause is gyro scale: `imu.c` assumes the power-on
+/-250 dps range (131 LSB/dps) for the WHO_AM_I 0x70 (MPU6500-class) part.
Matters for the EKF, which fuses gyro yaw rate; not used by `slam_nav`.
(2026-09-28: wrong diagnosis. The gyro was right; odom overstated rotation
~1.96x. See below.)

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
(Resolved 2026-09-28, see "Nav2 speed limits matched to the drivetrain".)

## 2026-09-28: Rotation, SLAM and LiDAR calibration on hardware

### Context

Goal runs with `slam_nav` still produced smeared maps with ghost walls and
paths that did not point where the robot was going, even after the PCNT
fix. Four independent causes were found, each from a recorded bag or a
dedicated test, and fixed one at a time. All bags are in `~/bags/` on the
Pi (`/cmd_vel_nav`, `/cmd_vel`, `/odom`, `/plan`, `/tf`, `/tf_static`,
plus `/imu/data_raw` from `wheelbase_*` on).

Bag recovery note: unplugging the LiPo reboots the Pi (see root
`CLAUDE.md`), which leaves an MCAP bag without `metadata.yaml`.
`ros2 bag reindex <dir> -s mcap` recovered the full 14.4 min recording.

### Nav2 speed limits matched to the drivetrain

From `goal_run_190907`: commanded 0.16-0.18 m/s gave 0.08 m/s measured.
Top speed is ~0.085 m/s because of the `PID_OUT_MAX` 0.24 duty clamp,
which is a hard limit, not a tuning choice: the N20s are the 3.3 V variant
and the ceiling at 3S is ~0.26 duty. Nav2 was tuned for 0.20 m/s (sim), so
both wheel PIDs sat saturated and could not steer.

Fix, in `slam_nav.launch.py` only (the shared `nav2_params.yaml` keeps the
sim tuning), with ~20% headroom:

| Parameter | Sim | Hardware |
|---|---|---|
| RPP `desired_linear_vel` | 0.20 | 0.07 m/s |
| velocity_smoother vx limits | +/-0.22 | +/-0.07 m/s |
| RPP `rotate_to_heading_angular_vel` | 0.8 | 0.5 rad/s |
| velocity_smoother wz limits | +/-1.0 | +/-0.6 rad/s |
| behavior_server `max_rotational_vel` | 1.0 | 0.6 rad/s |
| progress_checker `required_movement_radius` | 0.5 | 0.25 m |

The angular ceiling is 2 x 0.085 / 0.230 = ~0.74 rad/s in place. The
progress checker needed 0.05 m/s average, too close to the new cruise.
Scalar overrides go through `RewrittenYaml` path rewrites; the smoother's
list parameters are set as a second parameter dict on its `Node` because
`RewrittenYaml` raises `TypeError` on list-element paths
(`max_velocity.0`) and would turn a whole-list rewrite into a string.

Verified (`speed_limit_181244`): commanded 0.052 m/s mean, measured 0.049
m/s, 0 stalls in 3 goal segments.

### Wheel base was half the real value

Same bag: every slam_toolbox yaw correction opposed the direction of turn
(61 deg of odom rotation, +21 deg correction, the matcher's search limit
again), with no odom spikes at all. Cause: firmware
`robot_kinematics.h` had `WHEEL_BASE_M 0.115` ("refine empirically") while
the URDF had `track_width 0.216` ("TODO measure"). The firmware uses
`WHEEL_BASE_M` both to turn `/cmd_vel` into wheel speeds and to integrate
odom, so the robot physically turned 0.115 / L of what it reported, and
the two always agreed with each other. Measured on the assembly: 0.225 m
wheel centre to wheel centre. This also explains the 2026-09-24 "gyro reads
half" finding: 710 / 1429 = 0.50 vs 0.115 / 0.225 = 0.51.

2-turn in-place spin (0.5 rad/s, 25.13 s, floor mark as ground truth):

| `WHEEL_BASE_M` | Odom | Gyro | Floor mark |
|---|---|---|---|
| 0.115 (09-24, 4 turns) | 1428.7 deg | 710 deg | not checked |
| 0.225 | 722.0 deg | 702.8 deg | ~15-20 deg short |
| 0.230 | 722.5 deg | 719.4 deg | ~1 deg |

The effective (kinematic) track is 0.225 x 720 / 702.8 = 0.230 m,
slightly wider than the wheel centres (tire contact/scrub). Firmware uses
0.230; the URDF keeps the measured 0.225 as geometry (the sim has no
scrub). The gyro scale (+/-250 dps, 131 LSB/dps) is correct and needs no
change.

### slam_toolbox ignored in-place rotation

`wheelbase_200513`: map still soft. `map -> odom` changed only twice in
8 min, i.e. 2 scans processed during ~3 min of Nav2 driving; one uncorrected
wheel slip (odom +122 deg vs gyro +54 deg over one goal) was then fixed in
a single capped -20.4 deg jump. Scan/odom/TF stamps were fine (scan
received ~100 ms after stamp, one period; odom and TF current).

Cause, from slam_toolbox `jazzy` `SlamToolbox::shouldProcessScan()`:
unless `check_min_dist_and_heading_precisely` is true, it only checks
`dist2 < 0.8 * min_dist2` and ignores `minimum_travel_heading`, so pure
rotations never trigger a scan match. The parameter was backported to
Jazzy (#819) and is present in the installed 2.8.5. Fix in
`slam_toolbox_mapping.yaml`:

```
check_min_dist_and_heading_precisely: true
minimum_travel_distance: 0.2   # was 0.5
minimum_travel_heading: 0.2    # was 0.5 (and ignored)
```

Verified (`slamfix_201706`, 3 goals): 79 updates, per-update yaw
correction median 0.40 deg, max 2.40 deg. Heading over the run: odom
+142.3, gyro +142.2, SLAM +144.3 deg.

### LiDAR mounting yaw measured

Same bag: between SLAM updates on straight segments the corrected
trajectory moved -17.4 deg (weighted mean, n=3) sideways relative to the
robot heading, which a diff-drive cannot do, with 5-8 cm translation
corrections per update. The Pi had an uncommitted base->laser yaw of -30
deg, a guess read off an earlier map while the wheel base was still wrong.

Direct measurement: drove 0.602 m straight at 0.05 m/s (odom lateral
0.000 m, dyaw +0.22 deg; gyro +0.49 deg), took per-beam median scans over
1.5 s at rest before and after, and matched them with 2D point-to-point
ICP (coarse search over direction, then SVD refinement): translation
(+0.581, +0.083) m, |t| 0.587 m, rotation +0.86 deg, 79% of points within
5 cm, inlier RMS 1.4 cm. The robot's forward axis is +8.1 deg in the laser
frame, so base_link -> laser yaw = -8.1 deg (-0.1414 rad). Set in both
`rplidar_c1.launch.py` (static TF) and `idefix.urdf.xacro`
(`lidar_joint`), which must match.

Verified 2026-10-10 (`laseryaw_221557`, 3 goals, fresh map): sideways
offset on straight segments -1.1 deg weighted mean (n=5: -4.8, -4.1, +1.3,
+1.5, +0.7) vs -17.4 before. SLAM corrections per update: yaw median 0.80
deg, max 2.00; translation median 1.3 cm (was 2.2), max 6.7 cm. Odom vs gyro
per goal: +36.3/+37.9, +74.1/+75.3, -73.8/-72.0 deg. No stalls; commanded
0.051-0.056 m/s, measured 0.047-0.052 m/s. (Over the whole 15 min bag odom
and gyro differ by 12 deg: gyro bias integrated while parked between goals.)

Map saved from this session, both pairs, on the Pi in `~/robot_ws/maps/`:
`idefix_room_20261010.pgm/.yaml` (`nav2_map_server map_saver_cli`; the
`nav2_map_saver` package name previously documented in
`slam_mapping.launch.py` does not exist) and
`idefix_room_20261010.data/.posegraph` (`/slam_toolbox/serialize_map`).

### Low-battery alarm

The continuous tone at 10.0 V was too intrusive during long sessions.
`battery_task` now beeps twice (75 ms on, 100 ms gap) every 15 s while the
alarm is active, starting immediately on entry; the pattern fits inside one
500 ms task period. Threshold 9.5 V (3.17 V/cell under load), clear at 9.8
V. Flashed 2026-10-10 with the wheel base change. Pending: check the ADC
reading against a multimeter before relying on the 9.5 V margin.
