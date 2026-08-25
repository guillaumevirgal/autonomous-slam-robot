// main.c
//
// Idefix firmware: micro-ROS integration on top of the Stage 2 PID work.
//
// Architecture:
//   Two FreeRTOS tasks share four atomic variables and never call each other.
//
//   control_task (100 Hz, priority 5, core 1, stack 4096):
//     Owns the PID loop. Reads shared setpoints atomically at the top of each
//     iteration, runs the two PIDs (unchanged from Stage 2), writes an atomic
//     encoder snapshot at the bottom of each iteration. Implements the /cmd_vel
//     500 ms watchdog by forcing setpoints to zero if no message has arrived.
//     Never touches ROS.
//
//   uros_task (executor-driven ~50 Hz spin, priority 3, core 0, stack 16000):
//     Owns micro-ROS. Subscribes to /cmd_vel, unpacks the Twist into per-wheel
//     omegas via robot_kinematics.h, writes the shared setpoints atomically.
//     A 30 Hz timer reads the encoder snapshot, integrates pose in the odom
//     frame, publishes /odom (nav_msgs/Odometry) and /tf (odom -> base_link).
//     Never touches PID, motors, or PCNT.
//
// Shared state (stdatomic.h):
//   uros_task -> control_task: g_setpoint_a, g_setpoint_b, g_last_cmd_vel_us
//   control_task -> uros_task: g_snapshot_counts_a, g_snapshot_counts_b, g_snapshot_time_us
//   No mutex. No critical section. Float and int64 loads/stores are word-atomic
//   on ESP32-S3 for naturally aligned addresses, and _Atomic enforces that.
//
// Transport:
//   micro-ROS over UART0 at 460800 baud, via the DevKitC-1 onboard USB-UART bridge
//   on the "UART" USB-C port. Console output (ESP_LOG, printf) is routed to the
//   native USB-Serial/JTAG on the "USB" USB-C port. See docs/hardware_bringup.md
//   for the two-cable topology and menuconfig requirements.

#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "pins.h"
#include "robot_kinematics.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdatomic.h>

// micro-ROS core
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// micro-ROS transport (custom UART on UART0)
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

// Message type supports
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>

static const char *TAG = "main";

// -------------------------------------------------------------------------
// Loop timing
// -------------------------------------------------------------------------

// Control loop period: 100 Hz, matches Stage 2. Do not change without re-tuning PID.
#define CONTROL_PERIOD_MS      10

// Odometry publish period: 30 Hz nominal. Actual jitter bounded by FreeRTOS
// tick period (10 ms), so /odom fires at 30 ms or 40 ms intervals. Timestamps
// are accurate because we stamp at publish time with rmw_uros_epoch_nanos(),
// so robot_localization / Nav2 do not care about the arrival jitter.
#define ODOM_TIMER_PERIOD_MS   33

// /cmd_vel watchdog: if no message received within this window, force zero
// setpoints. 500 ms is 10 missed Nav2 controller frames at the default 20 Hz
// rate. Adjust if the network is very lossy or if we want tighter safety.
#define CMD_VEL_TIMEOUT_US     500000

// -------------------------------------------------------------------------
// PID constants (unchanged from Stage 2, validated wheels-off)
// -------------------------------------------------------------------------

#define PID_KP                 0.04f
#define PID_KI                 1.0f
#define PID_KD                 0.0f
#define PID_OUT_MIN            (-0.24f)   // hard duty ceiling from Stage 1
#define PID_OUT_MAX            ( 0.24f)
#define PID_INT_MIN            (-2.0f)    // integral clamp, belt-and-braces
#define PID_INT_MAX            ( 2.0f)

// Coast bypass: below this |setpoint|, skip PID, command coast, reset integrator.
// Reasoning: below the motor deadband, the PID would wind up trying to move a
// stationary wheel. Coast is safer and quieter.
#define COAST_EPS_RAD_S        0.05f

// -------------------------------------------------------------------------
// Encoder velocity filter (unchanged from Stage 2)
// -------------------------------------------------------------------------

#define FILTER_WINDOW          5

// -------------------------------------------------------------------------
// Shared atomics
// -------------------------------------------------------------------------

// Written by uros_task, read by control_task.
static _Atomic float   g_setpoint_a       = 0.0f;    // rad/s, left wheel (motor A)
static _Atomic float   g_setpoint_b       = 0.0f;    // rad/s, right wheel (motor B)
static _Atomic int64_t g_last_cmd_vel_us  = 0;       // esp_timer_get_time() of last /cmd_vel; 0 = never

// Written by control_task, read by uros_task.
static _Atomic int64_t g_snapshot_counts_a = 0;      // raw accumulated encoder counts, left
static _Atomic int64_t g_snapshot_counts_b = 0;      // raw accumulated encoder counts, right
static _Atomic int64_t g_snapshot_time_us  = 0;      // esp_timer_get_time() at snapshot

// -------------------------------------------------------------------------
// PID and filter state (control_task private)
// -------------------------------------------------------------------------

static pid_t pid_a;                                  // left wheel PID (per-motor state)
static pid_t pid_b;                                  // right wheel PID (per-motor state)

static float filt_buf_a[FILTER_WINDOW] = {0};        // ring buffer, left velocity samples
static float filt_buf_b[FILTER_WINDOW] = {0};        // ring buffer, right velocity samples
static int   filt_idx_a = 0;                         // ring index, left
static int   filt_idx_b = 0;                         // ring index, right

// -------------------------------------------------------------------------
// Odometry integration state (uros_task private)
// -------------------------------------------------------------------------

// Integrated pose in the odom frame. Updated in the odom timer callback.
static double  odom_x       = 0.0;                   // metres
static double  odom_y       = 0.0;                   // metres
static double  odom_theta   = 0.0;                   // radians, yaw

// Previous snapshot values so we can compute deltas each timer tick.
static int64_t odom_last_counts_a = 0;               // encoder counts at last integration
static int64_t odom_last_counts_b = 0;
static int64_t odom_last_time_us  = 0;               // timestamp at last integration
static bool    odom_primed        = false;           // false until first snapshot read

// Running body-frame velocity estimates, published in the /odom twist field.
static double odom_vx       = 0.0;                   // m/s along base_link X (forward)
static double odom_wz       = 0.0;                   // rad/s about base_link Z (yaw)

// -------------------------------------------------------------------------
// Frame ID string buffers (static so rosidl String pointers stay valid)
// -------------------------------------------------------------------------

static char frame_odom[8]      = "odom";             // parent frame for /odom and TF
static char frame_base[16]     = "base_link";        // child frame for /odom 

// -------------------------------------------------------------------------
// micro-ROS handles and message buffers
// -------------------------------------------------------------------------

// Handles created once in uros_task at startup.
static rcl_subscription_t sub_cmd_vel;               // subscribes to /cmd_vel
static rcl_publisher_t    pub_odom;                  // publishes /odom

// Message buffers: static so we do not thrash the allocator every tick.
static geometry_msgs__msg__Twist        msg_cmd_vel; // ingress buffer for /cmd_vel
static nav_msgs__msg__Odometry          msg_odom;    // egress buffer for /odom

// UART port number passed to the transport as `args`. Storage lifetime must
// outlive the transport session, so it lives at file scope.
static size_t uart_port_num = UROS_UART_NUM;

// -------------------------------------------------------------------------
// RCCHECK: abort task on error. RCSOFTCHECK: log and continue.
// -------------------------------------------------------------------------

#define RCCHECK(fn)                                                            \
    { rcl_ret_t rc = fn;                                                       \
      if (rc != RCL_RET_OK) {                                                  \
          ESP_LOGE(TAG, "RCCHECK failed at %s:%d (%d)", __FILE__, __LINE__, (int)rc); \
          vTaskDelete(NULL);                                                   \
      } }

#define RCSOFTCHECK(fn)                                                        \
    { rcl_ret_t rc = fn;                                                       \
      if (rc != RCL_RET_OK) {                                                  \
          ESP_LOGW(TAG, "RCSOFTCHECK failed at %s:%d (%d)", __FILE__, __LINE__, (int)rc); \
      } }

// -------------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------------

// Push a new sample into a ring buffer and return the current arithmetic mean.
// Same helper as Stage 2. FILTER_WINDOW is small (5) so O(N) scan is cheap.
static float filter_push(float *buf, int *idx, float new_sample){
    buf[*idx] = new_sample;                          // overwrite the oldest slot
    *idx = (*idx + 1) % FILTER_WINDOW;               // advance the ring index
    float sum = 0.0f;                                // accumulator for the mean
    for (int i = 0; i < FILTER_WINDOW; i++) {
        sum += buf[i];                               // sum all slots
    }
    return sum / (float) FILTER_WINDOW;              // arithmetic mean
}

// Convert delta-counts and delta-microseconds to rad/s on the output shaft.
static float counts_to_rad_s(int64_t d_counts, int64_t d_us){
    if (d_us <= 0) return 0.0f;                      // guard against zero dt
    float revs = (float) d_counts / (float) ENCODER_COUNTS_PER_OUTPUT_REV;   // fraction of a revolution
    float rad = revs * 2.0f * (float) M_PI;                  // radians
    float seconds = (float) d_us * 1e-6f;                    // convert us to s
    return rad / seconds;                                     // rad/s
}

// Fill a builtin_interfaces__msg__Time with the current session-synced time.
// If micro-ROS is not connected to the agent yet, rmw_uros_epoch_nanos()
// returns 0 or a monotonic local value; either is acceptable for bringup.
static void fill_time_stamp(builtin_interfaces__msg__Time *stamp){
    int64_t now_ns = rmw_uros_epoch_nanos();         // agent-synced ns since epoch (if synced)
    stamp->sec     = (int32_t)  (now_ns / 1000000000LL);   // whole seconds
    stamp->nanosec = (uint32_t) (now_ns % 1000000000LL);   // remainder ns
}

// -------------------------------------------------------------------------
// /cmd_vel callback: runs inside uros_task executor
// -------------------------------------------------------------------------

// Callback signature is fixed by rclc executor. `msg_in` points at our
// pre-registered ingress buffer (`msg_cmd_vel`), which the executor has
// already populated from the deserialized incoming message.
static void cmd_vel_callback(const void *msg_in){
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *) msg_in;

    // Twist fields we care about: linear.x (forward m/s), angular.z (yaw rad/s).
    // Ignore everything else: differential drive cannot use linear.y or roll/pitch.
    float v = (float) msg->linear.x;                 // ROS uses float64, cast to float for kinematics
    float w = (float) msg->angular.z;

    // Convert body velocity to per-wheel angular velocity setpoints.
    float omega_left  = 0.0f;
    float omega_right = 0.0f;
    kin_cmd_vel_to_wheel_omegas(v, w, &omega_left, &omega_right);

    // Publish to the shared atomics. Motor A = left, Motor B = right.
    atomic_store(&g_setpoint_a, omega_left);
    atomic_store(&g_setpoint_b, omega_right);

    // Refresh the watchdog timestamp. The control task compares this to
    // esp_timer_get_time() every iteration and forces zero if stale.
    atomic_store(&g_last_cmd_vel_us, esp_timer_get_time());
}

// -------------------------------------------------------------------------
// /odom + TF timer callback: runs inside uros_task executor at ~30 Hz
// -------------------------------------------------------------------------

// Read the encoder snapshot, integrate pose, publish /odom and TF.
static void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time){
    (void) timer;                                    // unused, we only have one timer
    (void) last_call_time;                           // we use our own timing

    // Read the snapshot atomically. Three loads are not one transaction, so
    // in principle we could see counts from cycle N and time from cycle N+1.
    // At worst, ~10 ms of temporal skew. Invisible to Nav2.
    int64_t counts_a = atomic_load(&g_snapshot_counts_a);
    int64_t counts_b = atomic_load(&g_snapshot_counts_b);
    int64_t time_us  = atomic_load(&g_snapshot_time_us);

    if (!odom_primed) {
        // First callback: initialize the "last" values and skip integration.
        // No delta yet, so no meaningful pose update possible.
        odom_last_counts_a = counts_a;
        odom_last_counts_b = counts_b;
        odom_last_time_us  = time_us;
        odom_primed        = true;
        return;                                      // skip this tick
    }

    // Compute deltas since the last integration.
    int64_t d_counts_a = counts_a - odom_last_counts_a;    // left wheel delta
    int64_t d_counts_b = counts_b - odom_last_counts_b;    // right wheel delta
    int64_t d_us       = time_us  - odom_last_time_us;     // time delta in microseconds

    // Save for next iteration.
    odom_last_counts_a = counts_a;
    odom_last_counts_b = counts_b;
    odom_last_time_us  = time_us;

    if (d_us <= 0) return;                           // safety: no time has passed
    float dt = (float) d_us * 1e-6f;                 // seconds

    // Wheel arc lengths at the ground.
    float ds_left  = kin_counts_to_metres(d_counts_a);   // metres, left wheel
    float ds_right = kin_counts_to_metres(d_counts_b);   // metres, right wheel

    // Body-frame deltas.
    float ds     = 0.0f;                             // forward distance
    float dtheta = 0.0f;                             // yaw change
    kin_wheel_deltas_to_body(ds_left, ds_right, &ds, &dtheta);

    // Midpoint pose integration. Using theta + dtheta/2 is more accurate than
    // Euler for curved paths at moderate turn rates. For the paper, this is
    // the "exact integration for constant curvature" approximation.
    double theta_mid = odom_theta + (double) dtheta * 0.5;
    odom_x     += (double) ds * cos(theta_mid);      // update x in odom frame
    odom_y     += (double) ds * sin(theta_mid);      // update y in odom frame
    odom_theta += (double) dtheta;                   // update yaw

    // Twist (published as instantaneous velocity in the /odom message).
    odom_vx = (double) ds / (double) dt;             // forward velocity, m/s
    odom_wz = (double) dtheta / (double) dt;         // yaw rate, rad/s

    // Yaw as a quaternion (2D: only qz and qw are non-zero).
    double qz = sin(odom_theta * 0.5);
    double qw = cos(odom_theta * 0.5);

    // Populate the /odom message.
    fill_time_stamp(&msg_odom.header.stamp);         // synced timestamp for the header
    msg_odom.pose.pose.position.x    = odom_x;
    msg_odom.pose.pose.position.y    = odom_y;
    msg_odom.pose.pose.position.z    = 0.0;
    msg_odom.pose.pose.orientation.x = 0.0;
    msg_odom.pose.pose.orientation.y = 0.0;
    msg_odom.pose.pose.orientation.z = qz;
    msg_odom.pose.pose.orientation.w = qw;
    msg_odom.twist.twist.linear.x    = odom_vx;      // forward velocity
    msg_odom.twist.twist.linear.y    = 0.0;
    msg_odom.twist.twist.linear.z    = 0.0;
    msg_odom.twist.twist.angular.x   = 0.0;
    msg_odom.twist.twist.angular.y   = 0.0;
    msg_odom.twist.twist.angular.z   = odom_wz;      // yaw rate

    RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));
}

// -------------------------------------------------------------------------
// control_task: 100 Hz PID loop. Structurally identical to Stage 2, plus
// atomic setpoint reads at the top and atomic snapshot writes at the bottom.
// -------------------------------------------------------------------------

static void control_task(void *arg){
    (void) arg;

    // Prime the "last" values so the first iteration has a real dt.
    int64_t last_counts_a = encoder_read_counts(ENCODER_A);
    int64_t last_counts_b = encoder_read_counts(ENCODER_B);
    int64_t last_time_us  = esp_timer_get_time();

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    while (1) {
        vTaskDelayUntil(&last_wake, period);         // wait for next 10 ms slot

        // Timestamp first, then counts. Sub-millisecond misalignment at 100 Hz
        // is inconsequential compared to encoder update rate.
        int64_t now_us   = esp_timer_get_time();
        int64_t counts_a = encoder_read_counts(ENCODER_A);
        int64_t counts_b = encoder_read_counts(ENCODER_B);

        // Read the shared setpoints atomically.
        float sp_a = atomic_load(&g_setpoint_a);
        float sp_b = atomic_load(&g_setpoint_b);
        int64_t last_cmd_us = atomic_load(&g_last_cmd_vel_us);

        // /cmd_vel watchdog: if no message ever received (last_cmd_us == 0) or
        // the last message is older than the timeout, force zero setpoints.
        // The coast bypass below will then reset both PIDs.
        if (last_cmd_us == 0 || (now_us - last_cmd_us) > CMD_VEL_TIMEOUT_US) {
            sp_a = 0.0f;
            sp_b = 0.0f;
        }

        // Per-motor deltas.
        int64_t d_counts_a = counts_a - last_counts_a;
        int64_t d_counts_b = counts_b - last_counts_b;
        int64_t d_us       = now_us   - last_time_us;

        // Save "last" for next iteration.
        last_counts_a = counts_a;
        last_counts_b = counts_b;
        last_time_us  = now_us;

        // Raw velocities (LSB dither present, ~0.07 rad/s std).
        float meas_a_raw = counts_to_rad_s(d_counts_a, d_us);
        float meas_b_raw = counts_to_rad_s(d_counts_b, d_us);

        // Moving-average filter applied BEFORE the PID.
        float meas_a_filt = filter_push(filt_buf_a, &filt_idx_a, meas_a_raw);
        float meas_b_filt = filter_push(filt_buf_b, &filt_idx_b, meas_b_raw);

        float dt = (float) d_us * 1e-6f;             // seconds

        // Per-motor coast bypass. Independent per motor because rotation-in-place
        // commands can have one small and one large setpoint (though typically
        // both are large in opposite directions and neither triggers coast).
        float duty_a;
        float duty_b;
        if (fabsf(sp_a) < COAST_EPS_RAD_S) {
            duty_a = 0.0f;
            pid_reset(&pid_a);
        } else {
            duty_a = pid_update(&pid_a, sp_a, meas_a_filt, dt);
        }
        if (fabsf(sp_b) < COAST_EPS_RAD_S) {
            duty_b = 0.0f;
            pid_reset(&pid_b);
        } else {
            duty_b = pid_update(&pid_b, sp_b, meas_b_filt, dt);
        }

        // Command the motors.
        motor_set_duty(MOTOR_A, duty_a);
        motor_set_duty(MOTOR_B, duty_b);

        // Publish encoder snapshot for the uros_task to integrate.
        atomic_store(&g_snapshot_counts_a, counts_a);
        atomic_store(&g_snapshot_counts_b, counts_b);
        atomic_store(&g_snapshot_time_us, now_us);
    }
}

// -------------------------------------------------------------------------
// uros_task: micro-ROS executor. Sets up node, subscriptions, publishers,
// and the odom timer, then spins forever.
// -------------------------------------------------------------------------

static void uros_task(void *arg){
    (void) arg;

    // Populate the frame_id strings once so the rosidl runtime knows where the
    // data lives. `data` points at our static buffer, `size` is the string
    // length (no null terminator), `capacity` is the buffer size including the
    // null slot. These pointers stay valid for the lifetime of the program.
    msg_odom.header.frame_id.data     = frame_odom;
    msg_odom.header.frame_id.size     = strlen(frame_odom);
    msg_odom.header.frame_id.capacity = sizeof(frame_odom);
    msg_odom.child_frame_id.data      = frame_base;
    msg_odom.child_frame_id.size      = strlen(frame_base);
    msg_odom.child_frame_id.capacity  = sizeof(frame_base);

    // Set conservative diagonal covariances for pose and twist. The 6x6
    // covariance is stored in row-major order: index = row*6 + col.
    // Placeholder values, real numbers will be measured for the paper.
    // Diagonal: [x-x, y-y, z-z, roll-roll, pitch-pitch, yaw-yaw].
    msg_odom.pose.covariance[0]  = 0.01;             // sigma_x^2  (10 cm 1-sigma)
    msg_odom.pose.covariance[7]  = 0.01;             // sigma_y^2
    msg_odom.pose.covariance[14] = 1e6;              // z is unused: huge variance rejects updates
    msg_odom.pose.covariance[21] = 1e6;              // roll unused
    msg_odom.pose.covariance[28] = 1e6;              // pitch unused
    msg_odom.pose.covariance[35] = 0.03;             // sigma_yaw^2 (~10 deg 1-sigma)

    msg_odom.twist.covariance[0]  = 0.01;            // sigma_vx^2
    msg_odom.twist.covariance[7]  = 1e6;             // no lateral motion (differential drive)
    msg_odom.twist.covariance[14] = 1e6;
    msg_odom.twist.covariance[21] = 1e6;
    msg_odom.twist.covariance[28] = 1e6;
    msg_odom.twist.covariance[35] = 0.01;            // sigma_wz^2

    
    // micro-ROS initialization.
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t  support;

        // Set DDS domain ID to 94 explicitly. rmw_microxrcedds hardcodes
    // the default to 0 regardless of the RMW_UXRCE_DEFAULT_DOMAIN_ID
    // CMake flag; must be overridden here to match the Pi's domain.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 94));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rcl_init_options_fini(&init_options));

    
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "idefix_base", "", &support));

    // /cmd_vel subscription (best-effort, matches teleop_twist_keyboard and Nav2 defaults).
    RCCHECK(rclc_subscription_init_default(
        &sub_cmd_vel,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // /odom publisher (default QoS, reliable).
    RCCHECK(rclc_publisher_init_default(
        &pub_odom,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"));

    // 30 Hz odometry timer. The rclc_timer_init_default2 variant with
    // autostart=true starts the timer immediately after creation.
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(ODOM_TIMER_PERIOD_MS),
        odom_timer_callback,
        true));

    // Executor with capacity for 2 handles: the subscription and the timer.
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // Try to sync our clock to the agent's clock once at startup. If the sync
    // fails (agent slow to respond), we fall through and use local time. It
    // is safe to re-attempt periodically later; for bringup, once is enough.
    rmw_uros_sync_session(1000);                     // 1000 ms timeout

    ESP_LOGI(TAG, "micro-ROS executor ready, spinning");

    // Spin forever. spin_some processes any ready subscriptions and timers,
    // waiting up to 10 ms for new work. The trailing vTaskDelay yields the
    // CPU briefly so lower-priority tasks can run.
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// -------------------------------------------------------------------------
// app_main: bring up hardware, install the transport, launch the tasks.
// -------------------------------------------------------------------------

void app_main(void){
    ESP_LOGI(TAG, "Idefix firmware starting (Stage 3: micro-ROS integration)");

    // Hardware bring-up in the same order as Stage 2.
    ESP_ERROR_CHECK(motor_init_all());               // TB6612 configured, STBY high, duty = 0
    ESP_ERROR_CHECK(encoder_init_all());             // PCNT units running, watch-points armed

    // PID initialization: same gains as validated Stage 2. Kept identical so
    // Stage 2 validation carries over unchanged.
    pid_init(&pid_a, PID_KP, PID_KI, PID_KD, PID_OUT_MIN, PID_OUT_MAX, PID_INT_MIN, PID_INT_MAX);
    pid_init(&pid_b, PID_KP, PID_KI, PID_KD, PID_OUT_MIN, PID_OUT_MAX, PID_INT_MIN, PID_INT_MAX);

    // Install the micro-ROS UART transport BEFORE creating uros_task.
    // rmw_uros_set_custom_transport is a pure setter: it does not touch the
    // UART hardware itself; the actual UART bring-up happens later inside
    // esp32_serial_open() when the micro-ROS session first connects.
#if defined(CONFIG_MICRO_ROS_ESP_UART_TRANSPORT)
    rmw_uros_set_custom_transport(
        true,                                        // framing required for byte-oriented UART
        (void *) &uart_port_num,                     // context: pointer to our UART port number
        esp32_serial_open,                           // callbacks provided by esp32_serial_transport.c
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read);
#else
    #error "micro-ROS transport not configured: enable 'Micro XRCE-DDS over UART' in menuconfig (defines CONFIG_MICRO_ROS_ESP_UART_TRANSPORT)"
#endif

    // Launch control_task (100 Hz PID) on core 1. Same core and priority as Stage 2.
    xTaskCreatePinnedToCore(
        control_task,                                // task function
        "control",                                   // debug name
        4096,                                        // stack: proven sufficient in Stage 2
        NULL,                                        // no argument
        5,                                           // priority: higher than uros_task, protects the loop timing
        NULL,                                        // no handle needed
        1);                                          // core 1 (deterministic isolation from system stuff)

    // Launch uros_task (micro-ROS executor) on core 0. Priority 3 so it never
    // pre-empts control_task. Stack 16000 bytes because micro-ROS internals
    // (rcl, rmw, XRCE-DDS) are stack-hungry, and 16 kB is the shipped example
    // default. Downsizing risks silent stack-overflow hangs during setup.
    xTaskCreatePinnedToCore(
        uros_task,                                   // task function
        "uros",                                      // debug name
        16000,                                       // stack: matches upstream example
        NULL,                                        // no argument
        3,                                           // priority: below control_task
        NULL,                                        // no handle needed
        0);                                          // core 0

    ESP_LOGI(TAG, "tasks launched: control_task on core 1 prio 5, uros_task on core 0 prio 3");
}