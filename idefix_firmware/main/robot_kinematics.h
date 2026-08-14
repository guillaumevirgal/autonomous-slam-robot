/*
robot_kinematics.h

Header-only differential-drive kinematics for Idefix.
 *
 * WHY HEADER-ONLY: the functions are small, called from exactly two
 * places (the /cmd_vel callback and the odom timer), and have no state.
 * Inlining costs nothing and keeps the whole robot kinematics story in
 * one file the paper can cite verbatim.
 *
 * FRAME CONVENTION (REP-103):
 *   Robot frame: X forward, Y left, Z up.
 *   linear.x  > 0  = move forward
 *   angular.z > 0  = turn counterclockwise viewed from above
 *
 * WHEEL ASSIGNMENT (locked, matches chassis):
 *   MOTOR_A drives the LEFT  wheel (+Y side of base_link).
 *   MOTOR_B drives the RIGHT wheel (-Y side of base_link).
 */

#pragma once

#include <math.h>
#include <stdint.h>
#include "encoder.h"


// Wheel geometry
#define WHEEL_RADIUS_M       0.022f      // metres
#define WHEEL_BASE_M         0.115f      // metres, center-to-center between drive wheels, refine empirically

// Convert a delta count from one encoder into arc length (metres) at the wheel contact patch. Positive counts -> positive arc.
static inline float kin_counts_to_metres(int64_t d_counts){
    float revs = (float)d_counts / (float)ENCODER_COUNTS_PER_OUTPUT_REV;   // fraction of a wheel revolution
    return 2.0f * (float)M_PI * WHEEL_RADIUS_M * revs;          // arc length = 2 * pi * r * counts / counts_per_rev
}




// Differential-drive inverse kinematics
//   (v>0, w=0):  both wheels spin forward at v/r         
//   (v=0, w>0):  right forward, left backward --> turn left
static inline void kin_cmd_vel_to_wheel_omegas(float v, float w, float *omega_left, float *omega_right){ // v --> body forward velocity (m/s), w --> body yaw rate (rad/s)
    float half_base = WHEEL_BASE_M * 0.5f;                      // L/2 precomputed
    *omega_left  = (v - w * half_base) / WHEEL_RADIUS_M;        // rad/s at left  wheel output shaft = (v - w * L/2) / r
    *omega_right = (v + w * half_base) / WHEEL_RADIUS_M;        // rad/s at right wheel output shaft = (v + w * L/2) / r
}


// Forward kinematics
static inline void kin_wheel_deltas_to_body(float ds_left, float ds_right, float *ds, float *dtheta){ // ds --> distance delta (m), dtheta --> yaw delta (rad) 
    *ds     = 0.5f * (ds_left + ds_right);                      // metres travelled by base_link origin = (ds_L + ds_R) / 2
    *dtheta = (ds_right - ds_left) / WHEEL_BASE_M;              // radians of yaw change (right forward = CCW) = (ds_R - ds_L) / L
}