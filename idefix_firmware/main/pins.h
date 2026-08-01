/*
pins.h

Central pin assignment for the Idefix ESP32-S3 firmware.
Every GPIO number in the firmware lives here. No other file hardcodes a pin. 
*/

#pragma once

// Motor A (left wheel)
#define MOTOR_A_PWM_GPIO   15   // PWMA -> TB6612FNG PWMA input 
#define MOTOR_A_IN1_GPIO   17   // AIN1 -> TB6612FNG AIN1 
#define MOTOR_A_IN2_GPIO   16   // AIN2 -> TB6612FNG AIN2 

// Motor B (right wheel) 
#define MOTOR_B_PWM_GPIO   11   // PWMB -> TB6612FNG PWMB input 
#define MOTOR_B_IN1_GPIO   9    // BIN1 -> TB6612FNG BIN1 
#define MOTOR_B_IN2_GPIO   10   // BIN2 -> TB6612FNG BIN2 

// TB6612FNG driver control 
#define TB6612_STBY_GPIO   18   // STBY: LOW = driver off, HIGH = enabled. Firmware must drive this LOW at boot, then HIGH only after all motor pins are configured. This is the master safety line for the drivetrain.

// Encoder A (paired with Motor A) 
#define ENCODER_A_CHA_GPIO  4   // Encoder A channel A (quadrature) C1U1
#define ENCODER_A_CHB_GPIO  5   // Encoder A channel B (quadrature) C2U1

// Encoder B (paired with Motor B) 
#define ENCODER_B_CHA_GPIO 13   // Encoder B channel A (quadrature) C1U2
#define ENCODER_B_CHB_GPIO 14   // Encoder B channel B (quadrature) C2U2
