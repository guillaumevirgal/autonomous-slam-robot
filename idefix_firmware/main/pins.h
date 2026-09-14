/*
pins.h

Central pin assignment for the Idefix ESP32-S3 firmware.
Every GPIO number in the firmware lives here. No other file hardcodes a pin. 
*/

#pragma once

#include "driver/uart.h"


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

// Battery voltage sense: resistor divider from Vbat (top 100 kOhm, bottom
// 22 kOhm) into an ADC1 pin. Must stay on ADC1 (GPIO1-10): ADC2 is unusable
// while Wi-Fi is active on the ESP32-S3.
#define BATTERY_ADC_GPIO    6    // ADC1 channel 5 (GPIO1-10 map to ADC1_CH0-9, GPIO6 = CH5)

// Active buzzer, driven directly by a GPIO (no PWM/tone needed, the buzzer
// has its own oscillator).
#define BUZZER_GPIO         42



// USB-C connector is wired to an onboard USB-to-UART bridge chip
#define UROS_UART_NUM       UART_NUM_0    // hard-wired to the onboard USB-UART bridge chip
#define UROS_UART_TX_GPIO   43            // U0TXD, fixed by the DevKitC-1 board layout
#define UROS_UART_RX_GPIO   44            // U0RXD, fixed by the DevKitC-1 board layout
#define UROS_UART_BAUDRATE  921600        // raised from 460800: /odom (30 Hz, ~700 B/msg with two
                                          // 6x6 covariances) + /imu (100 Hz, ~316 B/msg) exceeds
                                          // 460800's ~46 KB/s raw capacity (~53 KB/s combined at
                                          // full rate); 921600 gives ~92 KB/s. Agent must launch
                                          // with matching -b 921600.