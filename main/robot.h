#include "Arduino.h"

#ifndef ROBOT_H
#define ROBOT_H

/* ~~~~~ Defining Global Constants ~~~~~ */

/* ~ Main Program ~ */
// Button
#define button_pin 13


/* ~ Navigation ~ */
// Motor Controls
#define  L_motor 1
#define  R_motor 2

// Calibrated motor speeds
#define straight_L 250
#define straight_R 235
#define dynamic_turn_fast 255
#define dynamic_turn_slow_L 180
#define dynamic_turn_slow_R 110
#define static_turn_L 210
#define static_turn_R 205
#define increment_L 175
#define increment_R 150
#define turn_increment_num 30

// White line following
#define optopin_L 5
#define optopin_R 6
#define optopin_S 7



/* ~ Block Actions ~ */
#define IR_pin A0
#define servo_pin 9
#define trigPin 4
#define echoPin 3
#define servo_min 20
#define servo_max 160
#define ultrasonic_threshold 10
#define ultrasonic_num 6
#define sweep_offset 240
#define IR_threshold 12



/* ~ LEDs ~ */
#define redLED_pin 0
#define yellowLED_pin 1
#define greenLED_pin 2



#endif
