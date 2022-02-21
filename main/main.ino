#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include "robot.h"

/* ~~~ Initialise Global Variables ~~~ */

// Process Selectors
// Mode 0 is search, Mode 1 is return, Mode 2 is search white square
int mode;
int block_count = 0;
bool not_finished = false;


// Define global variables

// Navigation
int white_left = 0;
int white_right = 0;
int white_side = 0;
int cross_count = 0;
long cross_duration = -3000;
long turn_duration;


// Button Variables
int reading;
long last_button_time = 0;
long debounceDelay = 500;
int last_button_reading = 1;


// Block-related
int block_type; // 0 is Coarse, 1 is Fine
float IR_block;
float ultrasonic;
long duration;
float ultra_distance;


// LEDs
long yellowLED_pin_duration;
int redLED = 0;
int yellowLED = 0;
int greenLED = 0;


// Third Block
float IR_scans[2] = {150, 0};
long t_from_square;
long t_turn;
long t_turn_2;


// Init Motor and Servo
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *MotorL = AFMS.getMotor(L_motor);
Adafruit_DCMotor *MotorR = AFMS.getMotor(R_motor);

Servo myservo;



/* ~~~ Initialisation ~~~ */
void setup() {
    // Button 
    pinMode(button_pin, INPUT);
    
    // Navigation
    pinMode(optopin_L, INPUT);
    pinMode(optopin_R, INPUT);
    
    // Block-related
    pinMode(IR_pin, INPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    myservo.attach(servo_pin, servo_min, servo_max);
    myservo.write(servo_min);
    delay(1000);
    
    // Motor
    AFMS.begin(30);

    // LEDs
    pinMode(redLED_pin, OUTPUT);
    pinMode(yellowLED_pin, OUTPUT);
    pinMode(greenLED_pin, OUTPUT);
    digitalWrite(redLED_pin, redLED);
    digitalWrite(yellowLED_pin, yellowLED);
    digitalWrite(greenLED_pin, greenLED);

    // Troubleshooting
    Serial.begin(9600);
}



/* ~~~ LEDs ~~~ */
void yellow_LED_flash() {
    if (millis() - yellowLED_pin_duration > 250) {
            yellowLED = !yellowLED;
            digitalWrite(yellowLED_pin, yellowLED);
            yellowLED_pin_duration = millis();
        }
}

void yellow_LED_OFF() {
    digitalWrite(yellowLED_pin, 0);
}

void block_LED_OFF() {
    digitalWrite(redLED_pin, 0);
    digitalWrite(greenLED_pin, 0);
}



/* ~~~ NAVIGATION ~~~ */

// White Line Following
void get_readings() {
    white_left = digitalRead(optopin_L);
    white_right = digitalRead(optopin_R);
    white_side = digitalRead(optopin_S);
}

void white_line_following(){
    get_readings();
    yellow_LED_flash();
    
    // Increase cross_count
    if (white_side == 1) {
        if (millis() - cross_duration > 3000) {
            cross_count += 1;
            cross_duration = millis();
        }
      }
    
    // Going straight
    if (white_left == 1 && white_right == 1) {
        MotorL->run(FORWARD);
        MotorL->setSpeed(straight_L);
        MotorR->run(FORWARD);
        MotorR->setSpeed(straight_R);
    }
    else if (white_left == 0 && white_right == 0) {
        MotorL->run(FORWARD);
        MotorL->setSpeed(straight_L);
        MotorR->run(FORWARD);
        MotorR->setSpeed(straight_R);
      }

    // Detects white on left side
    else if (white_left == 1) {
        MotorL->run(FORWARD);
        MotorL->setSpeed(dynamic_turn_slow_L);
        MotorR->run(FORWARD);
        MotorR->setSpeed(dynamic_turn_fast);
      }

    // Detects white on right side
    else if (white_right == 1) {
        MotorL->run(FORWARD);
        MotorL->setSpeed(dynamic_turn_fast);
        MotorR->run(FORWARD);
        MotorR->setSpeed(dynamic_turn_slow_R);
      }

}

// Set Rotations and Translations
void turn_right_90() {
    turn_duration = millis();
    while ((millis() - turn_duration) < 2000) {
        yellow_LED_flash();
        MotorL->setSpeed(static_turn_L);
        MotorL->run(FORWARD);
        MotorR->setSpeed(static_turn_R);
        MotorR->run(BACKWARD);
    }
    yellow_LED_OFF();
    MotorL->setSpeed(0);
    MotorR->setSpeed(0);
}

void turn_left_90() {
    turn_duration = millis();
    while ((millis() - turn_duration) < 2000) {
        yellow_LED_flash();
        MotorL->setSpeed(static_turn_L);
        MotorL->run(BACKWARD);
        MotorR->setSpeed(static_turn_R);
        MotorR->run(FORWARD);
    }
    yellow_LED_OFF();
    MotorL->setSpeed(0);
    MotorR->setSpeed(0);
}

void move_forward() {
    turn_duration = millis();
    while ((millis() - turn_duration) < 1000) {
        yellow_LED_flash();
        MotorL->setSpeed(static_turn_L);
        MotorL->run(FORWARD);
        MotorR->setSpeed(static_turn_R);
        MotorR->run(FORWARD);
    }
    yellow_LED_OFF();
    MotorL->setSpeed(0);
    MotorR->setSpeed(0);
}

void move_backward() {
    turn_duration = millis();
    while ((millis() - turn_duration) < 1000) {
        yellow_LED_flash();
        MotorL->setSpeed(static_turn_L);
        MotorL->run(BACKWARD);
        MotorR->setSpeed(static_turn_R);
        MotorR->run(BACKWARD);
    }
    yellow_LED_OFF();
    MotorL->setSpeed(0);
    MotorR->setSpeed(0);
}


/* ~~~ BLOCK-RELATED ~~~ */
void grab() {
    yellow_LED_OFF();
    myservo.write(servo_max);
    delay(1000);
}

void release_block() {
    yellow_LED_OFF();
    block_LED_OFF();
    myservo.write(servo_min);
    delay(1000);
}

void identify_block() {
    for (int i = 1; i <ultrasonic_num; i++) {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        // Sets the trigPin on HIGH state for 10 micro seconds
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        // Reads the echoPin, returns the sound wave travel time in microseconds
        duration = pulseIn(echoPin, HIGH);
        // Calculating the distance
        duration = duration*0.034/2;
        ultra_distance += (duration - ultra_distance) / i;
        delay(20);
    }
    
    // Prints the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.println(ultra_distance);

    if (ultra_distance > ultrasonic_threshold) {
        Serial.println("This is the coarse block");
        block_type = 0;
        digitalWrite(redLED_pin, 1);
    }
    else {
        Serial.println("This is the fine block");
        block_type = 1;
        digitalWrite(greenLED_pin, 1);
      }
    ultra_distance = 0;
}

float scan_IR() {
    float IR_v = analogRead(IR_pin)*0.0048828125;
    IR_v = pow(IR_v, -1.184) * 27.675;
    return IR_v;
}

void sweep() {
    turn_left_90();
    float IR_value = scan_IR();
    IR_scans[0] = IR_value;
    t_turn = millis();
    while ((millis() - t_turn) < 5200) {
        yellow_LED_flash();
        MotorL->setSpeed(increment_L);
        MotorL->run(FORWARD);
        MotorR->setSpeed(increment_R);
        MotorR->run(BACKWARD);
        IR_value = scan_IR();
        if (IR_value < IR_scans[0]){
            IR_scans[0] = IR_value;
        }
    }
    t_turn = millis() - t_turn;
    MotorL->setSpeed(0);
    MotorR->setSpeed(0);
}

void sweep_offset_correction() {
    float IR_value = scan_IR();
    turn_duration = millis();
    while ((millis() - turn_duration) < (sweep_offset * 10 / IR_value)) {
        yellow_LED_flash();
        MotorL->setSpeed(increment_L);
        MotorL->run(BACKWARD);
        MotorR->setSpeed(increment_R);
        MotorR->run(FORWARD);
    }
    MotorL->setSpeed(0);
    MotorR->setSpeed(0);
}

void turn_to_block() {
    float IR_value = scan_IR();
    t_turn_2 = millis();
    while (IR_value > (IR_scans[0] + 2)) {
        Serial.println(IR_value);
        Serial.println(IR_scans[0]);
        Serial.println();
        yellow_LED_flash();
        MotorL->setSpeed(increment_L);
        MotorL->run(BACKWARD);
        MotorR->setSpeed(increment_R);
        MotorR->run(FORWARD);
        IR_value = scan_IR();
    }
    sweep_offset_correction();
    t_turn_2 = millis() -  t_turn_2;
    MotorL->setSpeed(0);
    MotorR->setSpeed(0);
    yellow_LED_OFF();
}

void anti_turn_to_block() {
    double t_turn_3 = millis();
    while ((millis() - t_turn_3) < (t_turn - t_turn_2)) {
        yellow_LED_flash();
        MotorL->setSpeed(increment_L);
        MotorL->run(BACKWARD);
        MotorR->setSpeed(increment_R);
        MotorR->run(FORWARD);
    }
    MotorL->setSpeed(0);
    MotorR->setSpeed(0);
    turn_left_90();
    yellow_LED_OFF();
}

void detect_block(){
    IR_block = analogRead(IR_pin)*0.0048828125;
    IR_block = pow(IR_block, -1.184) * 27.675;
    Serial.println(IR_block);
  
    if (IR_block < IR_threshold) {
        sweep();
        turn_to_block();
        t_from_square = millis();
        float IR_v;
        IR_v = scan_IR();
        while (IR_v > 8) {
            yellow_LED_flash();
            IR_v = scan_IR();
            MotorL->setSpeed(static_turn_L);
            MotorL->run(FORWARD);
            MotorR->setSpeed(static_turn_R);
            MotorR->run(FORWARD);
            }
        t_from_square = millis() - t_from_square;
        MotorL->setSpeed(0);
        MotorR->setSpeed(0);
                
        move_forward();
        grab();
        block_count += 1;
        identify_block();
                
        long t_back = millis();
        while ((millis() - t_back) < t_from_square) {
            yellow_LED_flash();
            MotorL->setSpeed(straight_L);
            MotorL->run(BACKWARD);
            MotorR->setSpeed(straight_R);
            MotorR->run(BACKWARD);
            }
         MotorL->setSpeed(0);
         MotorR->setSpeed(0);
         anti_turn_to_block();
         mode = 1;
         cross_duration = millis();
        
  }
}






/* ~~~ MISC ~~~ */
// Robot finishes task and stops
void finished_program() {
    MotorL->setSpeed(0);
    MotorL->run(FORWARD);
    MotorR->setSpeed(0);
    MotorR->run(FORWARD);
    release_block();
    yellow_LED_OFF();
    cross_count = 0;
    mode = 0;
    block_count = 0;
  }





/* ~~~ MAIN LOOP ~~~ */
void loop() {
    // Button with debouncing
    reading = digitalRead(button_pin);
    Serial.println(reading);
        if (reading == 1) {
            if (millis() - last_button_time > debounceDelay) {
                if (last_button_reading == 0) {
                    not_finished = !not_finished;
                    Serial.println("Button pressed");
                 }
            }
        last_button_time = millis();
        }
        last_button_reading = reading;


    // Main Loop
    if (not_finished) {

        // Search algorithm
        if (mode == 0) {
            white_line_following();
            detect_block();
         }

        // Return and place block
        else if (mode == 1) {
            if (cross_count < 3) {
              cross_count = 3;
            }
            else if (cross_count == 3) {
              white_line_following();
            }
            else if (cross_count == 4){
                if (block_type == 0) {
                    // Adjust to be in line with cross and squares
                    move_forward();
                    move_forward();
                    turn_right_90();
                    move_forward();
                    release_block();
                    move_backward();
                    turn_right_90();
                }
                else if (block_type == 1) {
                    // Adjust to be in line with cross and squares
                    move_forward();
                    move_forward();
                    turn_left_90();
                    move_forward();
                    release_block();
                    move_backward();
                    turn_left_90();
                }
                
                cross_count = 0;
                cross_duration = millis();
                
                if (block_count == 1) {
                        mode = 0;
                }
                else if (block_count == 2) {
                        mode = 2;
                }
                else if (block_count == 3) {
                    // Return to starting white square
                    turn_right_90();
                    turn_right_90();
                    while (cross_count < 1) {
                        white_line_following();
                    }
                    move_forward();
                    move_forward();
                    not_finished = false;
                }
                else {
                    finished_program();
                }
            }
            
        }

        // Search for 3rd block
        else if (mode == 2) {
          if (cross_count < 1) {
              white_line_following();
            }
          else if (cross_count == 1) {
                sweep();
                turn_to_block();
                t_from_square = millis();
                float IR_v;
                IR_v = scan_IR();
                while (IR_v > 8) {
                    yellow_LED_flash();
                    IR_v = scan_IR();
                    MotorL->setSpeed(static_turn_L);
                    MotorL->run(FORWARD);
                    MotorR->setSpeed(static_turn_R);
                    MotorR->run(FORWARD);
                    }
                t_from_square = millis() - t_from_square;
                MotorL->setSpeed(0);
                MotorR->setSpeed(0);
                        
                move_forward();
                grab();
                block_count += 1;
                identify_block();
                        
                long t_back = millis();
                while ((millis() - t_back) < t_from_square) {
                    yellow_LED_flash();
                    MotorL->setSpeed(straight_L);
                    MotorL->run(BACKWARD);
                    MotorR->setSpeed(straight_R);
                    MotorR->run(BACKWARD);
                    }
                MotorL->setSpeed(0);
                MotorR->setSpeed(0);
                anti_turn_to_block();

                mode = 1;
                cross_count = 3;
                }
            }
    }

    // Stops robot and resets counter variables
    else {
        finished_program();
    }
}
