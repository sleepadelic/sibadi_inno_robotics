#define LR_ENCODER_PIN 13 // ODOM from left motor
#define RR_ENCODER_PIN 8  // ODOM from right motor

#define R_DR_PWM 6 // right motor pwm pin
#define R_DR_DIR 7 // right motor direction pin
#define L_DR_PWM 5
#define L_DR_DIR 4

#define DR_PWM_MIN 120
#define DR_PWM_MAX 250

#define WHEELRADIUS 0.3 //m

#define RELAY_CONTROLLER 9  //
#define RELAY_PWR_SUPPLY 12 // Реле вентиляторов и т.п.

#define RELAY_GEN_STARTER 11 //3 sec HIGH to start
#define RELAY_GEN_STOPPER 10 //set LOW to stop generator
#define GEN_START_TIME 3     //seconds

#define DEBUG_PIN A1
#define DEBUG_IS_ENABLE false