#ifndef MOUSE_H__
#define MOUSE_H__

#include "QueMaze.h"

/* Motors */
#define left_motor_F   18
#define left_motor_B   19 
#define right_motor_F   5
#define right_motor_B   17

/*IRs*/
#define IR_1 25 
#define IR_2 33 
#define IR_3 32 
#define IR_4 35 
#define IR_5 34 
#define IR_6 39

/*buzzer*/
#define buzzer 27

/*Dip Switches*/
#define Deep_1 16
#define Deep_2 4

/*Encoders*/
#define encoder_1 14 
#define encoder_2 26
#define MAX_NUM_OF_PULSES 2380

/*I2C communication protocols' pins */
#define I2C_SDAA 21 
#define I2C_SCLL 22

/* variable resistance to control speed*/
#define pot 36 

/*PWM configurations*/
#define PWM_FREQ 10000   
#define PWM_RES  8       

/* Serial monitor BuadRate*/
#define BuadRate        115200

/* Sensor thresholds */
#define FRONT_SENSOR_THRESHOLD 1888
#define SIDE_SENSOR_THRESHOLD 2333

// Movement control functions
void Robot_MoveForward(void);
void Robot_TurnRight(void);
void Robot_TurnLeft(void);
void Robot_TurnAround(void);
void Robot_StopMotors(void);

// Navigation functions
void Robot_NavigateToGoal1(void);
void Robot_NavigateToGoal2(void);

// Setup and loop functions
void setup(void);
void loop(void);

// Test functions
void TestSensors(void);
void TestTurns(void);
void TestMovement(void);

#endif