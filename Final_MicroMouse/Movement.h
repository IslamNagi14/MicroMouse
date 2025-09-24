#ifndef MOVEMENT_H__
#define MOVEMENT_H__

#include "StdTypes.h"
#include "QueMaze.h"

// Movement control functions
void Robot_StopMotors(void);
void Robot_MoveForward(void);
void Robot_TurnAround(void);
void Robot_TurnRight(void);
void Robot_TurnLeft(void);
void Robot_AdjustForWalls(RobotSensors_t sensors);
// ===== Read Yaw in 0-360 range =====
float readYaw(void) ;

// ===== MPU6050 initialization =====
void MPU6050Initialization(void);
// Encoder functions
void ENRight_ISR(void);
void ENLeft_ISR(void);

// External encoder counters
extern u16 ENLeft_Counter;
extern u16 ENRight_Counter;

#endif