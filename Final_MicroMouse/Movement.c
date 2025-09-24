#include "Movement.h"
#include "mouse.h"
#include <Arduino.h>
#include <BluetoothSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

//extern BluetoothSerial MySerial;
extern s8 robot_current_row;
extern s8 robot_current_col;
extern Orientation_t robot_current_orientation;
extern u8 speed;
extern f32 currentGyroAngle;
extern bool turningRight;
extern bool turningLeft;
extern f32 turnTargetAngle;

// encoder counters
u16 ENLeft_Counter = 0;
u16 ENRight_Counter = 0;
//extern int EN ;

// bool hasTurned = false; // flag to ensure single rotation
// 
//===== MPU6050 =====
// MPU6050 mpu;
// 
//===== MPU variables =====
// bool dmpReady = false;
// uint8_t devStatus;
// uint16_t packetSize;
// uint8_t fifoBuffer[64];
// Quaternion q;
// VectorFloat gravity;
// float ypr[3];
// 
//===== MPU6050 initialization =====
// void  MPU6050Initialization(void)
// {
    // Wire.begin();
    // mpu.initialize();
// 
    // MySerial.println(mpu.testConnection() ? "MPU6050 connected" : "MPU6050 connection failed");
// 
    // devStatus = mpu.dmpInitialize();
    // mpu.setXGyroOffset(220);
    // mpu.setYGyroOffset(76);
    // mpu.setZGyroOffset(-85);
    // mpu.setZAccelOffset(1788);
// 
    // if (devStatus == 0) {
        // mpu.CalibrateAccel(6);
        // mpu.CalibrateGyro(6);
        // mpu.PrintActiveOffsets();
        // mpu.setDMPEnabled(true);
        // 
        // packetSize = mpu.dmpGetFIFOPacketSize();
        // dmpReady = true;
        // MySerial.println("DMP ready!");
    // } 
    // else 
    // {
        // MySerial.println("DMP Initialization failed!");
    // }
// } 
// 
//===== Read Yaw in 0-360 range =====
// float readYaw() 
// {
    // if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        // mpu.dmpGetQuaternion(&q, fifoBuffer);
        // mpu.dmpGetGravity(&gravity, &q);
        // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
// 
        // float yawDeg = ypr[0] * 180.0 / M_PI;
        // if (yawDeg < 0) yawDeg += 360.0;
        // return yawDeg;
    // }
    // return -1; // invalid reading
// }

/****** Enhanced Turn Control Functions with DMP Yaw ******/
void Robot_TurnLeft(void) 
{

      analogWrite(left_motor_F, 0 );
  analogWrite(left_motor_B, 200 );
  analogWrite(right_motor_F, 200 );
  analogWrite(right_motor_B, 0 );
delay(280);
  analogWrite(left_motor_F, 0 );
  analogWrite(left_motor_B, 0 );
  analogWrite(right_motor_F, 0 );
  analogWrite(right_motor_B, 0 );
  // float yaw;
  // MySerial.println("=== STARTING 90° LEFT TURN WITH DMP ===");
  // 
  // while(1)
  // {
  //     if (!dmpReady || hasTurned) break;
  //     yaw = readYaw();
  //     if (yaw >= 0) 
  //     { // valid reading
  //         MySerial.print("Yaw: ");
  //         MySerial.println(yaw);


  //         // left
  //        if (yaw >= (225) && yaw <= (240)) {
  //          Robot_StopMotors();
  //          hasTurned = true; // stop further movement
  //          MySerial.println("Target angle reached, motors stopped.");
  //      }
  //       else 
  //       {
  //         
  //         // keep turning
  //         analogWrite(left_motor_F, 75);
  //         analogWrite(left_motor_B, LOW);
  //         analogWrite(right_motor_F, LOW);
  //         analogWrite(right_motor_B, 75);
  //      }
  //     }
  // }

  // hasTurned = false;

    robot_current_orientation = (Orientation_t)((robot_current_orientation + 1) % 4);// 3-->1

    // MySerial.print("Now facing: ");
    // switch(robot_current_orientation)
    //  {
    //     case ORIENTATION_NORTH: MySerial.println("NORTH"); break;
    //     case ORIENTATION_EAST: MySerial.println("EAST"); break;
    //     case ORIENTATION_SOUTH: MySerial.println("SOUTH"); break;
    //     case ORIENTATION_WEST: MySerial.println("WEST"); break;
    // }
}

void Robot_TurnRight(void) 
{
   // MySerial.println("=== STARTING 90° RIGHT TURN WITH DMP ===");
   // float yaw ;
   // while(1)
   // {
   //     if (!dmpReady || hasTurned) break;
   //     yaw = readYaw();
   //     if (yaw >= 0) 
   //     { // valid reading
   //         MySerial.print("Yaw: ");
   //         MySerial.println(yaw);
//
//
   //         // 90 Right 
   //         if (yaw >= (45) && yaw <= (50)) 
   //         {
   //           Robot_StopMotors();
   //           hasTurned = true; // stop further movement
   //           MySerial.println("Target angle reached, motors stopped.");
   //         } 
   //         else
   //         {
   //           // keep turning
   //           analogWrite(left_motor_F, 75);
   //         analogWrite(left_motor_B, LOW);
   //         analogWrite(right_motor_F, LOW);
   //         analogWrite(right_motor_B, 75);
   //         }
   //     }
   // }
//
   // hasTurned = false;

   analogWrite(left_motor_F, 200 );
  analogWrite(left_motor_B, 0 );
  analogWrite(right_motor_F, 0 );
  analogWrite(right_motor_B, 200 );
  delay(290);
  analogWrite(left_motor_F, 0 );
  analogWrite(left_motor_B, 0 );
  analogWrite(right_motor_F, 0 );
  analogWrite(right_motor_B, 0 );
    robot_current_orientation = (Orientation_t)((robot_current_orientation + 3) % 4);//1-->3
    // MySerial.print("Now facing: ");
    // switch(robot_current_orientation) 
    // {
    //     case ORIENTATION_NORTH: MySerial.println("NORTH"); break;
    //     case ORIENTATION_EAST: MySerial.println("EAST"); break;
    //     case ORIENTATION_SOUTH: MySerial.println("SOUTH"); break;
    //     case ORIENTATION_WEST: MySerial.println("WEST"); break;
    // }
}

void Robot_TurnAround(void) 
{
   // MySerial.println("Turning around 180 degrees with DMP");
   // float yaw  = 0;
   //  while(1)
   // {
   //     if (!dmpReady || hasTurned) break;
   //     yaw = readYaw();
   //     if (yaw >= 0) 
   //     { // valid reading
   //         MySerial.print("Yaw: ");
   //         MySerial.println(yaw);
//
   //         //180
   //         if (yaw >= (45*3) && yaw <= (50*3)) 
   //       {
   //           Robot_StopMotors();
   //           hasTurned = true; // stop further movement
   //           MySerial.println("Target angle reached, motors stopped.");
   //       } 
   //       else 
   //       {
   //          // keep turning
   //         analogWrite(left_motor_F, 75);
   //         analogWrite(left_motor_B, LOW);
   //         analogWrite(right_motor_F, LOW);
   //         analogWrite(right_motor_B, 75);
   //       }
   //     }
   // }
//
   // hasTurned = false;
analogWrite(left_motor_F, LOW );
  analogWrite(left_motor_B, 200 );
  analogWrite(right_motor_F, 200 );
  analogWrite(right_motor_B, LOW );
  delay(775);
  analogWrite(left_motor_F, 0 );
  analogWrite(left_motor_B, 0 );
  analogWrite(right_motor_F, 0 );
  analogWrite(right_motor_B, 0 );
    robot_current_orientation = (Orientation_t)((robot_current_orientation + 2) % 4);
}

/****** Movement Functions ******/
void Robot_MoveForward(void) 
{
    attachInterrupt(digitalPinToInterrupt(encoder_1), ENRight_ISR, FALLING);// enable interrupt
    attachInterrupt(digitalPinToInterrupt(encoder_2), ENLeft_ISR, FALLING);// enable interrupt
    ENRight_Counter = 0;ENLeft_Counter = 0;
    analogWrite(left_motor_F, speed);
    analogWrite(left_motor_B, 0);
    analogWrite(right_motor_F, speed);
    analogWrite(right_motor_B, 0);

    while(ENRight_Counter < MAX_NUM_OF_PULSES && ENLeft_Counter < MAX_NUM_OF_PULSES)//polling untill moved the expected destance
    {
        if(ENRight_Counter <  ENLeft_Counter)
        {
            analogWrite(left_motor_F, speed-25);//40-->15
        }
        else if(ENRight_Counter >  ENLeft_Counter)
        {
            analogWrite(right_motor_F, speed-17);//40-->15
        }

        if(digitalRead(IR_1) || digitalRead(IR_6)) break;

       // MySerial.print("ENRigh_tCounter: ");MySerial.println(ENRight_Counter);
       // MySerial.print("ENLeft_Counter: ");MySerial.println(ENLeft_Counter);
    }
    detachInterrupt(digitalPinToInterrupt(encoder_1));// disable interrupt
    detachInterrupt(digitalPinToInterrupt(encoder_2));// disable interrupt

    analogWrite(left_motor_F, 0);
    analogWrite(left_motor_B, 0);
    analogWrite(right_motor_F, 0);
    analogWrite(right_motor_B, 0);

    switch(robot_current_orientation) 
    {
        case ORIENTATION_NORTH: robot_current_row--; break;
        case ORIENTATION_EAST: robot_current_col++; break;
        case ORIENTATION_SOUTH: robot_current_row++; break;
        case ORIENTATION_WEST: robot_current_col--; break;
    }
    
    // MySerial.print("Moved forward to ");
    // MySerial.print(robot_current_row);
    // MySerial.print(", ");
    // MySerial.println(robot_current_col);
    
}
void Robot_StopMotors() 
{
    analogWrite(left_motor_F, 0);
    analogWrite(left_motor_B, 0);
    analogWrite(right_motor_F, 0);
    analogWrite(right_motor_B, 0);
   // MySerial.println("Motors Stopped");
    delay(50);
}

// /****** Test Functions ******/
// void TestSensors(void) {
//     MySerial.println("=== SENSOR TEST ===");
//     for(int i = 0; i < 5; i++) {
//         RobotSensors_t sensors = Robot_ReadSensors();
//         delay(500);
//     }
// }

// void TestTurns(void) {
//     MySerial.println("=== TURN TEST ===");
//     Robot_TurnRight();
//     delay(1000);
//     Robot_TurnLeft();
//     delay(1000);
// }

// void TestMovement(void) {
//     MySerial.println("=== MOVEMENT TEST ===");
//     Robot_MoveForward();
//     delay(1000);
//     Robot_StopMotors();
// }

// Encoder ISR implementations
void ENRight_ISR(void) {
    ENRight_Counter++;
}

void ENLeft_ISR(void) {
    ENLeft_Counter++;
}