#include "../mouse.h"
#include "../QueMaze.h"
#include"../StdTypes.h"
#include "../Movement.h"
#include "../Movement.c"
#include "../FloodFill.c"
#include <Arduino.h>
#include <BluetoothSerial.h>

// BluetoothSerial MySerial;

// String inputString = "";   // لتجميع البيانات اللي جاية
// bool stringComplete = false;


/****** Robot State Tracking ******/
s8 robot_current_row = 15;
s8 robot_current_col = 0;
Orientation_t robot_current_orientation = ORIENTATION_NORTH;
u8 speed = 150;

RobotSensors_t global_sensors ;
bool front_blocked;
bool right_blocked;
bool left_blocked ;

// /****** Gyro Tracking Variables ******/
// f32 currentGyroAngle = 0;
// f32 targetAngle = 0.0;
// bool turningRight = false;
// bool turningLeft = false;
// f32 turnTargetAngle = 0.0;

/****** Setup Function ******/
void setup(void) 
{
    Serial.begin(BuadRate);
    //MySerial.begin("HEX_Mouse");
    //MySerial.println("Bluetooth Started!");

    // Initialize motor pins
    pinMode(left_motor_F, OUTPUT);
    pinMode(left_motor_B, OUTPUT);
    pinMode(right_motor_F, OUTPUT);
    pinMode(right_motor_B, OUTPUT);
    
    // Initialize buzzer

    // Initialize sensor pins
    pinMode(IR_1, INPUT);
    pinMode(IR_2, INPUT);
    pinMode(IR_3, INPUT);
    pinMode(IR_4, INPUT);
    pinMode(IR_5, INPUT);
    pinMode(IR_6, INPUT);

    pinMode(Deep_1, INPUT_PULLUP);
    pinMode(Deep_2, INPUT_PULLUP);
    pinMode(encoder_1, INPUT);
    pinMode(encoder_2, INPUT);
    pinMode(pot, INPUT);

    // Read speed from potentiometer
    int pot_value = analogRead(pot);
    speed = (u8)map(pot_value, 0, 4095, 100, 200);
    
   

    // Initialize maze
    Maze_Init(7, 7, 16);
  
    // Wait for initialization to complete
    delay(2000);
    // MySerial.println("start:....");
    // ENLeft_Counter = 0; ENRight_Counter = 0;
    // MySerial.print("Encoder: ");
    // 
    // while (!MySerial.available()); 
    // while (MySerial.available()) 
    // {
    // c = MySerial.read();  // اقرأ الحرف
    // 
    // }
    // MySerial.println("=== SYSTEM INITIALIZED ===");

}

/****** Main Loop ******/
u8 condition = 0;
void loop(void) 
{
  //  MySerial.print("Pos: (");
  //  MySerial.print(robot_current_row);
  //  MySerial.print(",");
  //  MySerial.print(robot_current_col);
  //  MySerial.print(") | Facing: ");
//
  //  switch (robot_current_orientation) {
  //      case ORIENTATION_NORTH: MySerial.print("NORTH"); break;
  //      case ORIENTATION_EAST: MySerial.print("EAST"); break;
  //      case ORIENTATION_SOUTH: MySerial.print("SOUTH"); break;
  //      case ORIENTATION_WEST: MySerial.print("WEST"); break;
  //  }

 //  while (!MySerial.available()); 
 //  while (MySerial.available()) 
 //  {
 //     c = MySerial.read();  // اقرأ الحرف
 //     
 //  }
condition = 0;
condition = (!digitalRead(Deep_1) << 0) | (!digitalRead(Deep_2) << 1);
Serial.println(condition);
switch (condition) 
{
  case 0:
    robot_current_row = 15;
    robot_current_col = 0;
    robot_current_orientation = ORIENTATION_NORTH;

   // Perform navigation
    Robot_NavigateToGoal1();
   // Stop after reaching goal
     while (1) 
     {
        Robot_StopMotors();
        condition = (!digitalRead(Deep_1) << 0) | (!digitalRead(Deep_2) << 1);
        if(condition != 0) break;
     }
     break;
     case 1:

    robot_current_row = 15;
    robot_current_col = 0;
    robot_current_orientation = ORIENTATION_NORTH;
    // Perform navigation
     Robot_NavigateToGoal1();
    // Stop after reaching goal
     while (1) 
     {
        Robot_StopMotors();
        condition = (!digitalRead(Deep_1) << 0) | (!digitalRead(Deep_2) << 1);
        if(condition != 1 ) break;
     }
     break;

     case 2:

     global_sensors = Robot_ReadSensors();  

     bool front_blocked = (global_sensors.front_sensor1 || global_sensors.front_sensor6);
     bool right_blocked = global_sensors.right_sensor;
     bool left_blocked  = global_sensors.left_sensor;


  if ( right_blocked && left_blocked && front_blocked )
  {
    Robot_StopMotors(); 
    delay(500);
    Robot_TurnAround(); 
  }
  else if ( right_blocked && left_blocked && !front_blocked )
  {
      Robot_StopMotors(); 
          delay(500);

    Robot_MoveForward();
  }
  else if ( right_blocked && !left_blocked && front_blocked )
  {
      Robot_StopMotors(); 
          delay(500);
Robot_TurnRight();
    //Robot_TurnLeft();
  }
  else if ( !right_blocked && left_blocked && front_blocked )
  {
      Robot_StopMotors();
          delay(500);
 Robot_TurnLeft();
    //Robot_TurnRight();
  }
  else if ( !right_blocked && !left_blocked && front_blocked )
  {
      Robot_StopMotors(); 
          delay(500);
Robot_TurnLeft();
    //Robot_TurnRight(); 
  }
  else if ( !right_blocked && !left_blocked && !front_blocked )
  {
    Robot_TurnLeft();
    //Robot_TurnRight(); 
        delay(500);
    Robot_MoveForward();
  }
  else if ( right_blocked && !left_blocked && !front_blocked )
  {
      Robot_StopMotors(); 
          delay(500);
    Robot_MoveForward(); 
  }
  else if ( !right_blocked && left_blocked && !front_blocked )
  {
    Robot_StopMotors(); 
        delay(500);
Robot_TurnLeft();
    //Robot_TurnRight(); 
    Robot_MoveForward();
  }
  else
  {
    Robot_StopMotors(); // Safety case
  }

  break;
}

  

}






