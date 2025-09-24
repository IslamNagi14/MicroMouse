/**************************************************************
************** File: FloodFill.c          *********************
************** Name: Islam Nagi           *********************
************** S/W:  flood-fill algorithm *********************
************** Date: 1 sep,2025           *********************
***************************************************************/

#include "QueMaze.h"
#include "mouse.h"
#include <stdio.h>
#include <string.h>
#include "StdTypes.h"
#include <Arduino.h>
#include <BluetoothSerial.h>

/****** Globals ******/
static u8 maze_size = MAZE_SIZE_MAX;
static u8 maze_walls[MAZE_SIZE_MAX][MAZE_SIZE_MAX];
static u8 maze_distances[MAZE_SIZE_MAX][MAZE_SIZE_MAX];
static s8 goal_row = 7;
static s8 goal_col = 7;

/* Row/col offsets for 4-neighbors (N, E, S, W) */
static const s8 row_offsets[4] = {-1, 0, 1, 0};
static const s8 col_offsets[4] = {0, 1, 0, -1};

/* Circular queue for BFS */
static Node_t queue[MAZE_SIZE_MAX * MAZE_SIZE_MAX];
static u16 queue_front = 0;
static u16 queue_rear = 0;
static u16 queue_size = 0;

// External declarations
//extern BluetoothSerial MySerial;
extern s8 robot_current_row;
extern s8 robot_current_col;
extern Orientation_t robot_current_orientation;
extern u8 speed;

// encoder counters
extern u16 ENLeft_Counter;
extern u16 ENRight_Counter;

/****** Queue functions ******/
static bool Queue_IsEmpty(void) 
{
    return (queue_size == 0);
}

static bool Queue_IsFull(void) 
{
    return (queue_size == (maze_size * maze_size));
}

static void Queue_Enqueue(s8 row, s8 col) 
{
    if (Queue_IsFull()) return;
    queue[queue_rear].row = row;
    queue[queue_rear].col = col;
    queue_rear = (queue_rear + 1) % (MAZE_SIZE_MAX * MAZE_SIZE_MAX);
    queue_size++;
}

static Node_t Queue_Dequeue(void) 
{
    Node_t node = {.row = -1, .col = -1};
    if (Queue_IsEmpty()) return node;
    node = queue[queue_front];
    queue_front = (queue_front + 1) % (MAZE_SIZE_MAX * MAZE_SIZE_MAX);
    queue_size--;
    return node;
}

static void Queue_Reset(void) 
{
    queue_front = queue_rear = queue_size = 0;
}

/****** Maze functions ******/
void Maze_Init(s8 goal_row_param, s8 goal_col_param, u8 size) 
{
    if (size > MAZE_SIZE_MAX) size = MAZE_SIZE_MAX;
    maze_size = size;
    goal_row = goal_row_param;
    goal_col = goal_col_param;

    for (u8 row = 0; row < maze_size; row++)
    {
        for (u8 col = 0; col < maze_size; col++) 
        {
            maze_distances[row][col] = CELL_UNREACHABLE;          
            maze_walls[row][col] = 0;                             
            
            if (row == 0) maze_walls[row][col] |= WALL_NORTH;
            if (row == maze_size - 1) maze_walls[row][col] |= WALL_SOUTH;
            if (col == 0) maze_walls[row][col] |= WALL_WEST;
            if (col == maze_size - 1) maze_walls[row][col] |= WALL_EAST;
        }
    }

    Maze_FloodFill();
}

/****** Wall Detection ******/
void Maze_DetectAndSetWalls(s8 robot_row, s8 robot_col, Orientation_t robot_orientation, RobotSensors_t sensors) 
{
    if (robot_row < 0 || robot_row >= maze_size || robot_col < 0 || robot_col >= maze_size) 
        return;
    
    /* Convert robot sensor readings to absolute walls */
    // Front wall detection (OR condition - either sensor detects wall)
    if (sensors.front_sensor1 && sensors.front_sensor6) //||-->&&
    {
        Direction_t abs_dir = (Direction_t)((ROBOT_FORWARD + robot_orientation) % 4);
        maze_walls[robot_row][robot_col] |= (1 << abs_dir);
        
        s8 neighbor_row = robot_row + row_offsets[abs_dir];
        s8 neighbor_col = robot_col + col_offsets[abs_dir];
        
        if (neighbor_row >= 0 && neighbor_row < maze_size && neighbor_col >= 0 && neighbor_col < maze_size) {
            u8 reciprocal_dir = (Direction_t)((abs_dir + 2) % 4);
            maze_walls[neighbor_row][neighbor_col] |= (1 << reciprocal_dir);
        }
    }
    
    // Right wall detection
    if (sensors.right_sensor) 
    {
        Direction_t abs_dir = (Direction_t)((ROBOT_RIGHT + robot_orientation) % 4);
        maze_walls[robot_row][robot_col] |= (1 << abs_dir);
        
        s8 neighbor_row = robot_row + row_offsets[abs_dir];
        s8 neighbor_col = robot_col + col_offsets[abs_dir];
        
        if (neighbor_row >= 0 && neighbor_row < maze_size && neighbor_col >= 0 && neighbor_col < maze_size) {
            u8 reciprocal_dir = (Direction_t)((abs_dir + 2) % 4);
            maze_walls[neighbor_row][neighbor_col] |= (1 << reciprocal_dir);
        }
    }
    
    // Left wall detection
    if (sensors.left_sensor ) 
    {
        Direction_t abs_dir = (Direction_t)((ROBOT_LEFT + robot_orientation) % 4);
        maze_walls[robot_row][robot_col] |= (1 << abs_dir);
        
        s8 neighbor_row = robot_row + row_offsets[abs_dir];
        s8 neighbor_col = robot_col + col_offsets[abs_dir];
        
        if (neighbor_row >= 0 && neighbor_row < maze_size && neighbor_col >= 0 && neighbor_col < maze_size) {
            u8 reciprocal_dir = (Direction_t)((abs_dir + 2) % 4);
            maze_walls[neighbor_row][neighbor_col] |= (1 << reciprocal_dir);
        }
    }
    
    Maze_FloodFill();
}

void Maze_FloodFill(void) 
{
    Queue_Reset();
    
    for (u8 row = 0; row < maze_size; row++) 
    {
        for (u8 col = 0; col < maze_size; col++) 
        {
            if (maze_distances[row][col] != CELL_WALL)
            {
                maze_distances[row][col] = CELL_UNREACHABLE;
            }
        }
    }
    
    maze_distances[goal_row][goal_col] = 0;
    Queue_Enqueue(goal_row, goal_col);
    
    if (goal_row + 1 < maze_size) {
        maze_distances[goal_row + 1][goal_col] = 0;
        Queue_Enqueue(goal_row + 1, goal_col);
    }
    if (goal_col + 1 < maze_size) {
        maze_distances[goal_row][goal_col + 1] = 0;
        Queue_Enqueue(goal_row, goal_col + 1);
    }
    if (goal_row + 1 < maze_size && goal_col + 1 < maze_size) {
        maze_distances[goal_row + 1][goal_col + 1] = 0;
        Queue_Enqueue(goal_row + 1, goal_col + 1);
    }
    
    while (!Queue_IsEmpty()) 
    {
        Node_t current = Queue_Dequeue();
        if (current.row < 0) continue;
        
        u8 current_dist = maze_distances[current.row][current.col];
        
        for (u8 dir = 0; dir < 4; dir++) 
        {
            if (maze_walls[current.row][current.col] & (1 << dir)) 
                continue;
            
            s8 next_row = current.row + row_offsets[dir];
            s8 next_col = current.col + col_offsets[dir];
            
            if (next_row < 0 || next_row >= maze_size || next_col < 0 || next_col >= maze_size) 
                continue;
            
            if (maze_distances[next_row][next_col] == CELL_WALL) 
                continue;
            
            if (maze_distances[next_row][next_col] > current_dist + 1) 
            {
                maze_distances[next_row][next_col] = current_dist + 1;
                Queue_Enqueue(next_row, next_col);
            }
        }
    }
}

u8 Maze_GetCellDistance(s8 row, s8 col) 
{
    if (row < 0 || row >= maze_size || col < 0 || col >= maze_size)
        return CELL_UNREACHABLE;
    return maze_distances[row][col];
}

u8 Maze_GetCellWalls(s8 row, s8 col) 
{
    if (row < 0 || row >= maze_size || col < 0 || col >= maze_size) 
        return 0;
    return maze_walls[row][col];
}

/****** Navigation Helper Functions ******/
bool IsAtGoal(s8 row, s8 col) {
    return (row >= goal_row && row <= goal_row + 1 && 
            col >= goal_col && col <= goal_col + 1);
}

RobotDirection_t ChooseBestDirection(s8 row, s8 col, Orientation_t orientation) {
    u8 min_distance = CELL_UNREACHABLE;
    RobotDirection_t best_direction = ROBOT_FORWARD;
    
    // Check all four directions using integer loop
    for (u8 i = 0; i < 4; i++) {
        RobotDirection_t rel_dir = (RobotDirection_t)i;
        Direction_t abs_dir = (Direction_t)((rel_dir + orientation) % 4);
        s8 next_row = row + row_offsets[abs_dir];
        s8 next_col = col + col_offsets[abs_dir];
        
        if (next_row < 0 || next_row >= maze_size || next_col < 0 || next_col >= maze_size) 
            continue;
        if (maze_walls[row][col] & (1 << abs_dir)) 
            continue;
        
        u8 dist = Maze_GetCellDistance(next_row, next_col);
        if (dist < min_distance) {
            min_distance = dist;
            best_direction = rel_dir;
        }
    }
    
    return best_direction;
}

void ExecuteMovement(RobotDirection_t best_dir) {
    switch(best_dir) {
        case ROBOT_FORWARD:
            Robot_MoveForward();
            Robot_StopMotors(); 
            break;
        case ROBOT_RIGHT:
            Robot_TurnLeft();
           // Robot_TurnRight();
           // Robot_MoveForward();
            break;
        case ROBOT_LEFT:
        Robot_TurnRight();
            //Robot_TurnLeft();
            //Robot_MoveForward();
            break;
        case ROBOT_BACKWARD:
            Robot_TurnAround();
            //Robot_MoveForward();
            break;
    }
}
//char c;
/****** Main Navigation Function ******/
void Robot_NavigateToGoal1(void) 
{
    while(!IsAtGoal(robot_current_row, robot_current_col)) 
    {
 
        RobotSensors_t sensors = Robot_ReadSensors();
//        while (!MySerial.available()); 
//  while (MySerial.available()) 
//  {
//     c = MySerial.read();  // اقرأ الحرف
    
//  }
        Maze_DetectAndSetWalls(robot_current_row, robot_current_col, robot_current_orientation, sensors);
       //MySerial.print("walls: ");MySerial.println(maze_walls[robot_current_row][robot_current_col]);
//         while (!MySerial.available()); 
//  while (MySerial.available()) 
//  {
//     c = MySerial.read();  // اقرأ الحرف
    
//  }
       
 RobotDirection_t best_dir = ChooseBestDirection(robot_current_row, robot_current_col, robot_current_orientation);
// MySerial.print("best_dir: ");MySerial.println(best_dir);
//  while (!MySerial.available()); 
//  while (MySerial.available()) 
//  {
//     c = MySerial.read();  // اقرأ الحرف
    
//  }
 ExecuteMovement(best_dir);
        
        delay(100);
    }
    
    // MySerial.print("🎉 Goal reached at (");
    // MySerial.print(robot_current_row);
    // MySerial.print(",");
    // MySerial.print(robot_current_col);
    // MySerial.println(")!");
    // digitalWrite(buzzer, HIGH);
}

// void Maze_Print(void) 
// {
//     MySerial.print("Maze (");
//     MySerial.print(maze_size);
//     MySerial.print("x");
//     MySerial.print(maze_size);
//     MySerial.print(") - Distances to goal at (");
//     MySerial.print(goal_row);
//     MySerial.print(",");
//     MySerial.print(goal_col);
//     MySerial.println("):");
    
//     for (s8 row = 0; row < maze_size; row++) 
//     {
//         for (s8 col = 0; col < maze_size; col++) 
//         {
//             if (maze_distances[row][col] == CELL_UNREACHABLE) 
//             {
//                 MySerial.print("  X ");
//             } 
//             else 
//             {
//                 if (maze_distances[row][col] < 10) MySerial.print(" ");
//                 MySerial.print(maze_distances[row][col]);
//                 MySerial.print(" ");
//             }
//         }
//         MySerial.println();
//     }
// }




void Robot_NavigateToGoal2(void) 
{

    while(!IsAtGoal(robot_current_row, robot_current_col)) 
    {
 
        RobotDirection_t best_dir = ChooseBestDirection(robot_current_row, robot_current_col, robot_current_orientation);
        ExecuteMovement(best_dir);
        
        delay(100);
    }
    
}


RobotSensors_t Robot_ReadSensors(void) 
{
    RobotSensors_t sensors;
    sensors.front_sensor1 = digitalRead(IR_1);
    sensors.front_sensor6 = digitalRead(IR_6);
    sensors.left_sensor = digitalRead(IR_3);
    sensors.right_sensor = digitalRead(IR_4);
    sensors.back_sensor = 0;

    // MySerial.print("Sensors - F1:");
    // MySerial.print(sensors.front_sensor1);
    // MySerial.print(" F6:");
    // MySerial.print(sensors.front_sensor6);
    // MySerial.print(" L:");
    // MySerial.print(sensors.left_sensor);
    // MySerial.print(" R:");
    // MySerial.println(sensors.right_sensor);

    return sensors;
}