/**************************************************************
************** File: QueMaze.h            *********************
************** Name: Islam Nagi           *********************
************** S/W:  flood-fill algorithm *********************
************** Date: 1 sep,2025           *********************
***************************************************************/

#ifndef __QUE_MAZE_H__
#define __QUE_MAZE_H__

#include "StdTypes.h"
#include <stdbool.h>

/* Maze constants */
#define MAZE_SIZE_MAX 20
#define CELL_UNREACHABLE 0xFF    /* 255 = unreachable */
#define CELL_WALL 0xFE           /* 254 = wall cell */

/* Wall bits: one bit per direction (N,E,S,W) */
#define WALL_NORTH (1 << 0) 
#define WALL_EAST  (1 << 1) 
#define WALL_SOUTH (1 << 2)
#define WALL_WEST  (1 << 3) 

/* Direction indices */
typedef enum 
{
    DIRECTION_NORTH = 0,
    DIRECTION_EAST = 1,
    DIRECTION_SOUTH = 2,
    DIRECTION_WEST = 3
} Direction_t;

/* Robot orientation */
typedef enum
{
    ORIENTATION_NORTH = 0,
    ORIENTATION_EAST = 1,
    ORIENTATION_SOUTH = 2,
    ORIENTATION_WEST = 3
} Orientation_t;

/* Relative directions from robot's perspective */
typedef enum 
{
    ROBOT_FORWARD = 0,
    ROBOT_RIGHT = 1,
    ROBOT_BACKWARD = 2,
    ROBOT_LEFT = 3
} RobotDirection_t;

/* Robot sensor data structure */
typedef struct 
{
    bool front_sensor1;  // Left front sensor (IR1)
    bool front_sensor6;  // Right front sensor (IR6)
    bool left_sensor;    // Left side sensor (IR3)
    bool right_sensor;   // Right side sensor (IR4)
    bool back_sensor;    // Not used
} RobotSensors_t;

/* Node structure for queue */
typedef struct {
    s8 row;
    s8 col;
} Node_t;

/* Public APIs */
void Maze_Init(s8 Copy_s8GoalRow, s8 Copy_s8GoalCol, u8 Copy_u8Size);
void Maze_DetectAndSetWalls(s8 robot_row, s8 robot_col, Orientation_t robot_orientation, RobotSensors_t sensors);
void Maze_FloodFill(void);
void Maze_Print(void);
u8 Maze_GetCellDistance(s8 Copy_s8Row, s8 Copy_s8Col);
u8 Maze_GetCellWalls(s8 Copy_s8Row, s8 Copy_s8Col);

/* Navigation functions */
RobotDirection_t ChooseBestDirection(s8 row, s8 col, Orientation_t orientation);
void ExecuteMovement(RobotDirection_t best_dir);
void Robot_NavigateToGoal(void);
bool IsAtGoal(s8 row, s8 col);
RobotSensors_t Robot_ReadSensors(void);
/* External variables */
extern s8 robot_current_row;
extern s8 robot_current_col;
extern Orientation_t robot_current_orientation;
extern u8 speed;

#endif // __QUE_MAZE_H__