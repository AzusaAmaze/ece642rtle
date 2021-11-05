/*
 * 18-642 Unit Testing for turtle maze solver
 * Code by Ziyue Zhang
 */

#include <iostream>

typedef enum {
  LEFT, 
  RIGHT, 
  MOVE, 
  STOP, 
  ERR_DEFAULT
} turtleMove;

typedef enum {
  S_0, 
  S_1, 
  S_2, 
  S_3, 
  S_4, 
  S_5, 
  S_6, 
  S_7, 
  ERR_DEFAULT
} states;

typedef enum : int32_t {
  left=0, 
  up=1, 
  right=2, 
  down=3, 
  ERR_DEFAULT
} directions;  // turtle directions

typedef struct map_pos {
  int32_t row;
  int32_t col;
} map_pos_t;

typedef enum {
  BLOCK = 0,
  JUNC = 1, 
  PATH = 2, 
  ERR_DEFAULT
} block_type;

typedef struct block_info {
  block_type up_block;
  block_type down_block;
  block_type left_block;
  block_type right_block;
  block_type curr_block;
  int32_t up_count;
  int32_t down_count;
  int32_t left_count;
  int32_t right_count;
} block_info_t;

typedef enum {
  FRONT_P, 
  BACK_P, 
  LEFT_P, 
  RIGHT_P, 
  ERR_DEFAULT
} path_type;


int32_t turtleStateTransit(int32_t turn_count, bool first_time, bool bumped, bool goal);
turtleMove studentTurtleStep();

// Functions called by main testing program to get or set values
void setState(states new_state);
states getState();

void setOrient(int32_t new_orient);
int32_t getOrient();

void visitSet(map_pos_t map_coord, int32_t new_visit);
int32_t visitGet(map_pos_t map_coord);

void setCoord(map_pos_t new_coord);
map_pos_t getCoord();

void juncSet(map_pos_t map_coord, block_info_t new_junc);
block_info_t juncGet(map_pos_t map_coord);

void ROS_ERROR(std::string e);
