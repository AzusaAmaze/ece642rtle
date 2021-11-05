/*
 * 18-642 Unit Testing for turtle maze solver
 * Code by Ziyue Zhang
 */

#include <iostream>
#include <boost/bind.hpp>

typedef enum {
  LEFT, 
  RIGHT, 
  MOVE, 
  STOP
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
  state_err
} states;

typedef enum : int32_t {
  left=0, 
  up=1, 
  right=2, 
  down=3, 
  dir_err
} directions;  // turtle directions

typedef struct map_pos {
  int32_t row;
  int32_t col;
} map_pos_t;

typedef enum {
  BLOCK = 0,
  JUNC = 1, 
  PATH = 2
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
  RIGHT_P
} path_type;


int32_t turtleStateTransit(int32_t turn_count, bool first_time, bool bumped, bool goal);
turtleMove studentTurtleStep();
int32_t turnLeftRight(int32_t turtle_orient, bool is_left);

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

// Functions called locally and by test
void incPath(int32_t path_orient);
map_pos_t orientedCoord(int32_t turtle_orient);
int32_t atPath(int32_t turtle_orient);
int32_t visitPath(int32_t turtle_orient);
path_type pickPath(int32_t turtle_orient, bool first_time);
void juncUpdate(int32_t turtle_orient, bool bumped, int32_t turn_count);

// Functions implemented to replace ros library
void ROS_ERROR(std::string e);

void setErr(bool err_val);
bool getErr();
