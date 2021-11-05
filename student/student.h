/*
 * Code by Ziyue Zhang
 * ANDREW ID: ziyuez
 * LAST UPDATE: Nov 12, 2021
 */

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <ece642rtle/timeInt8.h>
#include <std_msgs/Empty.h>
#include <ece642rtle/RTIbump.h>
#include <ece642rtle/RTIatend.h>
#include <ece642rtle/PoseOrntBundle.h>
#include <ece642rtle/bumpEcho.h>
#include <ece642rtle/aendEcho.h>
#include <QPointF>

// Functions to interface with ROS. Don't change these lines!
bool bumped(int x1,int y1,int x2,int y2);
bool atend(int x, int y);
void displayVisits(int visits);
bool moveTurtle(QPointF& pos_, int& nw_or);

// Scope-preserving changes to these lines permitted (see p5 writeup)
typedef enum {
  LEFT, 
  RIGHT, 
  MOVE, 
  STOP
} turtleMove;

typedef enum : int32_t {
  left=0, 
  up=1, 
  right=2, 
  down=3
} directions;  // turtle directions

typedef enum {
  S_0, 
  S_1, 
  S_2, 
  S_3, 
  S_4, 
  S_5, 
  S_6, 
  S_7
} states;

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

QPointF translatePos(QPointF pos_, int32_t orientation, turtleMove next_move);
int32_t translateOrnt(int32_t orientation, turtleMove next_move);

turtleMove studentTurtleStep();
int32_t studentTurtleTransit(bool bumped, bool goal);

int32_t turnLeftRight(int32_t turtle_orient, bool is_left);


