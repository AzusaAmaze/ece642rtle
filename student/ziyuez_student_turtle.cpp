/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Ziyue ZHANG
 * ANDREW ID: ziyuez
 * LAST UPDATE: Sep 24, 2021
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the right-hand rule.
 *
 */

#include "student.h"

// Ignore this line until project 5
turtleMove studentTurtleStep(bool bumped) {return MOVE;}

// OK TO MODIFY BELOW THIS LINE

typedef struct map_pos {
  int32_t row;
  int32_t col;
} map_pos_t;

typedef enum : int32_t {
  left=0, 
  up=1, 
  right=2, 
  down=3
} directions;  // turtle directions

typedef enum {
  state_0, 
  state_1
}states;

typedef struct turtle_pospair{
  int32_t x;
  int32_t y;
} turtle_pospair_t;

const uint32_t timeout = 40;    // bigger number slows down simulation so you can see what's happening                              // turtle states
static int32_t visit_map[23][23] = {0};                     // visit counts for map
static map_pos_t turtle_coord = {11, 11};                       // turtle position on visit map

/*
 * Helper function to turn turtle orientation to the right.
 * Input: turtle_orient turtle orientation
 */
void turnRight(int& turtle_orient) {
  switch (turtle_orient) {
    case(left): 
      turtle_orient = up;
      break;
    case(up):
      turtle_orient = right;
      break;
    case(right):
      turtle_orient = down;
      break;
    case(down):
      turtle_orient = left;
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }
}


/*
 * Helper function to turn turtle orientation to the left.
 * Input: turtle_orient turtle orientation
 */
void turnLeft(int& turtle_orient) {
  switch (turtle_orient) {
    case(left): 
      turtle_orient = down;
      break;
    case(up):
      turtle_orient = left;
      break;
    case(right):
      turtle_orient = up;
      break;
    case(down):
      turtle_orient = right;
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }
}


/*
 * The only function that reads/writes turtle_coord and visit_map
 * Input: turtle_orient turtle orientation
 */
void visitUpdate(int& turtle_orient) {
  // update the turtle pos
  switch (turtle_orient) {
    case(left): 
      turtle_coord.col -= 1;
      break;
    case(up):
      turtle_coord.row -= 1;
      break;
    case(right):
      turtle_coord.col += 1;
      break;
    case(down):
      turtle_coord.row += 1;
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }
  // update the visit count on the new pos
  visit_map[turtle_coord.row][turtle_coord.col] +=1;
}


/*
 * Reads turtle_coord and return the visit count on visit_map
 */
int32_t visitGet() {
  return visit_map[turtle_coord.row][turtle_coord.col];
}


/*
 * Helper function to step forward the turtle by one unit and update turtle visit counts.
 * Input: turtle_pos  turtle position
 *        turtle_orient turtle orientation
 */
void stepForward(QPointF& turtle_pos, int& turtle_orient) {
  switch (turtle_orient) {
    case(left): 
      turtle_pos.setX(turtle_pos.x() - 1);
      break;
    case(up):
      turtle_pos.setY(turtle_pos.y() - 1);
      break;
    case(right):
      turtle_pos.setX(turtle_pos.x() + 1);
      break;
    case(down):
      turtle_pos.setY(turtle_pos.y() + 1);
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }
  visitUpdate(turtle_orient);
}


/*
 * Helper function to test if turtle is bumped into wall.
 * Input: turtle_pos  turtle position
 *        turtle_orient turtle orientation
 * Output: whether turtle is facing a wall
 */
bool isBumped(QPointF& turtle_pos, int& turtle_orient) {
  turtle_pospair_t pos1, pos2;
  pos1.x = turtle_pos.x(); 
  pos1.y = turtle_pos.y();
  pos2.x = turtle_pos.x(); 
  pos2.y = turtle_pos.y();

  switch (turtle_orient) {
    case(left): 
      pos2.y += 1;
      break;
    case(up):
      pos2.x += 1;
      break;
    case(right):
      pos2.x += 1;
      pos2.y += 1;
      pos1.x += 1;
      break;
    case(down):
      pos2.x += 1;
      pos2.y += 1;
      pos1.y += 1;
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }

  return bumped(pos1.x,pos1.y,pos2.x,pos2.y);
}


/* 
 * This procedure takes the current turtle position and orientation and returns
 * true=submit changes, false=do not submit changes
 * Input:  turtle_pos         turtle position
 *         turtle_orient        turtle orientation
 * Saved:  timer        Count down timer for next turtle movement
 *         turtle_state State for turtle state machine
 * Output: Whether to submit turtle position changes
 */
bool studentMoveTurtle(QPointF& turtle_pos, int32_t& turtle_orient) { 
  static uint32_t timer = 0;
  static states turtle_state = state_1;
  bool time_up = (timer == 0);

  ROS_INFO("Turtle update Called  timer=%f", timer);

  // if already reached destination
  if (atend(turtle_pos.x(), turtle_pos.y())) return false;

  // when timer is up, perform movement 
  if(timer == 0) {
    /*
     * State of turtle is updated once with every call to studentTurtleStep, 
     * with the states in the "state machine" being: 
     * 0. If bumped into wall, turn right. Else step and change to state 1.
     * 1. Turn left and change to state 0.
     * State 1 is the entry point of this "state machine".
     */
    switch (turtle_state) {
      case state_0: 
        if (isBumped(turtle_pos, turtle_orient)) turnLeft(turtle_orient);
        else {
          stepForward(turtle_pos, turtle_orient);
          turtle_state = state_1;
        }
        break;
      case state_1: 
        turnRight(turtle_orient);
        turtle_state = state_0;
        break;
      default: 
        ROS_ERROR("Unrecognized turtle state");
        break;
    }

	  ROS_INFO("Orientation=%d  STATE=%d", turtle_orient, turtle_state);

    // update timer
    timer = timeout;
  } else {
    timer -= 1;
  }

  displayVisits(visitGet());
  return time_up;
}
