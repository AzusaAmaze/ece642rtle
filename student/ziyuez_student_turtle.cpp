/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Ziyue ZHANG
 * ANDREW ID: ziyuez
 * LAST UPDATE: Sep 10, 2021
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

const uint32_t timeout = 40;    // bigger number slows down simulation so you can see what's happening 
enum directions : int32_t {left=0, up=1, right=2, down=3};  // turtle directions
enum states {state_0, state_1};                             // turtle states
static int32_t visit_map[23][23] = {0};                     // visit counts for map
static map_pos_t turtle_pos = {11, 11};                       // turtle position on visit map

/*
 * Helper function to turn turtle orientation to the right.
 * Input: nw_or turtle orientation
 */
void turnRight(int& nw_or) {
  switch (nw_or) {
    case(left): 
      nw_or = up;
      break;
    case(up):
      nw_or = right;
      break;
    case(right):
      nw_or = down;
      break;
    case(down):
      nw_or = left;
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }
}


/*
 * Helper function to turn turtle orientation to the left.
 * Input: nw_or turtle orientation
 */
void turnLeft(int& nw_or) {
  switch (nw_or) {
    case(left): 
      nw_or = down;
      break;
    case(up):
      nw_or = left;
      break;
    case(right):
      nw_or = up;
      break;
    case(down):
      nw_or = right;
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }
}

/*
 * The only function that reads/writes turtle_pos and visit_map
 */
void visitUpdate(int& nw_or) {
  // update the turtle pos
  switch (nw_or) {
    case(left): 
      turtle_pos.col -= 1;
      break;
    case(up):
      turtle_pos.row -= 1;
      break;
    case(right):
      turtle_pos.col += 1;
      break;
    case(down):
      turtle_pos.row += 1;
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }
  // update the visit count on the new pos
  visit_map[turtle_pos.row][turtle_pos.col] +=1;
}

/*
 * Reads turtle_pos and return the visit count on visit_map
 */
int32_t visitGet() {
  return visit_map[turtle_pos.row][turtle_pos.col];
}


/*
 * Helper function to step forward the turtle by one unit and update turtle visit counts.
 * Input: pos_  turtle position
 *        nw_or turtle orientation
 */
void stepForward(QPointF& pos_, int& nw_or) {
  switch (nw_or) {
    case(left): 
      pos_.setX(pos_.x() - 1);
      break;
    case(up):
      pos_.setY(pos_.y() - 1);
      break;
    case(right):
      pos_.setX(pos_.x() + 1);
      break;
    case(down):
      pos_.setY(pos_.y() + 1);
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }
  visitUpdate(nw_or);
}


/*
 * Helper function to test if turtle is bumped into wall.
 * Input: pos_  turtle position
 *        nw_or turtle orientation
 * Output: whether turtle is facing a wall
 */
bool isBumped(QPointF& pos_, int& nw_or) {
  float fx1, fy1, fx2, fy2;
  fx1 = pos_.x(); 
  fy1 = pos_.y();
  fx2 = pos_.x(); 
  fy2 = pos_.y();

  switch (nw_or) {
    case(left): 
      fy2 += 1;
      break;
    case(up):
      fx2 += 1;
      break;
    case(right):
      fx2 += 1;
      fy2 += 1;
      fx1 += 1;
      break;
    case(down):
      fx2 += 1;
      fy2 += 1;
      fy1 += 1;
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }

  return bumped(fx1,fy1,fx2,fy2);
}


/* 
 * This procedure takes the current turtle position and orientation and returns
 * true=submit changes, false=do not submit changes
 * Input:  pos_         turtle position
 *         nw_or        turtle orientation
 * Saved:  timer        Count down timer for next turtle movement
 *         turtle_state State for turtle state machine
 * Output: Whether to submit turtle position changes
 */
bool studentMoveTurtle(QPointF& pos_, int32_t& nw_or) { 
  static uint32_t timer = 0;
  static states turtle_state = state_0;
  bool time_up = (timer == 0);

  ROS_INFO("Turtle update Called  timer=%f", timer);

  // if already reached destination
  if (atend(pos_.x(), pos_.y())) return false;

  // when timer is up, perform movement 
  if(timer == 0) {
    /*
     * State of turtle is updated once with every call to studentTurtleStep, 
     * with the states in the "state machine" being: 
     * 0. If bumped into wall, turn right. Else step and change to state 1.
     * 1. Turn left and change to state 0.
     * State 0 is the entry point of this "state machine".
     */
    switch (turtle_state) {
      case state_0: 
        if (isBumped(pos_, nw_or)) turnLeft(nw_or);
        else {
          stepForward(pos_, nw_or);
          turtle_state = state_1;
        }
        break;
      case state_1: 
        turnRight(nw_or);
        turtle_state = state_0;
        break;
      default: 
        ROS_ERROR("Unrecignized tuetle state");
        break;
    }

	  ROS_INFO("Orientation=%d  STATE=%d", nw_or, turtle_state);

    // update timer
    timer = timeout;
  } else {
    timer -= 1;
  }

  displayVisits(visitGet());
  return time_up;
}
