/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Ziyue ZHANG
 * ANDREW ID: ziyuez
 * LAST UPDATE: Sep 10, 2021
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 *
 */

#include "student.h"

// Ignore this line until project 5
turtleMove studentTurtleStep(bool bumped) {return MOVE;}

// OK TO MODIFY BELOW THIS LINE

#define TIMEOUT 40    // bigger number slows down simulation so you can see what's happening

enum directions : int {left=0, up=1, right=2, down=3};   // turtle directions
enum states {state_0, state_1};                          // turtle states


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
    break;
  }
}


/*
 * Helper function to step forward the turtle by one unit.
 * Input: pos_  turtle position
 *        nw_or turtle orientation
 */
void stepForward(QPointF& pos_, int& nw_or) {
  if (nw_or == up) pos_.setY(pos_.y() - 1); 
  if (nw_or == right) pos_.setX(pos_.x() + 1);
  if (nw_or == down) pos_.setY(pos_.y() + 1);
  if (nw_or == left) pos_.setX(pos_.x() - 1);
  return;
}


/*
 * Helper function to test if turtle is bumped into wall.
 * Input: pos_  turtle position
 *        nw_or turtle orientation
 */
bool isBumped(QPointF& pos_, int& nw_or) {
  float fx1, fy1, fx2, fy2;
  fx1 = pos_.x(); 
  fy1 = pos_.y();
  fx2 = pos_.x(); 
  fy2 = pos_.y();

  if (nw_or == left) fy2 += 1;
  else if (nw_or == up) fx2 += 1;
  else if (nw_or == right) {
    fx2 += 1;
    fy2 += 1;
    fx1 += 1;
  } else if (nw_or == down) {
    fx2 += 1;
    fy2 += 1;
    fy1 += 1;
  }
  return bumped(fx1,fy1,fx2,fy2);
}


/* 
 * This procedure takes the current turtle position and orientation and returns
 * true=submit changes, false=do not submit changes
 * Input: pos_  turtle position
 *        nw_or turtle orientation
 */
bool studentMoveTurtle(QPointF& pos_, int& nw_or) { 
  static int timer = 0;
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
        break;
    }

	  ROS_INFO("Orientation=%d  STATE=%d", nw_or, turtle_state);

    // update timer
    timer = TIMEOUT;
  } else {
    timer -= 1;
  }
  return time_up;
}
