/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME:
 * ANDREW ID:
 * LAST UPDATE:
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
float w;
float fx1, fy1, fx2, fy2;
float bp, q;
int state;

enum directions{left=0, up=1, right=2, down=3};

/*
 * Helper function to turn turtle orientation to the right.
 * Input: nw_or turtle orientation
 */
void turnRight(int& nw_or) {
  if (nw_or == left) nw_or = up;
  else if (nw_or == up) nw_or = right;
  else if (nw_or == right) nw_or = down;
  else if (nw_or == down) nw_or = left;
  return;
}

/*
 * Helper function to turn turtle orientation to the left.
 * Input: nw_or turtle orientation
 */
void turnLeft(int& nw_or) {
  if (nw_or == left) nw_or = down;
  else if (nw_or == up) nw_or = left;
  else if (nw_or == right) nw_or = up;
  else if (nw_or == down) nw_or = right;
  return;
}
		 
// this procedure takes the current turtle position and orientation and returns
// true=submit changes, false=do not submit changes
// Ground rule -- you are only allowed to call the helper functions "bumped(..)" and "atend(..)",
// and NO other turtle methods or maze methods (no peeking at the maze!)
bool studentMoveTurtle(QPointF& pos_, int& nw_or) { 
  ROS_INFO("Turtle update Called  w=%f", w);

  // if already reached destination
  if (atend(pos_.x(), pos_.y())) return false;

  // when wait time is up, perform movement 
  if(w == 0) {

    // check whether turtle is facing a wall
    fx1 = pos_.x(); 
    fy1 = pos_.y();
    fx2 = pos_.x(); 
    fy2 = pos_.y();

	  if (nw_or < 2) {
  		if (nw_or == 0) fy2+=1;
  		else            fx2+=1;
    }
		else{ 
      fx2+=1; 
      fy2+=1; 
		  if (nw_or == 2) fx1+=1;  
		  else            fy1+=1; 
		}

		bp = bumped(fx1,fy1,fx2,fy2);

    /*
     * State of turtle is updated once with every call to studentTurtleStep, 
     * with the states in the "state machine" being: 
     * 0. If bumped into wall, turn right. Else change to state 1.
     * 1. Turn left and change to state 0.
     * State 0 is the entry point of this "state machine".
     */
    ROS_INFO("old Orientation=%f  old STATE=%d", nw_or, state);

    switch (state) {
      case 0: {
        if (bp) turnRight(nw_or);
        else state = 1;
        break;
      }
      case 1: { 
        turnLeft(nw_or);
        state = 0;
        break;
      }
      default: break;
    }

	  ROS_INFO("Orientation=%d  STATE=%d", nw_or, state);
	  if(state == 1) {
      if (nw_or == 1) pos_.setY(pos_.y() - 1); 
      if (nw_or == 2) pos_.setX(pos_.x() + 1);
      if (nw_or == 3) pos_.setY(pos_.y() + 1);
      if (nw_or == 0) pos_.setX(pos_.x() - 1);
    }
  }
  if (w==0) w  = TIMEOUT; else w -= 1;
  if (w==TIMEOUT) return true;
  return false;
}
