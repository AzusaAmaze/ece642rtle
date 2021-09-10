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
float w, cs;
float fx1, fy1, fx2, fy2;
float bp, q;
		 
// this procedure takes the current turtle position and orientation and returns
// true=submit changes, false=do not submit changes
// Ground rule -- you are only allowed to call the helper functions "bumped(..)" and "atend(..)",
// and NO other turtle methods or maze methods (no peeking at the maze!)
bool studentMoveTurtle(QPointF& pos_, int& nw_or)
{ 
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
     * 0. If bumped into wall, turn right. Else change to state 2.
     * 1. If bumped into wall, turn right and change to state 0. Else change 
     *    to state 2.
     * 2. Turn left and change to state 1.
     * State 0 is the entry point of this "state machine".
     */
		if(nw_or == 0) {
  		if(cs == 2)  { nw_or = 3;  cs = 1; }
  		else if (bp) { nw_or = 1;  cs = 0; }
		  else cs = 2;
    }
		else if(nw_or == 1) {
  		if(cs == 2)  { nw_or = 0;  cs = 1; }
  		else if (bp) { nw_or = 2;  cs = 0; }
  		else cs = 2;
    }
		else if(nw_or == 2) {
  		if(cs == 2)  { nw_or = 1;  cs = 1; }
  		else if (bp) { nw_or = 3;  cs = 0; }
  		else cs = 2;
    }
		else if(nw_or == 3) {
  		if(cs == 2)  { nw_or = 2;  cs = 1; }
  		else if (bp) { nw_or = 0;  cs = 0; }
  		else cs = 2;
    }

	  ROS_INFO("Orientation=%f  STATE=%f", nw_or, cs);
	  if(cs == 2) {
      if (nw_or == 1) pos_.setY(pos_.y() - 1); 
      if (nw_or == 2) pos_.setX(pos_.x() + 1);
      if (nw_or == 3) pos_.setY(pos_.y() + 1);
      if (nw_or == 0) pos_.setX(pos_.x() - 1);
    }}
    if (w==0) w  = TIMEOUT; else w -= 1;
    if (w==TIMEOUT) return true;
  return false;
}
