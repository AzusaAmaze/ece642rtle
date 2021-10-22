/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Ziyue Zhang
 * ANDREW ID: ziyuez
 * LAST UPDATE: 
 *
 * This file keeps track of where the turtle is in the maze
 * and updates the location when the turtle is moved. It shall not
 * contain the maze solving logic/algorithm.
 *
 * This file is used along with student_turtle.cpp. student_turtle.cpp shall
 * contain the maze solving logic/algorithm and shall not make use of the
 * absolute coordinates or orientation of the turtle.
 *
 * This file shall call studentTurtleStep(..) in student_turtle.cpp to determine
 * the next move the turtle will make, and shall use translatePos(..) and
 * translateOrnt(..) to translate this move into absolute coordinates
 * to display the turtle.
 *
 */

#include "student.h"

typedef struct turtle_pospair{
  int32_t x;
  int32_t y;
} turtle_pospair_t;

const uint32_t timeout = 5;    // bigger number slows down simulation so you can see what's happening

/*
 * Helper function to test if turtle is bumped into wall.
 * Input: turtle_pos  turtle position
 *        turtle_orient turtle orientation
 * Output: whether turtle is facing a wall
 */
static bool isBumped(QPointF turtle_pos, int turtle_orient) {
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
 * Helper function to step forward the turtle by one unit
 * Input: turtle_pos  turtle position
 *        turtle_orient turtle orientation
 * Output: turtle position after movement
 */
static QPointF stepForward(QPointF turtle_pos, int32_t turtle_orient) {
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
  return turtle_pos;
}


/*
 * Helper function to turn turtle orientation to the right.
 * Input: turtle_orient turtle orientation
 * Output: turtle orientation after turn
 */
int32_t turnRight(int32_t turtle_orient) {
  int32_t result_orient;
  switch (turtle_orient) {
    case(left): 
      result_orient = up;
      break;
    case(up):
      result_orient = right;
      break;
    case(right):
      result_orient = down;
      break;
    case(down):
      result_orient = left;
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }
  return result_orient;
}


/*
 * Helper function to turn turtle orientation to the left.
 * Input: turtle_orient turtle orientation
 * Output: turtle orientation after turn
 */
int32_t turnLeft(int32_t turtle_orient) {
  int32_t result_orient;
  switch (turtle_orient) {
    case(left): 
      result_orient = down;
      break;
    case(up):
      result_orient = left;
      break;
    case(right):
      result_orient = up;
      break;
    case(down):
      result_orient = right;
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }
  return result_orient;
}


/*
 * Takes a position and a turtleMove and returns a new position
 * based on the move
 * Input: pos_        turtle position
 *        orientation turtle orientation
 *        next_move   next movement of turtle
 * Output: turtle postion after movement
 */
QPointF translatePos(QPointF pos_, int32_t orientation, turtleMove next_move) {
  switch (next_move) {
    case LEFT:
      // no positional change for turning
      break;
    case RIGHT:
      // no positional change for turning
      break;
    case MOVE:
      pos_ = stepForward(pos_, orientation);
      break;
    case STOP:
      // no positional change for STOP
      break;
    default:
      ROS_ERROR("Unrecognized turtle move");
      break;
  }
  return pos_;
}


/*
 * Takes an orientation and a turtleMove and returns a new orienation
 * based on the move
 * Input: orientation turtle orientation
 *        next_move   next movement of turtle
 * Output: turtle orientation after turn
 */
int32_t translateOrnt(int32_t orientation, turtleMove next_move) {
  int32_t result_orient;
  switch (next_move) {
    case LEFT:
      result_orient = turnLeft(orientation);
      break;
    case RIGHT:
      result_orient = turnRight(orientation);
      break;
    case MOVE:
      result_orient = orientation;
      break;
    case STOP:
      result_orient = orientation;
      break;
    default:
      ROS_ERROR("Unrecognized turtle move");
      break;
  }
  return result_orient;
}


/*
 * This procedure takes the current turtle position and orientation and returns true=accept changes, false=do not accept changes
 * Ground rule -- you are only allowed to call the three helper functions defined in student.h, and NO other turtle methods or maze methods (no peeking at the maze!)
 * This file interfaces with functions in student_turtle.cpp
 * Input: pos_  turtle position
 *        nw_or turtle orientation
 * Output: whether to accept turtle position and location changes
 */
bool moveTurtle(QPointF& pos_, int& nw_or)
{
  static uint32_t timer = 0;
  bool time_up = (timer == 0);

  if (time_up) {
    // turtle get the next movement
    turtleMove next_move = studentTurtleStep();
    pos_ = translatePos(pos_, nw_or, next_move);
    nw_or = translateOrnt(nw_or, next_move);

    // after performing movement, test conditions;
    bool bumped = isBumped(pos_, nw_or);
    bool goal = atend(pos_.x(), pos_.y());

    // update state & visit map by calling student_turtle methods
    int32_t visit_count = studentTurtleTransit(bumped, goal);
    displayVisits(visit_count);
    timer = timeout;
  } else {
    timer -= 1;
  }

  return time_up;
}