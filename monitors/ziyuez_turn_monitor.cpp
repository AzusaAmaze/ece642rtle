/*
 * Code by Ziyue Zhang
 * ANDREW ID: ziyuez
 * LAST UPDATE: Nov 12, 2021
 *
 * This monitor checks that the invariant "turtle turns no more than 90 
 * degrees per simulator time step" is not violated.
 * It keeps track of the previous orientation of the turtle and compares it
 * to the current orientation to check the invariant.
 */

#include "monitor_interface.h"

// Keeps track of the last orientation received
// turned is true if at least one pose has been received, false otherwise
static Orientation last_orient;
static bool turned = false;

// Flag that doesn't print pose updates if the turtle has turned 0 degrees
static const bool suppress_double_visits = true;

/*
 * Returns whether turtle performed less than a quarter 
 * turn
 * WARNING: unsafe for undefined orientation, should
 * only be called if turtle has turned at least once
 */
static bool isQuarter(Orientation o) {
  bool is_quarter = false;

  switch(last_orient) {
    case NORTH:
      is_quarter = (o != SOUTH);
      break;
    case WEST:
      is_quarter = (o != EAST);
      break;
    case SOUTH:
      is_quarter = (o != NORTH);
      break;
    case EAST:
      is_quarter = (o != WEST);
      break;
    default:
      ROS_INFO("[[%ld ns]] last_orient unrecognized");
      break;
  }

  return is_quarter;
}

/*
 * Whenever the turtle moves, compare the current orientation
 * to the previous orientation and throw an invariant violation
 * if the orientations differ by more than 90 degrees.
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
  // Print pose info
  // Last conditional makes sure that if suppress_double_visits is
  // true, that the same pose isn't printed twice
  std::string o_str;
  switch(o) {
  case NORTH:
    o_str = "NORTH";
    break;
  case WEST:
    o_str = "WEST";
    break;
  case SOUTH:
    o_str = "SOUTH";
    break;
  case EAST:
    o_str = "EAST";
    break;
  default:
    o_str = "ERROR";
    break;
  }
  if (!suppress_double_visits || !turned || last_orient != o) {
    ROS_INFO("[[%ld ns]] 'Pose' was sent. Data: x=%d, y=%d, o=%s", t.toNSec(), x, y, o_str.c_str());
  }

  // Check that the turtle has turned before and that the degrees turned
  // does not exceed 90
  if (turned && !isQuarter(o)) {
    ROS_WARN("VIOLATION: Turtle turned more than 90 degrees at x=%d, y=%d, o=%s", x, y, o_str.c_str());
  }

  // store last orientation in memory
  last_orient = o;

  // Update this flag the first time the turtle turns
  if (!turned) {
    turned = true;
  }
}

/*
 * Empty interrupt handlers beyond this point
 */

void tickInterrupt(ros::Time t) {
}

void visitInterrupt(ros::Time t, int visits) {
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}
