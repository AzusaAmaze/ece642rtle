/*
 * Code by Ziyue Zhang
 * ANDREW ID: ziyuez
 * LAST UPDATE: Nov 18, 2021
 *
 * This monitor checks that the invariant "the turtle shall face the 
 * direction it is moving" is not violated.
 * It keeps track of the previous orientation and the last position 
 * of the turtle and compares it to the current orientation and position 
 * to check the invariant.
 */

#include "monitor_interface.h"

// Keeps track of the last pose received
// Keeps track of the last orientation received
// moved is true if at least one pose has been received, false otherwise
static Pose last_pose;
static Orientation last_orient;
static bool moved = false;

/*
 * Whenever the turtle moves or turns, compare the current orientation
 * to the previous orientation and orientation to previous orientation 
 * and throw an invariant violation if the the turtle does not face the 
 * direction it is moving
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
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

  // Check that the turtle has turned before and that the turtle does not 
  // face the direction it is moving
  if (moved) {
    if ((last_pose.x != x || last_pose.y != y) && o != last_orient) {
      ROS_WARN("VIOLATION: Turtle turned and moved in same tick at x=%d, y=%d, o=%s, t=%ld ns", 
                x, y, o_str.c_str(), t.toNSec());
    }

    if (((x > last_pose.x) && (last_orient != EAST)) || ((x < last_pose.x) && (last_orient != WEST)) || 
        ((y > last_pose.y) && (last_orient != SOUTH)) || ((y < last_pose.y) && (last_orient != NORTH))) {
      ROS_WARN("VIOLATION: Turtle move and orientation not alligned at x=%d, y=%d, o=%s, t=%ld ns", 
          x, y, o_str.c_str(), t.toNSec());
    }
  }

  // store last orientation and position in memory
  last_orient = o;
  last_pose.x = x;
  last_pose.y = y;

  // Update this flag the first time the turtle turns
  if (!moved) {
    moved = true;
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
