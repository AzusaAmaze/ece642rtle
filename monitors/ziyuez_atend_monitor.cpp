/*
 * Code by Ziyue Zhang
 * ANDREW ID: ziyuez
 * LAST UPDATE: Nov 21, 2021
 *
 * This monitor checks that the invariant "Turtle shall only call 
 * atEnd(x,y) if it is at a position x,y." is not violated.
 * It checks whether upon atEnd calls the input poses are the same
 * as turtle's current pose
 */

#include "monitor_interface.h"

// Keeps track of the last pose received
// moved is true if at least one pose has been received, false otherwise
static Pose last_pose;
static bool moved = false;

/*
 * Updates previous pose
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
  // store last Pose in memory
  last_pose.x = x;
  last_pose.y = y;

  // Update this flag the first time the turtle moves
  if (!moved) {
    moved = true;
  }
}

/*
 * Compares the input pose to stored pose and throws an
 * invariant violation if the poses are different
 */
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
  // turtle pose should be the same
  if (moved && (last_pose.x != x || last_pose.y != y)) {
    ROS_WARN("VIOLATION: Turtle at position (%d,%d) calls atend at (%d,%d)!", last_pose.x, last_pose.y, x, y);
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
