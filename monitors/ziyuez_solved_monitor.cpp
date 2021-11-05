/*
 * Code by Ziyue Zhang
 * ANDREW ID: ziyuez
 * LAST UPDATE: Nov 21, 2021
 *
 * This monitor checks that the invariant "If the turtle has solved the 
 * maze (atEnd(x,y)==true), it shall not move or turn" is not violated.
 * It checks whether atEnd returns true at some point and compares 
 * current pose to previous pose.
 */

#include "monitor_interface.h"

// Keeps track of the last pose received
// moved is true if at least one pose has been received, false otherwise
// solved is true is turtle reaches at_end for once
static Pose last_pose;
static bool moved = false;
static bool solved = false;

/*
 * After turtle maze is solved, compare current pose and previous  
 * pose and throws a violation if the poses are different
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {

  // if maze is solved, turtle should not move
  if (moved && solved && (last_pose.x != x || last_pose.y != y)) {
    ROS_WARN("VIOLATION: Turtle moved after maze is solved!");
  }

  // store last Pose in memory
  last_pose.x = x;
  last_pose.y = y;

  // Update this flag the first time the turtle moves
  if (!moved) {
    moved = true;
  }
}

/*
 * Updates solved if atEnd is true
 */
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
  if (atEnd) solved = atEnd;
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
