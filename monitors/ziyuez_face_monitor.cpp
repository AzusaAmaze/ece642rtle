/*
 * Code by Ziyue Zhang
 * ANDREW ID: ziyuez
 * LAST UPDATE: Nov 21, 2021
 *
 * This monitor checks that the invariant "for any call to 
 * bumped(x1,y1,x2,y2), the turtle shall be facing the wall segment 
 * with endpoints (x1,y1) and (x2,y2)" is not violated.
 * It checks whether bumped is called on the eage where the turtle is 
 * currently facing.
 */

#include "monitor_interface.h"

// Keeps track of the last pose & orientation received
// moved is true if at least one pose has been received, false otherwise
static Pose last_pose;
static Orientation last_orient;
static bool moved = false;

/*
 * Calculates the end point between two positions following 
 * "(y1==y2 && x1 < x2) || (x1==x2 && y1 < y2)"
 */
Endpoints wallBetween(Pose p1, Pose p2) {
  Pose temp = p1;
  if (p1.y == p2.y && p1.x > p2.x) {
    p1 = p2;
    p2 = temp;
  }

  if (p1.x == p2.x && p1.y > p2.y) {
    p1 = p2;
    p2 = temp;
  }

  return {p1.x, p1.y, p2.x, p2.y};
}

/*
 * Whenever called, updates last turtle position and orientation
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
  // store last Pose in memory
  last_pose.x = x;
  last_pose.y = y;
  last_orient = o;

  // Update this flag the first time the turtle moves
  if (!moved) {
    moved = true;
  }
}

/*
 * Calculates faced segment and compare with the input endpoints
 */
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
  Endpoints called_e = wallBetween({x1, y1}, {x2, y2});
  Endpoints faced_e = called_e;

  switch(last_orient) {
    case NORTH:
      faced_e = wallBetween(last_pose, {last_pose.x+1, last_pose.y});
      break;
    case WEST:
      faced_e = wallBetween(last_pose, {last_pose.x, last_pose.y+1});
      break;
    case SOUTH:
      faced_e = wallBetween({last_pose.x, last_pose.y+1}, {last_pose.x+1, last_pose.y+1});
      break;
    case EAST:
      faced_e = wallBetween({last_pose.x+1, last_pose.y}, {last_pose.x+1, last_pose.y+1});
      break;
    default:
      ROS_INFO("[[%ld ns]] last_orient unrecognized");
      break;
  }

  // if calculated segment and input segment non equal, violation
  if (moved && ((called_e.x1 != faced_e.x1)||(called_e.x2 != faced_e.x2)||
      (called_e.y1 != faced_e.y1)||(called_e.y2 != faced_e.y2))) {
      ROS_WARN("VIOLATION: Turtle not facing wall segment (%d,%d) and (%d,%d), actually facing segment (%d,%d) and (%d,%d)", 
                x1, y1, x2, y2, 
                faced_e.x1, faced_e.y1, faced_e.x2, faced_e.y2);
  }
}

/*
 * Empty interrupt handlers beyond this point
 */

void tickInterrupt(ros::Time t) {
}

void visitInterrupt(ros::Time t, int visits) {
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}
