/*
 * Code by Ziyue Zhang
 * ANDREW ID: ziyuez
 * LAST UPDATE: Nov 22, 2021
 *
 * This monitor checks that the invariant "the turtle shall not go through 
 * walls" is not violated.
 * It checks whether bumped is called on the eage of movement before the 
 * movement takes place and whether there is a wall between the positional 
 * change.
 */

#include "monitor_interface.h"

// Keeps track of the last pose received
// moved is true if at least one pose has been received, false otherwise
static Pose last_pose;
static bool moved = false;

// last walls checked
Endpoints last_edge;
bool last_bumped = false;

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
 * Prints positional changes and checks whether invariant is violated
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
  // checks whether the movement edge is recorded in the wall dictionary
  // and whether a wall is not present
  if (moved && (last_pose.x != x || last_pose.y != y)) {
    Endpoints edge;
    if (x < last_pose.x) edge = wallBetween(last_pose, {last_pose.x, last_pose.y+1});
    else if (x > last_pose.x) edge = wallBetween({last_pose.x+1, last_pose.y}, {last_pose.x+1, last_pose.y+1});
    else if (y < last_pose.y) edge = wallBetween(last_pose, {last_pose.x+1, last_pose.y});
    else edge = wallBetween({last_pose.x, last_pose.y+1}, {last_pose.x+1, last_pose.y+1});
    if(edge.x1 != last_edge.x1 || edge.x2 != last_edge.x2 ||
       edge.y1 != last_edge.y1 || edge.y2 != last_edge.y2 ) {
      ROS_WARN("VIOLATION: Wall not checked between last postion (%d,%d) and current coordinate (%d,%d)", last_pose.x, last_pose.y, x, y);
    } else if (last_bumped == true) {
      ROS_WARN("VIOLATION: Turtle went through wall between last position (%d,%d) and current coordinate (%d,%d)", last_pose.x, last_pose.y, x, y);
    }
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
 * Updates the wall dictionary for the given two end points
 */
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
  Endpoints edge = wallBetween({x1, y1}, {x2, y2});
  last_edge = edge;
  last_bumped = bumped;
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
