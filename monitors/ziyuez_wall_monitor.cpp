/*
 * Code by Ziyue Zhang
 * ANDREW ID: ziyuez
 * LAST UPDATE: Nov 20, 2021
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

// Flag that doesn't print pose updates if the turtle has moved 0 steps
static const bool suppress_double_visits = true;

// Local dictionary for potential walls checked
static std::map<Endpoints, bool> wall_map; 

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
  // Print pose info
  // Last conditional makes sure that if suppress_double_visits is
  // true, that the same pose isn't printed twice
  if (!suppress_double_visits || !moved ||
      (last_pose.x != x || last_pose.y != y)) {
    ROS_INFO("[[%ld ns]] 'Pose' was sent. Data: x = %d, y=%d", t.toNSec(), x, y);
  }

  // checks whether the movement edge is recorded in the wall dictionary
  // and whether a wall is not present
  if (moved && (last_pose.x != x || last_pose.y != y)) {
    Endpoints edge = wallBetween(last_pose, {x, y});
    std::map<Endpoints, bool>::iterator it = wall_map.find(edge);
    if(it == wall_map.end()) {
      ROS_WARN("VIOLATION: Wall not checked between last postion (%d,%d) and current coordinate (%d,%d)", last_pose.x, last_pose.y, x, y);
    } else if (wall_map[edge] == true) {
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
  wall_map[edge] = bumped;
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
