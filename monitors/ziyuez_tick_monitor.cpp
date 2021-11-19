/*
 * Code by Ziyue Zhang
 * ANDREW ID: ziyuez
 * LAST UPDATE: Nov 18, 2021
 *
 * This monitor checks that the invariant "between calls to tickInterrupt, 
 * there shall be at most one call to each of poseInterrupt, visitsInterrupt, 
 * and bumpInterrupt." is not violated.
 * It checks whether the number of each of these 3 interrupt calls exceeds 
 * 1 between calls to tickInterrupt
 */

#include "monitor_interface.h"

// Keeps track of the number of interrupts called
static int32_t pose_count = 0;
static int32_t visit_count = 0;
static int32_t bump_count = 0;

/*
 * Increases interrupt counts when called
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
  pose_count += 1;
}

/*
 * Checks whether each interrupt is called more than once
 */
void tickInterrupt(ros::Time t) {
  ROS_INFO("[[%ld ns]] new tick, checking interrupt counts");
  // Check the corresponding count for each interrupt and print warning 
  // if greater than 1
  if (pose_count > 1) {
    ROS_WARN("poseInterrupt called more than once!");
  }
  if (visit_count > 1) {
    ROS_WARN("visitInterrupt called more than once!");
  }
  if (bump_count > 1) {
    ROS_WARN("bumpInterrupt called more than once!");
  }

  // Refresh interrupt count values
  pose_count = 0;
  visit_count = 0;
  bump_count = 0;
}

/*
 * Increases interrupt counts when called
 */
void visitInterrupt(ros::Time t, int visits) {
  visit_count += 1;
}

/*
 * Increases interrupt counts when called
 */
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
  bump_count += 1;
}

/*
 * Empty interrupt handlers beyond this point
 */
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}
