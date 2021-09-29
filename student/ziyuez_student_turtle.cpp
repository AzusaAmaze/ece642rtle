/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME: Ziyue ZHANG
 * ANDREW ID: ziyuez
 * LAST UPDATE: Sep 24, 2021
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the right-hand rule.
 *
 */

#include "student.h"

// OK TO MODIFY BELOW THIS LINE

typedef struct map_pos {
  int32_t row;
  int32_t col;
} map_pos_t;

typedef enum {
  S_0, 
  S_1, 
  S_2, 
  S_3
} states;

static states turtle_state = S_1;              // turtle state
static int32_t map_orient = up;                // visit map orientation
static int32_t visit_map[23][23] = {0};        // visit counts for map
static map_pos_t turtle_coord = {11, 11};      // turtle position on visit map


/* 
 * This procedure decides the next move of turtle based on the current
 * state and updates the local visit map information. Always called 
 * before studentTurtleTransit() in each time cycle.
 * Output: next turtle movement to be performed by maze methods
 */
turtleMove studentTurtleStep() {
  turtleMove next_move;

  /*
  * Decides next movement of turtle before transitioning state
  * with the states in the "state machine" being: 
  * 0. STOP. No movement
  * 1. RIGHT. turn right 
  * 2. LEFT. turn left
  * 3. MOVE. step 1 block forward
  * State 1 is the entry point of this "state machine".
  */
  switch (states)
  {
    case S_0:
      next_move = STOP;
      break;
    case S_1:
      next_move = RIGHT;
      map_orient = turnRight(map_orient);
      break;
    case S_2:
      next_move = LEFT;
      map_orient = turnLeft(map_orient);
      break;
    case S_3:
      next_move = MOVE;
      visitUpdate(map_orient)
      break;
    default:
      ROS_ERROR("Unrecognized turtle state");
      break;
  }
  return next_move;
}


/* 
 * This procedure decides the next state of turtle based on the current
 * state and conditions. Always called after studentTurtleTransit() 
 * in each time cycle.
 * Input:  bumped       if turtle is facing a wall
 *         goal         if turtle is at goal
 */
void studentTurtleTransit(bool bumped, bool goal) {
  /*
  * State of turtle is updated once with every call to studentTurtleStep, 
  * with the states in the "state machine" being: 
  * 0. STOP. always stay in current state.
  * 1. RIGHT. will move forward next if not bumped. else turn left 
  * 2. LEFT. will move forward next if not bumped. else turn left 
  * 3. MOVE. will stop if at goal. else turn right
  * State 1 is the entry point of this "state machine".
  */
  switch (states)
  {
    case S_0:
      turtle_state = S_0;
      break;
    case S_1:
      if (bumped) turtle_state = S_2;
      else turtle_state = S_3;
      break;
    case S_2:
      if (bumped) turtle_state = S_2;
      else turtle_state = S_3;
      break;
    case S_3:
      if (goal) turtle_state = S_0;
      else turtle_state = S_1;
      break;
    default:
      ROS_ERROR("Unrecognized turtle state");
      break;
  }
  return next_move;
}


/*
 * The only function that reads/writes turtle_coord and visit_map
 * Input: turtle_orient turtle orientation
 */
static void visitUpdate(int turtle_orient) {
  // update the turtle pos
  switch (turtle_orient) {
    case(left): 
      turtle_coord.col -= 1;
      break;
    case(up):
      turtle_coord.row -= 1;
      break;
    case(right):
      turtle_coord.col += 1;
      break;
    case(down):
      turtle_coord.row += 1;
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }
  // update the visit count on the new pos
  visit_map[turtle_coord.row][turtle_coord.col] +=1;
}


/*
 * Reads turtle_coord and return the visit count on visit_map
 * Output: visit count at the current turtle coordinates
 */
int32_t visitGet() {
  return visit_map[turtle_coord.row][turtle_coord.col];
}