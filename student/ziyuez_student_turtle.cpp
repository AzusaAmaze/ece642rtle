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

#ifdef testing
#include "ziyuez_student_mock.h"
#endif
#ifndef testing
#include "student.h"
#include "ros/ros.h"
#endif

// OK TO MODIFY BELOW THIS LINE

static states turtle_state = S_6;                   // turtle state
static int32_t map_orient = up;                     // visit map orientation
static int32_t visit_map[23][23] = {0};             // visit counts for map
static block_info_t junction_map[23][23] = {{BLOCK, BLOCK, BLOCK, BLOCK, BLOCK, 0, 0, 0, 0}};   // map marking junctions and walkable paths
static map_pos_t turtle_coord = {11, 11};           // turtle position on visit map


// TODO: Some functions that are only supposed to be called during test

void setState(states new_state) {
  turtle_state = new_state;
}


states getState() {
  return turtle_state;
}

void setOrient(int32_t new_orient) {
  map_orient = new_orient;
}


int32_t getOrient() {
  return turtle_orient;
}


void visitSet(map_pos_t map_coord, int32_t new_visit) {
  visit_map[map_coord.row][map_coord.col] = new_visit;
}


void setCoord(map_pos_t new_coord) {
  turtle_coord = new_coord;
}


map_pos_t getCoord() {
  return turtle_coord;
}


void juncSet(map_pos_t map_coord, block_info_t new_junc) {
  junction_map[map_coord.row][map_coord.col] = new_junc;
}


block_info_t juncGet(map_pos_t map_coord) {
  return junction_map[map_coord.row][map_coord.col];
}

// Functions called both locally and by tests

/*
 * Reads map_coord and return the visit count on visit_map
 * Input:  map_coord  coordinate where we want the visit count at
 * Output: visit      count at the current turtle coordinates
 */
int32_t visitGet(map_pos_t map_coord) {
  return visit_map[map_coord.row][map_coord.col];
}


// Functions called locally

/*
 * Increases visit map count at current turtle map position
 */
static void visitInc() {
  // update the visit count on the new pos
  visit_map[turtle_coord.row][turtle_coord.col] +=1;
}


/*
 * Return whether current turtle block is visited the first time
 * Output: bool       whether current block is visited the first time
 */
static bool firstVisit() {
  return visitGet(turtle_coord) == 1;
}


/*
 * Compute the coordinates of the block turtle is orienting towards
 * Input:  turtle_orient  orientation of turtle
 * Output: faced_coord    coordinates turtle facing towards
 */
static map_pos_t orientedCoord(int32_t turtle_orient) {
  map_pos_t faced_coord;
  faced_coord.row = turtle_coord.row; 
  faced_coord.col = turtle_coord.col;

  // update the oriented block coord
  switch (turtle_orient) {
    case(left): 
      faced_coord.col -= 1;
      break;
    case(up):
      faced_coord.row -= 1;
      break;
    case(right):
      faced_coord.col += 1;
      break;
    case(down):
      faced_coord.row += 1;
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }
  return faced_coord;
}


/*
 * Determine if turtle is facing towards a walkable path
 * Input:  turtle_orient  orientation of turtle
 * Output: bool           whether oriented block is a path
 */
static bool atPath(int32_t turtle_orient) {
  block_info_t curr_info = junction_map[turtle_coord.row][turtle_coord.col];
  block_type oriented_block = curr_info.curr_block;
  switch (turtle_orient) {
    case(left): 
      oriented_block = curr_info.left_block;
      break;
    case(up):
      oriented_block = curr_info.up_block;
      break;
    case(right):
      oriented_block = curr_info.right_block;
      break;
    case(down):
      oriented_block = curr_info.down_block;
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }

  return oriented_block == PATH;
}


/*
 * Count number of times the faced path is visited
 * Input:  turtle_orient  orientation of turtle
 * Output: path_count     number of times path is visited
 */
static int32_t visitPath(int32_t turtle_orient) {
  int32_t path_count = -1;
  block_info_t curr_info = junction_map[turtle_coord.row][turtle_coord.col];
  switch (turtle_orient) {
    case(left): 
      path_count = curr_info.left_count;
      break;
    case(up):
      path_count = curr_info.up_count;
      break;
    case(right):
      path_count = curr_info.right_count;
      break;
    case(down):
      path_count = curr_info.down_count;
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }
  return path_count;
}


/*
 * Determine the path to take at a junction. This function is only 
 * called when turtle is in S_7, which is when turtle is at junction.
 * This function updates the path visit count of the entered path and 
 * future path.
 * Input:  turtle_orient  orientation of turtle
 *         first_time     whether junction is visited the first time
 * Output: next_path      type of the path to be taken
 */
static path_type pickPath(int32_t turtle_orient, bool first_time) {
  int32_t front_orient = turtle_orient; 
  int32_t back_orient = turtle_orient; 
  int32_t left_orient = turtle_orient;
  int32_t right_orient = turtle_orient; 
  int32_t path_orient = turtle_orient;
  path_type next_path = FRONT_P;

  /* pick path based on junction map and number of visits at junction */
  switch (turtle_orient) {
    case(left): 
      front_orient = left;
      back_orient = right;
      left_orient = down;
      right_orient = up;
      junction_map[turtle_coord.row][turtle_coord.col].right_count += 1;
      break;
    case(up):
      front_orient = up;
      back_orient = down;
      left_orient = left;
      right_orient = right;
      junction_map[turtle_coord.row][turtle_coord.col].down_count += 1;
      break;
    case(right):
      front_orient = right;
      back_orient = left;
      left_orient = up;
      right_orient = down;
      junction_map[turtle_coord.row][turtle_coord.col].left_count += 1;
      break;
    case(down):
      front_orient = down;
      back_orient = up;
      left_orient = right;
      right_orient = left;
      junction_map[turtle_coord.row][turtle_coord.col].up_count += 1;
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }

  /* 
   * If the junction is visited for the first time, it will pick one walkable 
   * path that is not the path it came from (in my implementation, favoring 
   * front over left over right). If the junction is not visited for the first 
   * time, we then look at how many times the entered path has been visited. 
   * If the entered path has only one visit, we then return from the entered path. 
   * Else if the entered path has more than 1 visit, we then pick from the other 
   * walkable paths a path that is visited the least number of times (again, 
   * favoring front over left over right).
   */
  if (first_time) {
    if (atPath(front_orient)) {
      next_path = FRONT_P;
      path_orient = front_orient;
    }
    else if (atPath(left_orient)) {
      next_path = LEFT_P;
      path_orient = left_orient;
    }
    else {
      next_path = RIGHT_P;
      path_orient = right_orient;
    }
  } else {
    if (visitPath(back_orient) == 1) {
      next_path = BACK_P;
      path_orient = back_orient;
    } else {
      /* pick a path with least number of visits (0 or 1) */
      if (atPath(front_orient) && visitPath(front_orient) == 0) {
        next_path = FRONT_P;
        path_orient = front_orient;
      } else if (atPath(left_orient) && visitPath(left_orient) == 0) {
        next_path = LEFT_P;
        path_orient = left_orient;
      } else if (atPath(right_orient) && visitPath(right_orient) == 0) {
        next_path = RIGHT_P;
        path_orient = right_orient;
      } else if (atPath(front_orient) && visitPath(front_orient) < 2) {
        next_path = FRONT_P;
        path_orient = front_orient;
      } else if (atPath(left_orient) && visitPath(left_orient) < 2) {
        next_path = LEFT_P;
        path_orient = left_orient;
      } else if (atPath(right_orient) && visitPath(right_orient) < 2) {
        next_path = RIGHT_P;
        path_orient = right_orient;
      } else {
        ROS_ERROR("counts %d %d %d %d", visitPath(front_orient), visitPath(back_orient), visitPath(left_orient), visitPath(right_orient));
        ROS_ERROR("Unable to find walkable path at junction!");
      }
    }
  }

  /* update path visits for the exiting path */
  switch (path_orient) {
    case (left):
      junction_map[turtle_coord.row][turtle_coord.col].left_count += 1;
      break;
    case (right):
      junction_map[turtle_coord.row][turtle_coord.col].right_count += 1;
      break;
    case (up):
      junction_map[turtle_coord.row][turtle_coord.col].up_count += 1;
      break;
    case (down):
      junction_map[turtle_coord.row][turtle_coord.col].down_count += 1;
      break;
    default: 
      ROS_ERROR("Unrecognized path orientation");
      break;
  }

  return next_path;
}


/*
 * Update junction map by marking walkable paths and junction at current block.
 * A walkable path to the current block is an adjacent block which is not 
 * obstructed by a wall. A junction is defined as a block where there are more 
 * than 2 walkable paths or the walkable paths connected don't form a straight line.
 * Input: turtle_orient turtle orientation
 *        bumped        if turtle is facing a wall
 *        turn_count    number of times turtle turned at current block
 */
static void juncUpdate(int32_t turtle_orient, bool bumped, int32_t turn_count) {
  block_info_t curr_info = junction_map[turtle_coord.row][turtle_coord.col];

  if (turn_count == 1) curr_info.curr_block = BLOCK;
  /*
   * Update walkable path by checking whether faced block is obstructed.
   * If turtle turned 90 or 270 degrees and is not obstructed by wall, 
   * current block is junction.
   */
  switch (turtle_orient) {
    case(left): 
      if (!bumped) {
        curr_info.left_block = PATH;
      } else {
        curr_info.left_block = BLOCK;
      }
      break;
    case(up):
      if (!bumped) {
        curr_info.up_block = PATH;
      } else {
        curr_info.up_block = BLOCK;
      }
      break;
    case(right):
      if (!bumped) {
        curr_info.right_block = PATH;
      } else {
        curr_info.right_block = BLOCK;
      }
      break;
    case(down):
      if (!bumped) {
        curr_info.down_block = PATH;
      } else {
        curr_info.down_block = BLOCK;
      }
      break;
    default:
      ROS_ERROR("Unrecognized turtle orientation");
      break;
  }

  if (!bumped) {
    if (turn_count%2 != 0) curr_info.curr_block = JUNC;
    if (turn_count == 0 && curr_info.curr_block != JUNC) curr_info.curr_block = PATH;
  }

  junction_map[turtle_coord.row][turtle_coord.col] = curr_info;
}


// Functions called by maze/test

/*
 * Helper function to turn turtle orientation to the right.
 * Input: turtle_orient turtle orientation
 * Output: turtle orientation after turn
 */
int32_t turnRight(int32_t turtle_orient) {
  int32_t result_orient = turtle_orient;
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
  int32_t result_orient = turtle_orient;
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
 * This procedure decides the next move of turtle based on the current
 * state and updates the local visit map information. Always called 
 * before studentTurtleTransit() in each time cycle.
 * Output: next turtle movement to be performed by maze methods
 */
turtleMove studentTurtleStep() {
  turtleMove next_move = STOP;

  /*
  * Decides next movement of turtle before transitioning state
  * with the states in the "state machine" being: 
  * 0. STOP. No movement
  * 1. MOVE. step 1 block forward 
  * 2. LEFT. turn left
  * 3. LEFT. step 1 block forward
  * 4. LEFT. turn left
  * 5. RIGHT. turn right
  * 6. STOP. No movement
  * 7. STOP. No movement
  * State 1 is the entry point of this "state machine".
  */
  switch (turtle_state)
  {
    case S_0:
      next_move = STOP;
      break;
    case S_1:
      next_move = MOVE;
      turtle_coord = orientedCoord(map_orient);
      visitInc();
      break;
    case S_2:
      next_move = LEFT;
      map_orient = turnLeft(map_orient);
      break;
    case S_3:
      next_move = LEFT;
      map_orient = turnLeft(map_orient);
      break;
    case S_4:
      next_move = LEFT;
      map_orient = turnLeft(map_orient);
      break;
    case S_5:
      next_move = RIGHT;
      map_orient = turnRight(map_orient);
      break;
    case S_6:
      next_move = STOP;
      visitInc();
      break;
    case S_7:
      next_move = STOP;
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
 * Output: visit_count  number of visits after transit
 */
int32_t studentTurtleTransit(bool bumped, bool goal) {
  static int32_t turn_count = 0;
  bool first_time = firstVisit();

  turn_count = turtleStateTransit(turn_count, first_time, bumped, goal);

  int32_t visit_count = visitGet(turtle_coord);
  return visit_count;
}


// TODO: add comment
int32_t turtleStateTransit(int32_t turn_count, bool first_time, bool bumped, bool goal) {
  /*
  * State of turtle is updated once with every call to studentTurtleStep, 
  * with the states in the "state machine" being: 
  * 0. At Goal. always stay in current state.
  * 1. Step. will test junction if stepped in block for the first time. Else 
  *          move according to block type. 
  * 2. Test Junction. will turn left until 4 directions tested and block types 
  *                   recorded. 
  * 3. Turn Around. will turn left until turned around.
  * 4. Left Path. Will turn to the left path and move forward next.
  * 5. Right Path. Will turn to the right path and move forward next.
  * 6. Init stage. Decide movement based on whether at goal.
  * 7. At Junc. Will update junction map  and choose next path.
  * State 6 is the entry point of this "state machine".
  */
  switch (turtle_state)
  {
    /* State S_0: at goal */
    case S_0:
      /* state transition */
      turtle_state = S_0;   // T0
      break;

    /* State S_1: step */
    case S_1:
    {
      /* side effects */
      turn_count = 0;
      /* state transition */
      block_type curr_type = junction_map[turtle_coord.row][turtle_coord.col].curr_block;
      if (first_time && goal) turtle_state = S_0;                         // T1
      else if (first_time && !goal) turtle_state = S_2;                   // T2
      else if (!first_time && curr_type == BLOCK) turtle_state = S_3;     // T3
      else if (!first_time && curr_type == PATH) turtle_state = S_1;      // T4
      else if (!first_time && curr_type == JUNC) turtle_state = S_7;      // T5
      else ROS_ERROR("Unlisted combination at S1");
      break;
    }

    /* State S_2: test junction */
    case S_2:
    {
      /* side effects */
      turn_count = (turn_count+1)%4;
      juncUpdate(map_orient, bumped, turn_count);
      /* state transition */
      block_type curr_type = junction_map[turtle_coord.row][turtle_coord.col].curr_block;
      if (turn_count != 0) turtle_state = S_2;                              // T6
      else if (turn_count == 0 && curr_type == BLOCK) turtle_state = S_3;   // T7
      else if (turn_count == 0 && curr_type == PATH) turtle_state = S_1;    // T8
      else if (turn_count == 0 && curr_type == JUNC) turtle_state = S_7;    // T9
      else ROS_ERROR("Unlisted combination at S2");
      break;
    }

    /* State S_3: turn around */
    case S_3:
      /* side effects */
      turn_count = turn_count+1;
      /* state transition */
      if (turn_count != 2) turtle_state = S_3;        // T10
      else if (turn_count == 2) turtle_state = S_1;   // T11
      else ROS_ERROR("Unlisted combination at S3");
      break;

    /* State S_4: left path */
    case S_4:
      /* state transition */
      turtle_state = S_1;       // T12
      break;

    /* State S_5: right path */
    case S_5:
      /* state transition */
      turtle_state = S_1;       // T13
      break;

    /* State S_6: initial stage */
    case S_6:
      /* state transition */
      if (goal) turtle_state = S_0;         // T14
      else if (!goal) turtle_state = S_2;   // T15
      else ROS_ERROR("Unlisted combination at S6");  
      break;

    /* State S_7: at junc */
    case S_7:
    {
      /* side effects */
      path_type next_path;
      next_path = pickPath(map_orient, first_time);
      /* state transitions */
      if (next_path == FRONT_P) turtle_state = S_1;         // T16
      else if (next_path == LEFT_P) turtle_state = S_4;     // T17
      else if (next_path == RIGHT_P) turtle_state = S_5;    // T18
      else if (next_path == BACK_P) turtle_state = S_3;     // T19
      else ROS_ERROR("Unlisted combination at S7");
      break;
    }

    default:
      ROS_ERROR("Unrecognized turtle state");
      break;
  }
  return turn_count;
}
