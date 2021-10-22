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
  BLOCK = 0,
  JUNC = 1, 
  PATH = 2
} block_type;

typedef struct block_info {
  block_type up_block;
  block_type down_block;
  block_type left_block;
  block_type right_block;
  block_type curr_block;
  int32_t up_count;
  int32_t down_count;
  int32_t left_count;
  int32_t right_count;
} block_info_t;

typedef enum {
  FRONT_P, 
  BACK_P, 
  LEFT_P, 
  RIGHT_P
} path_type;

typedef enum {
  S_0, 
  S_1, 
  S_2, 
  S_3, 
  S_4, 
  S_5, 
  S_6, 
  S_7
} states;

static states turtle_state = S_6;                   // turtle state
static int32_t map_orient = up;                     // visit map orientation
static int32_t visit_map[23][23] = {0};             // visit counts for map
static block_info_t junction_map[23][23] = {{BLOCK, BLOCK, BLOCK, BLOCK, BLOCK, 0, 0, 0, 0}};   // map marking junctions and walkable paths
static map_pos_t turtle_coord = {11, 11};           // turtle position on visit map


/*
 * Increases visit map count at current turtle map position
 */
static void visitInc() {
  // update the visit count on the new pos
  visit_map[turtle_coord.row][turtle_coord.col] +=1;
}

/*
 * Reads map_coord and return the visit count on visit_map
 * Input:  map_coord  coordinate where we want the visit count at
 * Output: visit      count at the current turtle coordinates
 */
static int32_t visitGet(map_pos_t map_coord) {
  return visit_map[map_coord.row][map_coord.col];
}


// TODO: add comment
static bool firstVisit() {
  return visitGet(turtle_coord) == 1;
}


// TODO: add comment
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
 * The only function that reads/writes turtle_coord and visit_map
 * Input: turtle_orient turtle orientation
 */
static void visitUpdate(int32_t turtle_orient) {
  // update the turtle pos
  turtle_coord = orientedCoord(turtle_orient);
  
  // update the visit count on the new pos
  visitInc();
}


// TODO: add comment
static bool atPath(int32_t turtle_orient) {
  block_type oriented_block;
  block_info_t curr_info = junction_map[turtle_coord.row][turtle_coord.col];
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


static int32_t visitPath(int32_t turtle_orient) {
  int32_t path_count;
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


// TODO: add comments
static path_type pickPath(int32_t turtle_orient, bool first_time) {
  int32_t front_orient, back_orient, left_orient, right_orient, path_orient;
  path_type next_path;

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
        ROS_INFO("counts %d %d %d %d", visitPath(front_orient), visitPath(back_orient), visitPath(left_orient), visitPath(right_orient));
        ROS_ERROR("Unable to find walkable path at junction!");
      }
    }
  }

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
  map_pos_t path_coord = orientedCoord(turtle_orient);
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


/* 
 * This procedure decides the next move of turtle based on the current
 * state and updates the local visit map information. Always called 
 * before studentTurtleTransit() in each time cycle.
 * Output: next turtle movement to be performed by maze methods
 */
turtleMove studentTurtleStep() {
  turtleMove next_move;

  // TODO: update comments
  /*
  * Decides next movement of turtle before transitioning state
  * with the states in the "state machine" being: 
  * 0. STOP. No movement
  * 1. RIGHT. turn right 
  * 2. LEFT. turn left
  * 3. MOVE. step 1 block forward
  * State 1 is the entry point of this "state machine".
  */
  switch (turtle_state)
  {
    case S_0:
      next_move = STOP;
      break;
    case S_1:
      next_move = MOVE;
      visitUpdate(map_orient);
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
  // TODO: update comments
  static int32_t turn_count = 0;
  bool first_time = firstVisit();

  /*
  * State of turtle is updated once with every call to studentTurtleStep, 
  * with the states in the "state machine" being: 
  * 0. STOP. always stay in current state.
  * 1. RIGHT. will move forward next if not bumped. else turn left 
  * 2. LEFT. will move forward next if not bumped. else turn left 
  * 3. MOVE. will stop if at goal. else turn right
  * State 1 is the entry point of this "state machine".
  */
  switch (turtle_state)
  {
    case S_0:
      /* state transition */
      turtle_state = S_0;
      break;

    case S_1:
    {
      /* side effects */
      turn_count = 0;
      /* state transition */
      block_type curr_type = junction_map[turtle_coord.row][turtle_coord.col].curr_block;
      if (first_time && goal) turtle_state = S_0;
      else if (first_time && !goal) turtle_state = S_2;
      else if (!first_time && curr_type == BLOCK) turtle_state = S_3;
      else if (!first_time && curr_type == PATH) turtle_state = S_1;
      else if (!first_time && curr_type == JUNC) turtle_state = S_7;
      else ROS_ERROR("Unlisted combination at S1");
      break;
    }

    case S_2:
    {
      /* side effects */
      turn_count = (turn_count+1)%4;
      juncUpdate(map_orient, bumped, turn_count);
      /* state transition */
      block_type curr_type = junction_map[turtle_coord.row][turtle_coord.col].curr_block;
      if (turn_count != 0) turtle_state = S_2;
      else if (turn_count == 0 && curr_type == BLOCK) turtle_state = S_3;
      else if (turn_count == 0 && curr_type == PATH) turtle_state = S_1;
      else if (turn_count == 0 && curr_type == JUNC) turtle_state = S_7;
      else ROS_ERROR("Unlisted combination at S2");
      break;
    }

    case S_3:
      /* side effects */
      turn_count = turn_count+1;
      /* state transition */
      if (turn_count != 2) turtle_state = S_3;
      else if (turn_count == 2) turtle_state = S_1;
      else ROS_ERROR("Unlisted combination at S3");
      break;

    case S_4:
      /* state transition */
      turtle_state = S_1;
      break;

    case S_5:
      /* state transition */
      turtle_state = S_1;
      break;

    case S_6:
      /* state transition */
      if (goal) turtle_state = S_0;
      else if (!goal) turtle_state = S_2;
      else ROS_ERROR("Unlisted combination at S6");  
      break;

    case S_7:
    {
      /* side effects */
      path_type next_path = pickPath(map_orient, first_time);
      /* state transitions */
      if (next_path == FRONT_P) turtle_state = S_1;
      else if (next_path == LEFT_P) turtle_state = S_4;
      else if (next_path == RIGHT_P) turtle_state = S_5;
      else if (next_path == BACK_P) turtle_state = S_3;
      else ROS_ERROR("Unlisted combination at S7");
      break;
    }

    default:
      ROS_ERROR("Unrecognized turtle state");
      break;
  }

  int32_t visit_count = visitGet(turtle_coord);
  return visit_count;
}
