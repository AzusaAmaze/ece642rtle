/*
 * 18-642 Unit Testing Example
 * Example code by Milda Zizyte
 * This code exercises Transitions T1 and T2 of the Project 8
 * dummy turtle statechart. It uses the CUnit framework (cunit.sourceforge.net)
 */

#include "ziyuez_student_mock.h"
#include <CUnit/Basic.h>

/*
 * List of things to be tested: 
 * JuncUpdate() (branch coverage, tested at S2 & extra) ok
 * pickPath() (switch cases, if statements, tested at S7 & extra) ok
 * atPath() & visitPath() (switch cases, tested at S7 & extra) ok
 * turtleStateTransit() (test all transitions & default)
 * studentTurtleStep() (test side effects at each state & default)
 * turnLeftRight() (tested in S2, S3, S4, S5 & extra) ok
 * incPath() (tested in S7 & extra) ok
 * orientedCoord() (tested in S1 & extra) ok
*/

// TODO: extra test for S1 orientedCoord()
// TODO: extra test for S2 updateJunc()

void test_Block(block_info_t actual, block_info_t expected) {
  CU_ASSERT_EQUAL(actual.up_block, expected.up_block);
  CU_ASSERT_EQUAL(actual.down_block, expected.down_block);
  CU_ASSERT_EQUAL(actual.left_block, expected.left_block);
  CU_ASSERT_EQUAL(actual.right_block, expected.right_block);
  CU_ASSERT_EQUAL(actual.curr_block, expected.curr_block);
  CU_ASSERT_EQUAL(actual.up_count, expected.up_count);
  CU_ASSERT_EQUAL(actual.down_count, expected.down_count);
  CU_ASSERT_EQUAL(actual.left_count, expected.left_count);
  CU_ASSERT_EQUAL(actual.right_count, expected.right_count);
}


// Tests for state chart transitions

// @ S0
void test_t0() {
  int32_t mock_count = 0;
  setOrient(left);
  setState(S_0);
  setCoord({13,13});
  visitSet({13,13}, 1);
  juncSet({13,13}, {BLOCK, BLOCK, BLOCK, PATH, BLOCK, -1, -1, -1, 0});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, false, false, true);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, STOP);
  CU_ASSERT_EQUAL(new_state, S_0);
  CU_ASSERT_EQUAL(new_orient, left);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {BLOCK, BLOCK, BLOCK, PATH, BLOCK, -1, -1, -1, 0});
}

// first_time && -- && goal @ S1
void test_t1() {
  int32_t mock_count = 2;
  setOrient(up);
  setState(S_1);
  setCoord({13,13});
  visitSet({12,13}, 0); // one block up
  juncSet({12,13}, {BLOCK, BLOCK, BLOCK, BLOCK, BLOCK, 0, 0, 0, 0});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, false, true);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, MOVE);
  CU_ASSERT_EQUAL(new_state, S_0);
  CU_ASSERT_EQUAL(new_orient, up);
  CU_ASSERT_EQUAL(new_coord.row, 12);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {BLOCK, BLOCK, BLOCK, BLOCK, BLOCK, 0, 0, 0, 0});
}

// first_time && -- && not goal @ S1
void test_t2() {
  int32_t mock_count = 2;
  setOrient(left);
  setState(S_1);
  setCoord({13,13});
  visitSet({13,12}, 0); // one block up
  juncSet({13,12}, {BLOCK, BLOCK, BLOCK, BLOCK, BLOCK, 0, 0, 0, 0});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, false, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, MOVE);
  CU_ASSERT_EQUAL(new_state, S_2);
  CU_ASSERT_EQUAL(new_orient, left);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 12);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {BLOCK, BLOCK, BLOCK, BLOCK, BLOCK, 0, 0, 0, 0});
}

// not first_time && -- && --; curr_block = BLOCk @ S1
void test_t3() {
  int32_t mock_count = 2;
  setOrient(right);
  setState(S_1);
  setCoord({13,13});
  visitSet({13,14}, 1); // one block up
  juncSet({13,14}, {BLOCK, BLOCK, PATH, BLOCK, BLOCK, -1, -1, 0, -1});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, false, true, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, MOVE);
  CU_ASSERT_EQUAL(new_state, S_3);
  CU_ASSERT_EQUAL(new_orient, right);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 14);
  CU_ASSERT_EQUAL(new_visit, 2);
  test_Block(new_junc, {BLOCK, BLOCK, PATH, BLOCK, BLOCK, -1, -1, 0, -1});
}

// not first_time && -- && --; curr_block = PATH @ S1
void test_t4() {
  int32_t mock_count = 2;
  setOrient(down);
  setState(S_1);
  setCoord({13,13});
  visitSet({14,13}, 1); // one block up
  juncSet({14,13}, {PATH, PATH, BLOCK, BLOCK, PATH, 0, 0, -1, -1});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, false, false, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, MOVE);
  CU_ASSERT_EQUAL(new_state, S_1);
  CU_ASSERT_EQUAL(new_orient, down);
  CU_ASSERT_EQUAL(new_coord.row, 14);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 2);
  test_Block(new_junc, {PATH, PATH, BLOCK, BLOCK, PATH, 0, 0, -1, -1});
}

// not first_time && -- && --; curr_block = JUNC @ S1
void test_t5() {
  int32_t mock_count = 2;
  setOrient(down);
  setState(S_1);
  setCoord({13,13});
  visitSet({14,13}, 1); // one block up
  juncSet({14,13}, {PATH, PATH, PATH, BLOCK, JUNC, 1, 1, 0, -1});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, false, false, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, MOVE);
  CU_ASSERT_EQUAL(new_state, S_7);
  CU_ASSERT_EQUAL(new_orient, down);
  CU_ASSERT_EQUAL(new_coord.row, 14);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 2);
  test_Block(new_junc, {PATH, PATH, PATH, BLOCK, JUNC, 1, 1, 0, -1});
}

// first_time && not bumped && --; turn_count != 0 @ S2
void test_t6() {
  int32_t mock_count = 0;
  setOrient(left);
  setState(S_2);
  setCoord({13,13});
  visitSet({13,13}, 1); // one block up
  juncSet({13,13}, {BLOCK, BLOCK, BLOCK, BLOCK, BLOCK, 0, 0, 0, 0});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, false, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 1);
  CU_ASSERT_EQUAL(next_move, LEFT);
  CU_ASSERT_EQUAL(new_state, S_2);
  CU_ASSERT_EQUAL(new_orient, down);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {BLOCK, PATH, BLOCK, BLOCK, JUNC, 0, 0, 0, 0});
}

// first_time && bumped && --; turn_count == 0; curr_block == BLOCK @ S2
void test_t7() {
  int32_t mock_count = 3;
  setOrient(up);
  setState(S_2);
  setCoord({13,13});
  visitSet({13,13}, 1); // one block up
  juncSet({13,13}, {BLOCK, BLOCK, BLOCK, PATH, BLOCK, -1, -1, 0, 0});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, true, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, LEFT);
  CU_ASSERT_EQUAL(new_state, S_3);
  CU_ASSERT_EQUAL(new_orient, left);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {BLOCK, BLOCK, BLOCK, PATH, BLOCK, -1, -1, -1, 0});
}

// first_time && not bumped && --; turn_count == 0; curr_block == PATH @ S2
void test_t8() {
  int32_t mock_count = 3;
  setOrient(down);
  setState(S_2);
  setCoord({13,13});
  visitSet({13,13}, 1); // one block up
  juncSet({13,13}, {BLOCK, BLOCK, PATH, BLOCK, BLOCK, -1, -1, 0, 0});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, false, true);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, LEFT);
  CU_ASSERT_EQUAL(new_state, S_1);
  CU_ASSERT_EQUAL(new_orient, right);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {BLOCK, BLOCK, PATH, PATH, PATH, -1, -1, 0, 0});
}

// first_time && not bumped && --; turn_count == 0; curr_block == JUNC @ S2
void test_t9() {
  int32_t mock_count = 3;
  setOrient(right);
  setState(S_2);
  setCoord({13,13});
  visitSet({13,13}, 1); // one block up
  juncSet({13,13}, {BLOCK, PATH, PATH, BLOCK, JUNC, 0, 0, 0, -1});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, true, true);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, LEFT);
  CU_ASSERT_EQUAL(new_state, S_7);
  CU_ASSERT_EQUAL(new_orient, up);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {BLOCK, PATH, PATH, BLOCK, JUNC, -1, 0, 0, -1});
}

// -- && -- && --; turn_count != 2 @ S3
void test_t10() {
  int32_t mock_count = 0;
  setOrient(up);
  setState(S_3);
  setCoord({13,13});
  visitSet({13,13}, 1); // one block up
  juncSet({13,13}, {PATH, PATH, BLOCK, BLOCK, BLOCK, 0, 0, -1, -1});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, false, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 1);
  CU_ASSERT_EQUAL(next_move, LEFT);
  CU_ASSERT_EQUAL(new_state, S_3);
  CU_ASSERT_EQUAL(new_orient, left);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {PATH, PATH, BLOCK, BLOCK, BLOCK, 0, 0, -1, -1});
}

// -- && -- && --; turn_count == 2 @ S3
void test_t11() {
  int32_t mock_count = 1;
  setOrient(left);
  setState(S_3);
  setCoord({13,13});
  visitSet({13,13}, 1); // one block up
  juncSet({13,13}, {PATH, PATH, BLOCK, BLOCK, BLOCK, 0, 0, -1, -1});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, true, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 2);
  CU_ASSERT_EQUAL(next_move, LEFT);
  CU_ASSERT_EQUAL(new_state, S_1);
  CU_ASSERT_EQUAL(new_orient, down);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {PATH, PATH, BLOCK, BLOCK, BLOCK, 0, 0, -1, -1});
}

// -- && -- && -- @ S4
void test_t12() {
  int32_t mock_count = 0;
  setOrient(right);
  setState(S_4);
  setCoord({13,13});
  visitSet({13,13}, 1); // one block up
  juncSet({13,13}, {PATH, BLOCK, PATH, BLOCK, JUNC, 1, -1, 1, -1});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, true, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, LEFT);
  CU_ASSERT_EQUAL(new_state, S_1);
  CU_ASSERT_EQUAL(new_orient, up);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {PATH, BLOCK, PATH, BLOCK, JUNC, 1, -1, 1, -1});
}

// -- && -- && -- @ S5
void test_t13() {
  int32_t mock_count = 0;
  setOrient(right);
  setState(S_5);
  setCoord({13,13});
  visitSet({13,13}, 1); // one block up
  juncSet({13,13}, {BLOCK, PATH, PATH, BLOCK, JUNC, -1, 1, 1,-1});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, true, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, RIGHT);
  CU_ASSERT_EQUAL(new_state, S_1);
  CU_ASSERT_EQUAL(new_orient, down);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {BLOCK, PATH, PATH, BLOCK, JUNC, -1, 1, 1, -1});
}

// -- && -- && goal @ S6
void test_t14() {
  int32_t mock_count = 0;
  setOrient(down);
  setState(S_6);
  setCoord({13,13});
  visitSet({13,13}, 0); // one block up
  juncSet({13,13}, {BLOCK, BLOCK, BLOCK, BLOCK, BLOCK, 0, 0, 0, 0});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, false, true);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, STOP);
  CU_ASSERT_EQUAL(new_state, S_0);
  CU_ASSERT_EQUAL(new_orient, down);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {BLOCK, BLOCK, BLOCK, BLOCK, BLOCK, 0, 0, 0, 0});
}

// -- && -- && not goal @ S6
void test_t15() {
  int32_t mock_count = 0;
  setOrient(left);
  setState(S_6);
  setCoord({13,13});
  visitSet({13,13}, 0); // one block up
  juncSet({13,13}, {BLOCK, BLOCK, BLOCK, BLOCK, BLOCK, 0, 0, 0, 0});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, false, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, STOP);
  CU_ASSERT_EQUAL(new_state, S_2);
  CU_ASSERT_EQUAL(new_orient, left);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {BLOCK, BLOCK, BLOCK, BLOCK, BLOCK, 0, 0, 0, 0});
}

// first_time && -- && not goal @ S7
void test_t16() {
  int32_t mock_count = 0;
  setOrient(left);
  setState(S_7);
  setCoord({13,13});
  visitSet({13,13}, 1); // one block up
  juncSet({13,13}, {PATH, BLOCK, PATH, PATH, JUNC, 0, -1, 0, 0});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, false, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, STOP);
  CU_ASSERT_EQUAL(new_state, S_1);
  CU_ASSERT_EQUAL(new_orient, left);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {PATH, BLOCK, PATH, PATH, JUNC, 0, -1, 1, 1});
}

// first_time && -- && not goal; branch coverage for return orientation at right @ S7
void test_t16_extra() {
  int32_t mock_count = 0;
  setOrient(right);
  setState(S_7);
  setCoord({13,13});
  visitSet({13,13}, 1); // one block up
  juncSet({13,13}, {BLOCK, PATH, BLOCK, PATH, JUNC, -1, 0, -1, 0});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, false, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, STOP);
  CU_ASSERT_EQUAL(new_state, S_1);
  CU_ASSERT_EQUAL(new_orient, right);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {BLOCK, PATH, BLOCK, PATH, JUNC, -1, 0, -1, 1});
}

// first_time && -- && not goal @ S7
void test_t17() {
  int32_t mock_count = 0;
  setOrient(right);
  setState(S_7);
  setCoord({13,13});
  visitSet({13,13}, 1); // one block up
  juncSet({13,13}, {PATH, BLOCK, PATH, BLOCK, JUNC, 0, -1, 0, -1});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, true, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, STOP);
  CU_ASSERT_EQUAL(new_state, S_4);
  CU_ASSERT_EQUAL(new_orient, right);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {PATH, BLOCK, PATH, BLOCK, JUNC, 1, -1, 1, -1});
}

// not first_time && -- && not goal @ S7
void test_t18() {
  int32_t mock_count = 0;
  setOrient(down);
  setState(S_7);
  setCoord({13,13});
  visitSet({13,13}, 3); // one block up
  juncSet({13,13}, {PATH, PATH, PATH, PATH, JUNC, 2, 1, 0, 1});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, false, false, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, STOP);
  CU_ASSERT_EQUAL(new_state, S_5);
  CU_ASSERT_EQUAL(new_orient, down);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 3);
  test_Block(new_junc, {PATH, PATH, PATH, PATH, JUNC, 3, 1, 1, 1});
}

// not first_time && -- && not goal @ S7
void test_t19() {
  int32_t mock_count = 0;
  setOrient(up);
  setState(S_7);
  setCoord({13,13});
  visitSet({13,13}, 2); // one block up
  juncSet({13,13}, {BLOCK, PATH, PATH, PATH, JUNC, -1, 0, 1, 1});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, false, false, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, STOP);
  CU_ASSERT_EQUAL(new_state, S_3);
  CU_ASSERT_EQUAL(new_orient, up);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 2);
  test_Block(new_junc, {BLOCK, PATH, PATH, PATH, JUNC, -1, 2, 1, 1});
}

// Tests for switch default cases for 100% branch coverage

// test default error case for incPath
void test_incPath() {
  setErr(false);

  incPath(dir_err);

  CU_ASSERT_EQUAL(getErr(), true);
}

// test default error case for orientedCoord
void test_orientedCoord() {
  setErr(false);

  map_pos_t pos = orientedCoord(dir_err);

  CU_ASSERT_EQUAL(getErr(), true);
}

// test default error case for atPath
void test_atPath() {
  setErr(false);

  int32_t is_path = atPath(dir_err);

  CU_ASSERT_EQUAL(getErr(), true);
}

// test default error case for visitPath
void test_visitPath() {
  setErr(false);

  int32_t path_count = visitPath(dir_err);

  CU_ASSERT_EQUAL(getErr(), true);
}

// test default error case for turnLeftRight
void test_turnLeftRight() {
  setErr(false);

  int32_t new_orient = turnLeftRight(dir_err, true);

  CU_ASSERT_EQUAL(getErr(), true);
}

// test default error case for pickPath
void test_pickPath() {
  setErr(false);

  path_type new_path = pickPath(dir_err, true);

  CU_ASSERT_EQUAL(getErr(), true);
}

// test default error case for juncUpdate
void test_juncUpdate() {
  setErr(false);

  juncUpdate(dir_err, false, 1);

  CU_ASSERT_EQUAL(getErr(), true);
}

// test default error case for turtleStateTransit
void test_turtleStateTransit() {
  setErr(false);
  setState(state_err);

  int32_t turn_count = turtleStateTransit(0, true, true, false);

  CU_ASSERT_EQUAL(getErr(), true);
}

// test default error case for studentTurtleStep
void test_studentTurtleStep() {
  setErr(false);
  setState(state_err);

  turtleMove next_move = studentTurtleStep();

  CU_ASSERT_EQUAL(getErr(), true);
}

// Test to attempt 100% data coverage, inputs: bumped & goal

// true & true @ S0
void test_S0_extra() {
  int32_t mock_count = 0;
  setOrient(up);
  setState(S_0);
  setCoord({13,13});
  visitSet({13,13}, 1);
  juncSet({13,13}, {BLOCK, BLOCK, BLOCK, PATH, BLOCK, -1, -1, -1, 0});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, false, true, true);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, STOP);
  CU_ASSERT_EQUAL(new_state, S_0);
  CU_ASSERT_EQUAL(new_orient, up);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {BLOCK, BLOCK, BLOCK, PATH, BLOCK, -1, -1, -1, 0});
}

// true & true @ S6
void test_S6_extra1() {
  int32_t mock_count = 0;
  setOrient(down);
  setState(S_6);
  setCoord({13,13});
  visitSet({13,13}, 0); // one block up
  juncSet({13,13}, {BLOCK, BLOCK, BLOCK, BLOCK, BLOCK, 0, 0, 0, 0});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, true, true);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, STOP);
  CU_ASSERT_EQUAL(new_state, S_0);
  CU_ASSERT_EQUAL(new_orient, down);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {BLOCK, BLOCK, BLOCK, BLOCK, BLOCK, 0, 0, 0, 0});
}

// true & false @ S6
void test_S6_extra2() {
  int32_t mock_count = 0;
  setOrient(up);
  setState(S_6);
  setCoord({13,13});
  visitSet({13,13}, 0); // one block up
  juncSet({13,13}, {BLOCK, BLOCK, BLOCK, BLOCK, BLOCK, 0, 0, 0, 0});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, true, true, false);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet(new_coord);
  block_info_t new_junc = juncGet(new_coord);

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, STOP);
  CU_ASSERT_EQUAL(new_state, S_2);
  CU_ASSERT_EQUAL(new_orient, up);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 1);
  test_Block(new_junc, {BLOCK, BLOCK, BLOCK, BLOCK, BLOCK, 0, 0, 0, 0});
}

int init() {
  // Any test initialization code goes here
  return 0;
}

int cleanup() {
  // Any test cleanup code goes here
  return 0;
}

/* Skeleton code from http://cunit.sourceforge.net/example.html */
int main() {

  CU_pSuite pSuite = NULL;

  /* initialize the CUnit test registry */
  if (CUE_SUCCESS != CU_initialize_registry())
    return CU_get_error();

  /* add a suite to the registry */
  pSuite = CU_add_suite("Suite_1", init, cleanup);
  if (NULL == pSuite) {
    CU_cleanup_registry();
    return CU_get_error();
  }

  /* add the tests to the suite */
  if ((NULL == CU_add_test(pSuite, "test of transition T0", test_t0)) || 
      (NULL == CU_add_test(pSuite, "test of transition T1", test_t1)) ||
      (NULL == CU_add_test(pSuite, "test of transition T2", test_t2)) ||
      (NULL == CU_add_test(pSuite, "test of transition T3", test_t3)) ||
      (NULL == CU_add_test(pSuite, "test of transition T4", test_t4)) ||
      (NULL == CU_add_test(pSuite, "test of transition T5", test_t5)) || 
      (NULL == CU_add_test(pSuite, "test of transition T6", test_t6)) || 
      (NULL == CU_add_test(pSuite, "test of transition T7", test_t7)) || 
      (NULL == CU_add_test(pSuite, "test of transition T8", test_t8)) || 
      (NULL == CU_add_test(pSuite, "test of transition T9", test_t9)) ||
      (NULL == CU_add_test(pSuite, "test of transition T10", test_t10)) ||
      (NULL == CU_add_test(pSuite, "test of transition T11", test_t11)) ||
      (NULL == CU_add_test(pSuite, "test of transition T12", test_t12)) ||
      (NULL == CU_add_test(pSuite, "test of transition T13", test_t13)) ||
      (NULL == CU_add_test(pSuite, "test of transition T14", test_t14)) ||
      (NULL == CU_add_test(pSuite, "test of transition T15", test_t15)) ||
      (NULL == CU_add_test(pSuite, "test of transition T16", test_t16)) ||
      (NULL == CU_add_test(pSuite, "test of transition T16 extra", test_t16_extra)) ||
      (NULL == CU_add_test(pSuite, "test of transition T17", test_t17)) ||
      (NULL == CU_add_test(pSuite, "test of transition T18", test_t18)) ||
      (NULL == CU_add_test(pSuite, "test of transition T19", test_t19)) ||
      (NULL == CU_add_test(pSuite, "test of incPath default case", test_incPath)) ||
      (NULL == CU_add_test(pSuite, "test of orientedCoord default case", test_orientedCoord)) ||
      (NULL == CU_add_test(pSuite, "test of atPath default case", test_atPath)) ||
      (NULL == CU_add_test(pSuite, "test of visitPath default case", test_visitPath)) ||
      (NULL == CU_add_test(pSuite, "test of turnLeftRight default case", test_turnLeftRight)) ||
      (NULL == CU_add_test(pSuite, "test of pickPath default case", test_pickPath)) ||
      (NULL == CU_add_test(pSuite, "test of juncUpdate default case", test_juncUpdate)) ||
      (NULL == CU_add_test(pSuite, "test of turtleStateTransit default case", test_turtleStateTransit)) ||
      (NULL == CU_add_test(pSuite, "test of studentTurtleStep default case", test_studentTurtleStep)) || 
      (NULL == CU_add_test(pSuite, "test of S0 data combination", test_S0_extra)) ||
      (NULL == CU_add_test(pSuite, "test of S6 data combination 1", test_S6_extra1)) ||
      (NULL == CU_add_test(pSuite, "test of S6 data combination 2", test_S6_extra2)))
    {
      CU_cleanup_registry();
      return CU_get_error();
    }
  
  /* Run all tests using the CUnit Basic interface */
  CU_basic_set_mode(CU_BRM_VERBOSE);
  CU_basic_run_tests();
  CU_cleanup_registry();
  return CU_get_error();

  return 0;
}

