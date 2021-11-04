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
 * JuncUpdate() (branch coverage)
 * pickPath() (switch cases, if statements)
 * atPath() & visitPath() (switch cases)
 * studentTurtleTransit() (separate state machine, test all transitions)
 * studentTurtleStep() (separate state machine, test side effects at each state)
 * Turn
*/

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

void test_t1() {
  int32_t mock_count = 0;
  setOrient(left);
  setState(S_0);
  setCoord({13,13});
  visitSet({13,13}, 0);
  juncSet({13,13}, {BLOCK, BLOCK, BLOCK, BLOCK, BLOCK, 0, 0, 0, 0});

  turtleMove next_move = studentTurtleStep();
  mock_count = turtleStateTransit(mock_count, false, false, true);

  states new_state = getState();
  int32_t new_orient = getOrient();
  map_pos_t new_coord = getCoord();
  int32_t new_visit = visitGet({13, 13});
  block_info_t new_junc = juncGet({13, 13});

  CU_ASSERT_EQUAL(mock_count, 0);
  CU_ASSERT_EQUAL(next_move, STOP);
  CU_ASSERT_EQUAL(new_state, S_0);
  CU_ASSERT_EQUAL(new_orient, left);
  CU_ASSERT_EQUAL(new_coord.row, 13);
  CU_ASSERT_EQUAL(new_coord.col, 13);
  CU_ASSERT_EQUAL(new_visit, 0);
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
  if ((NULL == CU_add_test(pSuite, "test of transition T1", test_t1)))
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

