/*
 * 18-642 Unit Testing mock functions for ros
 * Code by Ziyue Zhang
 * ANDREW ID: ziyuez
 * LAST UPDATE: Nov 12, 2021
 */

#include "ziyuez_student_mock.h"
#include <iostream>

static bool mock_error = false;

/* Dummy ROS_ERROR */
void ROS_ERROR(std::string e) {
  mock_error = true;
  std::cout << e << std::endl;
}

void setErr(bool err_val) {
  mock_error = err_val;
}

bool getErr(){
  return mock_error;
}