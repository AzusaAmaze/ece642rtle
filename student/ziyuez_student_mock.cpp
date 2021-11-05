/*
 * 18-642 Unit Testing Example
 * Example code by Milda Zizyte
 */

#include "ziyuez_student_mock.h"
#include <iostream>

/* Dummy ROS_ERROR */
void ROS_ERROR(std::string e) {
  mock_error = true;
  std::cout << e << std::endl;
}