/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/

/**
* @file
* @author
* @date
* @brief
*
*
*/
#include "ros_arithmetic/app/multiplier_arithmetic.h"
#include "ros/ros.h"

NumberMultiplier::NumberMultiplier() : NumberArithematicInterface() {
}

uint32_t NumberMultiplier::DoArithematicOperation(uint32_t value1, uint32_t value2) {
  multiplier_value_ = (value1)*(value2);
  return multiplier_value_;
}
