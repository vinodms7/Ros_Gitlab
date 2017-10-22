/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file        multiplier_arithmetic.cc
*
* @author       Sasi Kiran <Sasi.Alur@kpit.com>
*
* @date         18-Oct-2017
*
* @brief        Implementation class for multiplication functionality
**/

/*! Include files */
#include "ros_arithmetic/app/multiplier_arithmetic.h"

/*! Class Defintions */
NumberMultiplier::NumberMultiplier(): multiplier_value_(0) {
}

NumberMultiplier::~NumberMultiplier() {
}

uint32_t NumberMultiplier::DoArithmeticOperation(uint32_t value1,
                                                 uint32_t value2) {
  return DoMultiplication(value1, value2);
}

/**
* @brief Implements the functionality to perform multiplication using * operator
**/
uint32_t NumberMultiplier::DoMultiplication(uint32_t value1, uint32_t value2) {
  multiplier_value_ = (value1)*(value2);
  return multiplier_value_;
}

void NumberMultiplier::DisplayResult(uint32_t value) {
}

