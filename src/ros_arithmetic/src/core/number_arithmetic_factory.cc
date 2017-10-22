/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/

/**
* @file       Number Arithmetic Factory
*
* @author     Sasi Kiran <Sasi.Alur@kpit.com>
*
* @date       18-Oct-2017
*
* @brief      Implementation class for factory
**/

/*! Include files */
#include <cstddef>
#include "ros_arithmetic/core/number_arithmetic_factory.h"

/*! Class Definitions */
NumberArithmeticFactory::NumberArithmeticFactory() {
  number_arithmetic_ = nullptr;
}
NumberArithmeticFactory::~NumberArithmeticFactory() {
  if ( nullptr != number_arithmetic_ ) {
    delete number_arithmetic_;
    number_arithmetic_ = nullptr;
  }
}

/**
* @brief Implements the creation of arithmetic operation object
**/
void NumberArithmeticFactory::CreateArithmeticOperation(
                              NumberArithmeticInterface *number_arithmetic) {
  if ( nullptr != number_arithmetic_ ) {
    delete number_arithmetic_;
    number_arithmetic_ = nullptr;
  }
  number_arithmetic_ = number_arithmetic;
}

/**
* @brief Implements returning pointer to NumberArithmeticInterface
**/
NumberArithmeticInterface* NumberArithmeticFactory::GetArithmeticOperation() {
  return number_arithmetic_;
}

/**
* @brief Implements arithmetic operation Execution
**/
uint32_t NumberArithmeticFactory::ExecuteArithmeticOperation(uint32_t value1,
                                                            uint32_t value2) {
  uint32_t number_arithmetic;

  if (nullptr != number_arithmetic_)
    number_arithmetic = number_arithmetic_->DoArithmeticOperation(value1,
                                                                 value2);
  else
    number_arithmetic = 0;

  return number_arithmetic;
}

