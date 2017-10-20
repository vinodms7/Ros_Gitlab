/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/

/**
* @file		Number Arithmetic Factory
* @author       Sasi Kiran	
* @date         18 oct 2017
* @brief        Implementation class for factory
*
*
**/
#include <cstddef>

#include "ros_arithmetic/core/number_arithmetic_factory.h"

NumberArithmeticFactory::NumberArithmeticFactory() {
  number_arithmetic_ = NULL;
}
NumberArithmeticFactory::~NumberArithmeticFactory() {
  if (NULL != number_arithmetic_) {
    delete number_arithmetic_;
    number_arithmetic_ = NULL;
  }
}

void NumberArithmeticFactory::CreateArithmeticOperation(NumberArithmeticInterface *number_arithmetic) {
  if (NULL != number_arithmetic_) {
    delete number_arithmetic_;
    number_arithmetic_ = NULL;
  }
  number_arithmetic_ = number_arithmetic;
}

NumberArithmeticInterface* NumberArithmeticFactory::GetArithmeticOperation() {
  return number_arithmetic_;
}
