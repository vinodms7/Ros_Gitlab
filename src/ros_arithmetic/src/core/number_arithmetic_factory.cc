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
#include <cstddef>

#include "ros_arithmetic/core/number_arithmetic_factory.h"

NumberArithematicFactory::NumberArithematicFactory() {
  number_multiplier_ = NULL;
}
NumberArithematicFactory::~NumberArithematicFactory() {
  if (NULL != number_multiplier_) {
    delete number_multiplier_;
    number_multiplier_ = NULL;
  }
}

void NumberArithematicFactory::CreateMultiplier(NumberArithematicInterface *pNumberMult) {
  if (NULL != number_multiplier_) {
    delete number_multiplier_;
    number_multiplier_ = NULL;
  }
  number_multiplier_ = pNumberMult;
}

NumberArithematicInterface* NumberArithematicFactory::GetMultiplier() {
  return number_multiplier_;
}
