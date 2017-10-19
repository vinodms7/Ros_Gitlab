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
#include "ros_number_generator/core/number_generator_interface.h"
#include "ros_number_generator/core/number_generator_factory.h"

NumberGeneratorFactory::NumberGeneratorFactory() {
  number_generator_ = NULL;
}
NumberGeneratorFactory::~NumberGeneratorFactory() {
  if (NULL != number_generator_) {
    delete number_generator_;
    number_generator_ = NULL;
  }
}

void NumberGeneratorFactory::CreateGenerator(NumberGenerator *pNumberGen) {
  if (NULL != number_generator_) {
    delete number_generator_;
    number_generator_ = NULL;
  }
  number_generator_ = pNumberGen;
}
NumberGenerator * NumberGeneratorFactory::GetGenerator() const {
  return number_generator_;
}
