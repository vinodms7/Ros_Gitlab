/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file     number_generator_factory.cc
* @author   Rajat Jayanth Shetty <rajat.shetty@kpit.com>
* @date     18 Oct 2017
* @brief    This file declares Number Generator Factory Class
*
*/

/* include files */
#include "ros_number_generator/core/number_generator_interface.h"
#include "ros_number_generator/core/number_generator_factory.h"

/**
* Function name: NumberGeneratorFactory()
*
* @brief Constructor for the Number Generator Factory class
*
* Initializes the Number Generator factory class
**/
NumberGeneratorFactory::NumberGeneratorFactory() {
  number_generator_ = NULL;
}

/**
* Function name: ~NumberGeneratorFactory()
*
* @brief Destructor for the Number Generator Factory class
*
**/
NumberGeneratorFactory::~NumberGeneratorFactory() {
  if (NULL != number_generator_) {
    delete number_generator_;
    number_generator_ = NULL;
  }
}

/**
* Function name: CreateGenerator()
*  
* @brief Receives an instance for the Number Generator
*
* If an instance of Number generator already exist, the current method will 
* delete the previous instance before assigning a new Number Generator
*
**/
void NumberGeneratorFactory::CreateGenerator(NumberGenerator *pNumberGen) {
  if (NULL != number_generator_) {
    delete number_generator_;
    number_generator_ = NULL;
  }
  number_generator_ = pNumberGen;
}
/**
* Function name: GetGenerator() const
*
* @brief Returns a pointer to the instance of the Number generator
*
*/
NumberGenerator * NumberGeneratorFactory::GetGenerator() {
  return number_generator_;
}
