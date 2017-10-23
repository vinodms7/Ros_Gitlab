/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file     number_generator_factory.cc
* 
* @author   Rajat Jayanth Shetty <rajat.shetty@kpit.com>
* 
* @date     18-Oct-2017
* 
* @brief    This file declares Number Generator Factory Class
*
*/

/*! Include files */
#include "ros_number_generator/core/number_generator_interface.h"
#include "ros_number_generator/core/number_generator_factory.h"

/**
* Function name: NumberGeneratorFactory()
*
* @brief Constructor for the Number Generator Factory class
*
* Initializes the Number Generator factory class
**/
template<class T>
NumberGeneratorFactory<T>::NumberGeneratorFactory() {
  number_generator_ = nullptr;
}

/**
* Function name: ~NumberGeneratorFactory()
*
* @brief Destructor for the Number Generator Factory class
*
**/
template<class T>
NumberGeneratorFactory<T>::~NumberGeneratorFactory() {
  if ( nullptr != number_generator_ ) {
    delete number_generator_;
    number_generator_ = nullptr;
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
template<class T>
void NumberGeneratorFactory<T>::CreateGenerator(
                                       NumberGenerator<T> *number_gen) {
  if ( nullptr != number_generator_ ) {
    delete number_generator_;
    number_generator_ = nullptr;
  }
  number_generator_ = number_gen;
}

/**
* Function name: getGenerator()
*  
* @brief Returns the instance of the Number Generator
*
* If an instance of Number generator already exist, the current method will
* return the instance 
*
**/
template<class T>
NumberGenerator<T>* NumberGeneratorFactory<T>::getGenerator() {
  return number_generator_;
}

/**
* Function name: ExecuteGenerator()
*  
* @brief Execute the Number Generator functionality to get Number
*
**/
template<class T>
T NumberGeneratorFactory<T>::ExecuteGenerator() const {
  T number_generated;
  if ( nullptr != number_generator_ )
    number_generated =  number_generator_->GetGeneratedNumber();
  else
    number_generated = 0;
  return number_generated;
}

