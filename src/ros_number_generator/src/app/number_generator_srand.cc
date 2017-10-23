/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file    number_generator_srand.cc
* 
* @author  Rajat Jayanth Shetty  <rajat.shetty@kpit.com>
* 
* @date    18 Oct 2017
* 
* @brief   Default C++ SRand implemenation of random number generator 
*
*/

/*! Include files */
#include <string>
#include <cmath>
#include "ros_number_generator/app/number_generator_srand.h"

/**
* Function name: NumberGeneratorSRand()
*
* @brief Constructor for SRand generator
*
* The constructor initializes the Random generator and specifies the range between which random
* numbers need to be generator.
* The seed value is initialize using inbuilt srandom()  and random() function call
**/
template<class T>
NumberGeneratorSRand<T>::NumberGeneratorSRand(T max_random_value,
                                            T min_random_value)
                                         : current_random_number_(0) {
  SetRandomValRange(max_random_value, min_random_value);
  srandom(static_cast<T>(time(0)));
}

template<class T>
NumberGeneratorSRand<T>::~NumberGeneratorSRand() {
}

/**
* Function name: GenerateNumber()
*
* @brief       Function call containing the actual implementation of the Number Generator
*
**/
template<class T>
T NumberGeneratorSRand<T>::GenerateNumber() {
  current_seed_ = random();
  return current_seed_;
}

/**
* Function name: GetGeneratorName()
*
* @brief Function call to query Generator name or Implementation name
**/
template<class T>
std::string NumberGeneratorSRand<T>::GetGeneratorName() const {
  return "Default CPP SRand Generator";
}

/**
* Function name: SetRandomValRange()
*
* @brief Function call to set the the random genrator range
*
* The Range between which the random numbers need to be generator.
* The method does a basic check to verify is the max_random_value is greater than min_random_value
* If the min_random_value is greater than max_random_value, the values swapped
**/
template<class T>
void NumberGeneratorSRand<T>::SetRandomValRange(T max_random_value,
                                            T min_random_value) {
  if ( max_random_value > min_random_value ) {
    max_random_value_ = max_random_value;
    min_random_value_ = min_random_value;
  } else {
    max_random_value_ = min_random_value;
    min_random_value_ = max_random_value;
  }
}

/**
* Function name: GetGeneratedNumber()
*
* @brief Function call to query for Number generator to provide a Random Number
*
* The function internally call the Implementation method for random number generator
**/
template<class T>
T NumberGeneratorSRand<T>::GetGeneratedNumber() {
  double random_int = (double)GenerateNumber();
  
  double random_range = (double)max_random_value_ - (double)min_random_value_;

  double random_numbr = (double)min_random_value_ + ((random_int/random_range)-
                                 floor(random_int/random_range))*random_range;

  current_random_number_ = static_cast<T>(random_numbr);

  return current_random_number_;
}

