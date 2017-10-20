/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file number_generator_srand.cc
* @author Rajat Jayanth Shetty  <rajat.shetty@kpit.com>
* @date   18 Oct 2017
* @brief  Default C++ SRand implemenation of random number generator 
*
*/

/* include files */
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
NumberGeneratorSRand::NumberGeneratorSRand(uint32_t _nMaxRandomValue, uint32_t _nMinRandomValue) 
  : current_random_number_(0) {
  SetRandomValRange(_nMaxRandomValue, _nMinRandomValue);
  srandom(static_cast<uint32_t>(time(0)));
}

/**
* Function name: GenerateNumber()
*
* @brief       Function call containing the actual implementation of the Number Generator
*
**/
uint32_t NumberGeneratorSRand::GenerateNumber() {

  current_seed_ = random();
  return current_seed_;
}

/**
* Function name: GetGeneratorName()
*
* @brief Function call to query Generator name or Implementation name
**/
std::string NumberGeneratorSRand::GetGeneratorName() {
  return "Default CPP SRand Generator";
}

/**
* Function name: SetRandomValRange()
*
* @brief Function call to set the the random genrator range
*
* The Range between which the random numbers need to be generator.
* The method does a basic check to verify is the _nMaxRandomValue is greater than _nMinRandomValue
* If the _nMinRandomValue is greater than _nMaxRandomValue, the values swapped
**/
void NumberGeneratorSRand::SetRandomValRange(uint32_t _nMaxRandomValue, uint32_t _nMinRandomValue) {
  if (_nMaxRandomValue > _nMinRandomValue) {
    max_random_value_ = _nMaxRandomValue;
    min_random_value_ = _nMinRandomValue;
  }
  else {
    max_random_value_ = _nMinRandomValue;
    min_random_value_ = _nMaxRandomValue;
  }
}

/**
* Function name: GetGeneratedNumber()
*
* @brief Function call to query for Number generator to provide a Random Number
*
* The function internally call the Implementation method for random number generator
**/
uint32_t NumberGeneratorSRand::GetGeneratedNumber() {
  current_random_number_ = min_random_value_ + (GenerateNumber() % (max_random_value_ - min_random_value_));
  return current_random_number_;
}

