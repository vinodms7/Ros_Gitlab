/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file number_generator_lcg.cc
* @author Rajat Jayanth Shetty  <rajat.shetty@kpit.com>
* @date 18 Oct 2017
* @brief    Linear Congruential Generator Implementation
*
* The Linear Congruential Generator (LCG) is based on Park & Miller
* algorithm found in "Numerical Recipes".
*
*/

/* include files */
#include "ros_number_generator/app/number_generator_lcg.h"
#include "ros/ros.h"

/** This uses the Park & Miller algorithm found in "Numerical Recipes"
 *  Define the constants for the Park & Miller algorithm */
const uint64_t aConstPMA = 16807;       // 7^5
const uint64_t mConstPMA = 2147483647;  // 2^32 - 1 (Largest Prime number)

/**  Schrage's algorithm constants  */
const uint64_t qConstSA = 127773;
const uint64_t rConstSA = 2836;
 
/**
* Function name: NumberGeneratorLCG()
*
* @brief Constructor for LCG generator
*
* The constructor initializes the Random generator and specifies the range between which random
* numbers need to be generator.
* The seed value is initialize using inbuilt srandom()  and random() function call
**/
NumberGeneratorLCG::NumberGeneratorLCG(uint32_t _nMaxRandomValue, uint32_t _nMinRandomValue)
  : current_random_number_(0) {
    SetRandomValRange(_nMaxRandomValue, _nMinRandomValue);

  srand(static_cast<uint32_t>(time(0)));
  current_seed_  = random();
}

/**
* Function name: GenerateNumber()
*
* @brief       Function call containing the actual implementation of the Number Generator
*
**/
uint32_t NumberGeneratorLCG::GenerateNumber() {
  uint64_t kRatio = current_seed_ / qConstSA;

  current_seed_ = aConstPMA * (current_seed_ - (kRatio * qConstSA)) \
                      - (rConstSA * kRatio);
  if (current_seed_ < 0) {
    current_seed_ += mConstPMA;
  }

  return current_seed_;
}

/**
* Function name: GetGeneratorName()
*
* @brief Function call to query Generator name or Implementation name
**/
std::string NumberGeneratorLCG::GetGeneratorName() {
  return "Linear Congruential Generator";
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
void NumberGeneratorLCG::SetRandomValRange(uint32_t _nMaxRandomValue, uint32_t _nMinRandomValue) {
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
uint32_t NumberGeneratorLCG::GetGeneratedNumber() {
  current_random_number_ = min_random_value_ + (GenerateNumber() % (max_random_value_ - min_random_value_));
  return current_random_number_;
}
