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
#ifndef __LINEAR_CONGRUENTIAL_GENERATOR_CPP
#define __LINEAR_CONGRUENTIAL_GENERATOR_CPP

#include "ros_number_generator/app/number_generator_lcg.h"
#include "ros/ros.h"

// This uses the Park & Miller algorithm found in "Numerical Recipes"
// Define the constants for the Park & Miller algorithm
const uint64_t aConstPMA = 16807;       // 7^5
const uint64_t mConstPMA = 2147483647;  // 2^32 - 1 (Largest Prime number)

// Schrage's algorithm constants
const uint64_t qConstSA = 127773;
const uint64_t rConstSA = 2836;

// Parameter constructor
NumberGeneratorLCG::NumberGeneratorLCG(uint32_t _nMaxRandomValue, uint32_t _nMinRandomValue)
  :NumberGenerator(_nMaxRandomValue, _nMinRandomValue) {
  srand(static_cast<uint32_t>(time(0)));
  current_seed_  = random();
}

// Obtains a random uint32_t integer
uint32_t NumberGeneratorLCG::GenerateNumber() {
  uint64_t kRatio = current_seed_ / qConstSA;

  current_seed_ = aConstPMA * (current_seed_ - (kRatio * qConstSA)) \
                      - (rConstSA * kRatio);
  if (current_seed_ < 0) {
    current_seed_ += mConstPMA;
  }

  return current_seed_;
}

std::string NumberGeneratorLCG::GetGeneratorName() {
  return "Linear Congruential Generator";
}

#endif
