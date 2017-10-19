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
#include "ros_number_generator/app/number_generator_srand.h"

NumberGeneratorSRand::NumberGeneratorSRand(uint32_t _nMaxRandomValue, uint32_t _nMinRandomValue)
  :NumberGenerator(_nMaxRandomValue, _nMinRandomValue) {
  srandom(static_cast<uint32_t>(time(0)));
}

uint32_t NumberGeneratorSRand::GenerateNumber() {

  current_seed_ = random();
  return current_seed_;
}
std::string NumberGeneratorSRand::GetGeneratorName() {
  return "Default CPP SRand Generator";
}
