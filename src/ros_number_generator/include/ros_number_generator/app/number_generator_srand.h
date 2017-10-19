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
#ifndef NUMBER_GENERATOR_SRAND_H
#define NUMBER_GENERATOR_SRAND_H

#include "ros_number_generator/core/number_generator_interface.h"

class NumberGeneratorSRand : public NumberGenerator {
public:
  explicit NumberGeneratorSRand(uint32_t nMaxRandomValue = 1000, uint32_t nMinRandomValue = 0);
  virtual ~NumberGeneratorSRand() {}
  virtual std::string GetGeneratorName();

protected:
  uint32_t GenerateNumber();
};

#endif /* NUMBER_GENERATOR_SRAND_H */
