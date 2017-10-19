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
#ifndef NUMBER_GENERATOR_LCG_H
#define NUMBER_GENERATOR_LCG_H

#include "ros_number_generator/core/number_generator_interface.h"

class NumberGeneratorLCG : public NumberGenerator {
public:
  NumberGeneratorLCG(uint32_t nMaxRandomValue = 1000, uint32_t nMinRandomValue = 0);
  virtual ~NumberGeneratorLCG() {}
  virtual std::string GetGeneratorName();

protected:
  virtual uint32_t GenerateNumber();
};
#endif  /* NUMBER_GENERATOR_LCG_H */
