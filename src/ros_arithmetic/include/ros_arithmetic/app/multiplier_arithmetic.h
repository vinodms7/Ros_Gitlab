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
#ifndef __MULTIPLIER_ARITHMETIC_H_
#define __MULTIPLIER_ARITHMETIC_H_

#include "ros_arithmetic/core/number_arithmetic_interface.h"

class NumberMultiplier : public NumberArithematicInterface {
 public:
  NumberMultiplier();
  virtual ~NumberMultiplier() {}

  uint32_t DoArithematicOperation(uint32_t value1,
                                       uint32_t value2);
 private:
  uint32_t multiplier_value_;
};

#endif /* __MULTIPLIER_ARITHMETIC_H_ */
