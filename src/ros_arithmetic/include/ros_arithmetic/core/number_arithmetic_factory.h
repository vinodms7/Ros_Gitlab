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
#ifndef __NUMBER_MULTIPLIER_FACTORY_H_
#define __NUMBER_MULTIPLIER_FACTORY_H_

#include "ros_arithmetic/core/number_arithmetic_factory.h"
#include "ros_arithmetic/core/number_arithmetic_interface.h"

class NumberArithematicFactory {
 public:
  NumberArithematicFactory();
  ~NumberArithematicFactory();

  void CreateMultiplier(NumberArithematicInterface *);
  NumberArithematicInterface* GetMultiplier();

 private:
  NumberArithematicInterface *number_multiplier_;
};
#endif /* __NUMBER_MULTIPLIER_FACTORY_H_ */
