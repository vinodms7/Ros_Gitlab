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
#ifndef __NUMBER_MULTIPLIER_H_
#define __NUMBER_MULTIPLIER_H_

#include <ros/types.h>

class NumberArithematicInterface {
 public:
  NumberArithematicInterface();
  virtual ~NumberArithematicInterface();

  virtual uint32_t DoArithematicOperation(uint32_t, uint32_t) = 0;

 protected:
  virtual void PrintValue();

};

#endif
