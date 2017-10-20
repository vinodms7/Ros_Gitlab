/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/

/**
* @file		Multiplier Node Handler
* @author       Sasi Kiran	
* @date         18 oct 2017
* @brief        Perform multiplication functionality
*
*
**/
#ifndef __MULTIPLIER_ARITHMETIC_H_
#define __MULTIPLIER_ARITHMETIC_H_

#include "ros_arithmetic/core/number_arithmetic_interface.h"

class NumberMultiplier : public NumberArithematicInterface {
 public:
  /**
  * Constructor
  **/
  NumberMultiplier();
  /**
  * Destructor
  **/
  ~NumberMultiplier() {}
  /**
  * Function name: DoArithematicOperation
  *
  * @brief Perfom arithmetic operation
  * 
  *
  * @param[in]	uint32_t  value1 This is first parameter for arithmetic operation
  *
  * @param[in]	uint32_t  value2 This is second parameter for arithmetic operation
  *
  * @return	uint32_t  return value after operation is done
  **/
  uint32_t DoArithematicOperation(uint32_t value1,
                                  uint32_t value2);
 /**
  * Function name: PrintValue
  *
  * @brief Show result
  * 
  * @return void
  *
  **/
 void PrintValue();

 private:
  uint32_t multiplier_value_; //  multiplier value
};

#endif /* __MULTIPLIER_ARITHMETIC_H_ */
