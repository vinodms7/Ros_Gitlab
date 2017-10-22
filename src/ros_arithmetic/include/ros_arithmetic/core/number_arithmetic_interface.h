/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/

/**
* @file    number_artihmetic_interface.h
*
* @author       Sasi Kiran <Sasi.Alur@kpit.com>  
*
* @date         18-Oct-2017
*
* @brief        Interface class for Number Arithmetic Operations
**/

#ifndef NUMBER_MULTIPLIER_H_
#define NUMBER_MULTIPLIER_H_

/*! Include files */
#include <ros/types.h>

/*! Class declarations */
class NumberArithmeticInterface {
 public:
  /**
  * Function name: DoArithmeticOperation
  *
  * @brief Perform number arithmetic operation
  *
  * @param[in]  uint32_t value1 this is first number for arithmetic operation
  *
  * @param[in]  uint32_t value2 this is second number for arithmetic operation
  *
  * @return  uint32_t retrun value after arithmetic opeation is done 
  **/
  virtual uint32_t DoArithmeticOperation(uint32_t, uint32_t) = 0;

 protected:
  /**
  * Function name: DisplayResult
  *
  * @brief Display result
  *
  * @param[in]  uint32_t 
  *                Holds the value to be displayed
  *
  * @return  void 
  **/
  virtual void DisplayResult(uint32_t) = 0;
};

#endif /* NUMBER_MULTIPLIER_H_ */

