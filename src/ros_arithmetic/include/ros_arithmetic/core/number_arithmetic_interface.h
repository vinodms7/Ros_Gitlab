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
template<class T, class RT>
class NumberArithmeticInterface {
 public:
  /**
  * Function name: DoArithmeticOperation
  *
  * @brief Perform number arithmetic operation
  *
  * @param[in]  T value1 this is first number for arithmetic operation
  *
  * @param[in]  T value2 this is second number for arithmetic operation
  *
  * @return  RT retrun value after arithmetic opeation is done 
  **/
  virtual RT DoArithmeticOperation(T, T) = 0;
};

#endif /* NUMBER_MULTIPLIER_H_ */

