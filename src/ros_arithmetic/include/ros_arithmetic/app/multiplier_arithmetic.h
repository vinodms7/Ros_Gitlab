/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/

/**
* @file         multiplier_arithmetic.h
*
* @author       Sasi Kiran  <Sasi.Alur@kpit.com>
*
* @date         18-Oct-2017
*
* @brief        Perform multiplication functionality
*
**/
#ifndef MULTIPLIER_ARITHMETIC_H_
#define MULTIPLIER_ARITHMETIC_H_

/*! Include Files */
#include "ros_arithmetic/core/number_arithmetic_interface.h"

/*! Class Definitions */
class NumberMultiplier : public NumberArithmeticInterface {
 public:
  /**
  * Function name: Constructor
  * @brief Construct number arithmetic object
  **/
  NumberMultiplier();

  /**
  * Function name: Destructor
  * @brief Destruct number arithmetic object
  **/
  ~NumberMultiplier();

  /**
  * Function name: DoArithmeticOperation
  *
  * @brief Perfom arithmetic operation
  * 
  *
  * @param[in]  uint32_t  value1 
  *               Holds first parameter for arithmetic operation
  *
  * @param[in]  uint32_t  value2 
  *               This is second parameter for arithmetic operation
  *
  * @return     uint32_t  
  *               Holds the return value after arithmetic operation is done
  **/
  uint32_t DoArithmeticOperation(uint32_t value1, uint32_t value2);

  /**
  * Function name: DisplayResult
  *
  * @brief Display result
  *
  * @param[in]  uint32_t value
  *               Holds the value of the result to be displayed
  *
  * @return     void 
  **/
  void DisplayResult(uint32_t value);

 private:
  /**
  * Function name: DoMultiplication
  *
  * @brief Perfom multiplication operation
  * 
  *
  * @param[in]  uint32_t  value1
  *               Holds the first parameter for multiplication
  *
  * @param[in]  uint32_t  value2
  *               Holds the second parameter for multiplication
  *
  * @return     uint32_t
  *               Holds the return value after multiplication is done
  **/
  uint32_t DoMultiplication(uint32_t value1, uint32_t value2);

  /*!  Multiplier value member variable */
  uint32_t multiplier_value_;
};

#endif /* __MULTIPLIER_ARITHMETIC_H_ */

