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
template<class T, class RT>
class NumberMultiplier : public NumberArithmeticInterface<T, RT> {
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
  * @param[in]  T  value1 
  *               Holds first parameter for arithmetic operation
  *
  * @param[in]  T  value2 
  *               This is second parameter for arithmetic operation
  *
  * @return     RT  
  *               Holds the return value after arithmetic operation is done
  **/
  RT DoArithmeticOperation(T value1, T value2);

 private:
  /**
  * Function name: DoMultiplication
  *
  * @brief Perfom multiplication operation
  * 
  *
  * @param[in]  T  value1
  *               Holds the first parameter for multiplication
  *
  * @param[in]  T  value2
  *               Holds the second parameter for multiplication
  *
  * @return     RT
  *               Holds the return value after multiplication is done
  **/
  RT DoMultiplication(T value1, T value2);

  /*!  Multiplier value member variable */
  RT multiplier_value_;
};
  template class NumberMultiplier<uint32_t,uint64_t>;
  template class NumberMultiplier<float, double>;

#endif /* __MULTIPLIER_ARITHMETIC_H_ */

