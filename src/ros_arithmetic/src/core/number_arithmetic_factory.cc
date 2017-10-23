/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/

/**
* @file       Number Arithmetic Factory
*
* @author     Sasi Kiran <Sasi.Alur@kpit.com>
*
* @date       18-Oct-2017
*
* @brief      Implementation class for factory
**/

/*! Include files */
#include <cstddef>
#include "ros_arithmetic/core/number_arithmetic_factory.h"

/*! Class Definitions */
template<class T, class RT>
NumberArithmeticFactory<T, RT>::NumberArithmeticFactory() {
  number_arithmetic_ = nullptr;
}

template<class T, class RT>
NumberArithmeticFactory<T, RT>::~NumberArithmeticFactory() {
  if ( nullptr != number_arithmetic_ ) {
    delete number_arithmetic_;
    number_arithmetic_ = nullptr;
  }
}

/**
* @brief Implements the creation of arithmetic operation object
**/
template<class T, class RT>
void NumberArithmeticFactory<T, RT>::CreateArithmeticOperation(
                      NumberArithmeticInterface<T, RT> *number_arithmetic) {
  if ( nullptr != number_arithmetic_ ) {
    delete number_arithmetic_;
    number_arithmetic_ = nullptr;
  }
  number_arithmetic_ = number_arithmetic;
}

/**
* @brief Implements returning pointer to NumberArithmeticInterface
**/
template<class T, class RT>
NumberArithmeticInterface<T, RT>* NumberArithmeticFactory<T, RT>::
                                  GetArithmeticOperation() {
  return number_arithmetic_;
}

/**
* @brief Implements arithmetic operation Execution
**/
template<class T, class RT>
RT NumberArithmeticFactory<T, RT>::ExecuteArithmeticOperation(T value1,
                                                            T value2) {
  RT number_arithmetic;

  if ( nullptr != number_arithmetic_ )
    number_arithmetic = number_arithmetic_->DoArithmeticOperation(value1,
                                                                 value2);
  else
    number_arithmetic = 0;

  return number_arithmetic;
}

