/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file        multiplier_arithmetic.cc
*
* @author       Sasi Kiran <Sasi.Alur@kpit.com>
*
* @date         18-Oct-2017
*
* @brief        Implementation class for multiplication functionality
**/

/*! Include files */
#include "ros_arithmetic/app/multiplier_arithmetic.h"

/*! Class Defintions */
template<class T, class RT>
NumberMultiplier<T, RT>::NumberMultiplier(): multiplier_value_(0) {
}

template<class T, class RT>
NumberMultiplier<T, RT>::~NumberMultiplier() {
}

template<class T, class RT>
RT NumberMultiplier<T, RT>::DoArithmeticOperation(T value1,
                                                  T value2) {
  return DoMultiplication(value1, value2);
}

/**
* @brief Implements the functionality to perform multiplication 
* using * operator
**/
template<class T, class RT>
RT NumberMultiplier<T, RT>::DoMultiplication(T value1, T value2) {
  multiplier_value_ = (value1)*(value2);
  return multiplier_value_;
}

