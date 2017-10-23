/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/

/**
* @file        number_arithmetic_factory.h
*
* @author      Sasi Kiran  
*
* @date        18-Oct-2017
*
* @brief       Factory class for creation of number arithmetic operations
*
**/
#ifndef NUMBER_ARITHMETIC_FACTORY_H_
#define NUMBER_ARITHMETIC_FACTORY_H_

/*! Include files */
#include "ros_arithmetic/core/number_arithmetic_factory.h"
#include "ros_arithmetic/core/number_arithmetic_interface.h"

/*! Class Declarations */
template<class T, class RT>
class NumberArithmeticFactory {
 public:
  /**
  * Function name: Constructor
  * @brief Construct NumberArithmeticFactory object
  **/
  NumberArithmeticFactory();

  /**
  * Function name: Destructor
  * @brief Destruct NumberArithmeticFactory object
  **/
  ~NumberArithmeticFactory();

  /**
  * Function name: CreateArithmeticOperation
  *
  * @brief Create number arithmetic operation object
  *
  * @param[in]  NumberArithmeticInterface*
  *             Holds the pointer to arithmetic interface object  
  *
  * @return  void
  **/
  void CreateArithmeticOperation(NumberArithmeticInterface<T, RT>* numb_int);

  /**
  * Function name: GetArithmeticOperation
  *
  * @brief Create number arithmetic operation object
  *
  * @return    NumberArithmeticInterface*
  *             Returns pointer to NumberArithmeticInterface*
  **/
  NumberArithmeticInterface<T, RT>* GetArithmeticOperation();

  /**
  * Function name: ExecuteArithmeticOperation
  *
  * @brief Execute arithmetic operation and return value
  *
  * @param[in]  T
  *                Holds the first input value for operation
  *
  * @param[in]  T
  *                Holds the second input value for operation
  *
  * @return     RT  
  *                Holds the return result
  **/
  RT ExecuteArithmeticOperation(T num1, T num2);

 private:
/*! Arithmetic Interface Ptr */
  NumberArithmeticInterface<T, RT>* number_arithmetic_;
};
  template class NumberArithmeticFactory<uint32_t, uint64_t>;
  template class NumberArithmeticFactory<float, double>;

#endif /* __NUMBER_MULTIPLIER_FACTORY_H_ */

