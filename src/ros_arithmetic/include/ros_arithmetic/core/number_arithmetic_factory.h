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
  void CreateArithmeticOperation(NumberArithmeticInterface * numb_interface);

  /**
  * Function name: GetArithmeticOperation
  *
  * @brief Create number arithmetic operation object
  *
  * @return    NumberArithmeticInterface*
  *             Returns pointer to NumberArithmeticInterface*
  **/
  NumberArithmeticInterface* GetArithmeticOperation();

  /**
  * Function name: ExecuteArithmeticOperation
  *
  * @brief Execute arithmetic operation and return value
  *
  * @param[in]  uint32_t
  *                Holds the first input value for operation
  *
  * @param[in]  uint32_t
  *                Holds the second input value for operation
  *
  * @return     uint32_t  
  *                Holds the return result
  **/
  uint32_t ExecuteArithmeticOperation(uint32_t num1, uint32_t num2);

 private:
  NumberArithmeticInterface *number_arithmetic_;  // Arithmetic Interface Ptr
};
#endif /* __NUMBER_MULTIPLIER_FACTORY_H_ */

