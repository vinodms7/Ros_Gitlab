/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/

/**
* @file		Number Arithmetic Factory
* @author       Sasi Kiran	
* @date         18 oct 2017
* @brief        Factory class for creation of number arithmetic operations
*
*
**/
#ifndef NUMBER_ARITHMETIC_FACTORY_H_
#define NUMBER_ARITHMETIC_FACTORY_H_

/* include files */
#include "ros_arithmetic/core/number_arithmetic_factory.h"
#include "ros_arithmetic/core/number_arithmetic_interface.h"

class NumberArithmeticFactory {
 public:
  /**
  * Constructor
  * @brief Construct NumberArithmeticFactory object
  **/
  NumberArithmeticFactory();
  /**
  * Destructor
  * @brief Destruct NumberArithmeticFactory object
  **/
  ~NumberArithmeticFactory();

  /**
  * Function name: CreateArithmeticOperation
  *
  * @brief Create number arithmetic operation object
  *
  * @param[in]	NumberArithmeticInterface*  multiplier_int 
  *             This is pointer to interface  
  *
  * @return	void
  **/
  void CreateArithmeticOperation(NumberArithmeticInterface *multiplier_int);
  /**
  * Function name: GetArithmeticOperation
  *
  * @brief Get Created number arithmetic operation object   
  *
  * @return	NumberArithmeticInterface* 
  *             returns created object to base class pointer 
  **/
  NumberArithmeticInterface* GetArithmeticOperation();

 private:
  NumberArithmeticInterface *number_arithmetic_; //  Pointer to Interface
};
#endif /* __NUMBER_MULTIPLIER_FACTORY_H_ */
