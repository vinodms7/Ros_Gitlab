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
#ifndef __NUMBER_MULTIPLIER_FACTORY_H_
#define __NUMBER_MULTIPLIER_FACTORY_H_

/* include files */
#include "ros_arithmetic/core/number_arithmetic_factory.h"
#include "ros_arithmetic/core/number_arithmetic_interface.h"

class NumberArithematicFactory {
 public:
  /**
  * Constructor
  **/
  NumberArithematicFactory();
  /**
  * Destructor
  **/
  ~NumberArithematicFactory();

  /**
  * Function name: CreateMultiplier
  *
  * @brief Create number arithmetic operation object
  *
  * @param[in]	NumberArithmeticInterface*  multiplier_int 
  *             This is pointer to interface  
  *
  * @return	void
  **/
  void CreateMultiplier(NumberArithematicInterface *multiplier_int);
  /**
  * Function name: GetMultiplier
  *
  * @brief Get Created number arithmetic operation object   
  *
  * @return	NumberArithmeticInterface* 
  *             returns created object to base class pointer 
  **/
  NumberArithematicInterface* GetMultiplier();

 private:
  NumberArithematicInterface *number_multiplier_; //  Pointer to Interface
};
#endif /* __NUMBER_MULTIPLIER_FACTORY_H_ */
