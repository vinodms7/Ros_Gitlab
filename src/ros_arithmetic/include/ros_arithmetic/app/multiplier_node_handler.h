/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file		Multiplier Node Handler
* @author       Sasi Kiran <Sasi.Alur@kpit.com>
* @author       Rajat Jayanth Shetty  <rajat.shetty@kpit.com>	
* @author       Sujeyndra Tummala	<Tummala.Sujeyendra@kpit.com>
* @date         18 oct 2017
* @brief        Perform factory creation and processing data functionalities
*
*
**/

#ifndef MULTIPLIER_NODE_H
#define MULTIPLIER_NODE_H

/* include files */
#include "ros_arithmetic/core/communication_factory.h"
#include "ros_arithmetic/core/number_arithmetic_interface.h"
#include "ros_arithmetic/core/number_arithmetic_factory.h"

class MultiplierNodeHandler {
public:
  /**
  * Constructor
  **/
  MultiplierNodeHandler();
  /**
  * Destructor
  **/
  ~MultiplierNodeHandler();

  /**
  * Function name: GetCommunicationFactory
  *
  * @brief Get communication object
  *
  * @return	CommFactory* returns created communication object
  **/
  CommFactory* GetCommunicationFactory();

  /**
  * Function name: GetResult
  *
  * @brief process data and return value
  * 
  *
  * @param[in]	uint32_t  value1 This is first parameter
  *
  * @param[in]	uint32_t  value2 This is second parameter 
  *
  * @return	uint32_t  return value after result is computed
  **/
  uint32_t GetResult(uint32_t, uint32_t);

private:
  /**
  * Function name: CreateMultiplierFactory
  *
  * @brief Call arithmetic factory and create Multiplication operation object
  *
  * @return	void
  *
  **/
  void CreateMultiplierFactory();
  /**
  * Function name: CreateCommunicationFactory
  *
  * @brief Call communication factory and create communication object
  *
  * @return	void
  *
  **/
  void CreateCommunicationFactory();

  NumberArithematicFactory* multiplier_factory_; //  pointer to arithmetic factory
  CommFactory*   comm_controller_factory_; //  pointer to comm factory
};

#endif /*MULTIPLIER_NODE_H */
