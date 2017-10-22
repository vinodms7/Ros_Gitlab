/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file         multiplier_node_handler.h
*
* @author       Sasi Kiran <Sasi.Alur@kpit.com>
* @author       Rajat Jayanth Shetty  <rajat.shetty@kpit.com>  
* @author       Sujeyndra Tummala  <Tummala.Sujeyendra@kpit.com>
*
* @date         18-Oct-2017
*
* @brief        Perform factory creation and processing data functionalities
**/

#ifndef MULTIPLIER_NODE_HANDLER_H
#define MULTIPLIER_NODE_HANDLER_H

/*! Include files */
#include "ros_arithmetic/core/arithmetic_node_handler_interface.h"
#include "ros_arithmetic/core/communication_factory.h"
#include "ros_arithmetic/core/number_arithmetic_interface.h"
#include "ros_arithmetic/core/number_arithmetic_factory.h"

/*! Class Declarations */
class MultiplierNodeHandler : public ArithmeticNodeHandlerInterface {
 public:
  /*! Enumeration type for arithmetic operation */
  enum OperationType {
    NONE = 0,
    MUL,
    ADD,
    SUB
  };

  /**
  * Function name: Constructor
  * @brief Constructs MultiplierNodeHandler object
  **/
  explicit MultiplierNodeHandler(OperationType operation_type =
                                              OperationType::MUL);
  /**
  * Function name: Destructor
  * @brief Destructs MultiplierNodeHandler object
  **/
  ~MultiplierNodeHandler();

  /**
  * Function name: Execute
  *
  * @brief Execute the multiplier and communication functionalities
  *
  * @return  void
  **/
  void Execute();

  /**
  * Function name: ProcessData
  *
  * @brief process data and return value
  * 
  *
  * @param[in]  uint32_t
  *               Holds the first input value
  *
  * @param[in]  uint32_t
  *               Holds the second input value
  *
  * @return     uint32_t
  *               Holds the return value after result is computed
  **/
  uint32_t ProcessData(uint32_t, uint32_t);

  /**
  * Function name: GetCommunicationFactory
  *
  * @brief Gets the communication factory object
  * 
  * @param[in]  None
  *
  * @return     CommFactory
  *               Holds the reference to communication factory object
  **/
  CommFactory* GetCommunicationFactory();

  /**
  * Function name: GetArithmeticFactory
  *
  * @brief Gets the arithmetic factory object
  * 
  * @param[in]  None
  *
  * @return     NumberArithmeticFactory
  *               Holds the reference to communication factory object
  **/
  NumberArithmeticFactory* GetArithmeticFactory();

 private:
  /**
  * Function name: CreateMultiplierFactory
  *
  * @brief Calls arithmetic factory and create Multiplication operation object
  * 
  * @param[in]  None
  *
  * @return     void
  *
  **/
  void CreateMultiplierFactory();

  /**
  * Function name: CreateCommunicationFactory
  *
  * @brief Call communication factory and create communication object
  * 
  * @param[in]  None
  *
  * @return     void
  *
  **/
  void CreateCommunicationFactory();

  OperationType operation_type_;  // enum  type for arithmetic operation
  NumberArithmeticFactory* arithmetic_factory_;  //  ptr to arithmetic factory
  CommFactory*   communication_factory_;  //  ptr to comm factory
};

#endif /*MULTIPLIER_NODE_H */

