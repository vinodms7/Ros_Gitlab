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
#include <string>

/*! Class Declarations */
template<class T, class RT>
class MultiplierNodeHandler : public ArithmeticNodeHandlerInterface<T, RT> {
 public:
  
  /**
  * Function name: Constructor
  * @brief Constructs MultiplierNodeHandler object
  **/
  explicit MultiplierNodeHandler(std::string operation_type = "MUL");
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
  * @param[in]  T
  *               Holds the first input value
  *
  * @param[in]  T
  *               Holds the second input value
  *
  * @return     RT
  *               Holds the return value after result is computed
  **/
  RT ProcessData(T, T);

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
  CommFactory<T, RT>* GetCommunicationFactory();

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
  NumberArithmeticFactory<T, RT>* GetArithmeticFactory();

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

  /*!  string variable for arithmetic operation type */
  std::string operation_type_;
  /*!  ptr to arithmetic factory */
  NumberArithmeticFactory<T, RT>* arithmetic_factory_;
  /*!  ptr to comm factory */
  CommFactory<T, RT>*   communication_factory_; 
};
  template class MultiplierNodeHandler<uint32_t, uint64_t>;
  template class MultiplierNodeHandler<float, double>;

#endif /*MULTIPLIER_NODE_H */

