/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file         generator_node_handler.h
* 
* @author       Rajat Jayanth Shetty <Rajat.Shetty@kpit.com>
* 
* @author       Sujeyendra Tummala <Tummala.Sujeyendra@kpit.com>
* 
* @author       Sasi Kiran Alur  <Sasi.Alur@kpit.com> 
* 
* @date         18 Oct 2017
* 
* @brief        Perform factory creation and processing data functionalities
*
**/
#ifndef GENERATOR_NODE_HANDLER_H
#define GENERATOR_NODE_HANDLER_H

/*! Include files  */
#include <ros/ros.h>

#include "ros_number_generator/core/generator_ node_handler_interface.h"
#include "ros_number_generator/core/communication_factory.h"
#include "ros_number_generator/core/number_generator_factory.h"
#include "ros_number_generator/app/number_generator_srand.h"
#include "ros_number_generator/app/number_generator_lcg.h"

template<class T>
class GeneratorNodeHandler : public NodeHandlerInterface<T> {
 public:
  /**
  * Function name: GeneratorNodeHandler()
  *
  * @brief Constructor for the Node Handler
  *
  * @param[in]  None
  **/
  explicit GeneratorNodeHandler();

  /**
  * Function name: ~GeneratorNodeHandler()
  * 
  * @brief Destructor for the Node Handler
  **/
  ~GeneratorNodeHandler();

  /**
  * Function name: GetNumber
  *
  * @brief Get random value generated using the generator Factory Node
  * 
  * @return  DataType  return value of result
  **/
  T GetNumber();

  /**
  * Function name: Execute
  *
  * @brief Execute the Generator and communication functionalities
  *
  * @return  void
  **/
  void Execute();

  /**
  * Function name: TimerCallback
  *
  * @brief Call back to the execute communication
  *
  * @param[in]  ros:TimerEvent
  *                Holds the callback event from timer
  *
  * @return  void
  **/
  static void CommCallback(const ros::TimerEvent& evt);

  /**
  * Function name: GetCommunicationFactory
  *
  * @brief Get pointer to communication object by factory 
  *
  * @return CommFactory<T>
  *            Holds the reference to CommFactory of type T   
  **/
  CommFactory<T>* GetCommunicationFactory();

  /**
  * Function name: GetNumberFactory
  *
  * @brief Get the pointer to the generator Factory Node
  *
  * @return NumberGeneratorFactory<T>
  *            Holds the reference to NumberGeneratorFactory of type T 
  **/
  NumberGeneratorFactory<T>* GetNumberFactory();

 private:
  /**
  * Function name: CreateNumberFactory
  *
  * @brief Call number generator factory and Create generator object
  * 
  * @return  void
  **/
  void CreateNumberFactory();

  /**
  * Function name: CreateCommunicationFactory
  *
  * @brief Call communication factory and Create communication object
  * 
  * @return  void
  **/
  void CreateCommunicationFactory();

  std::string generator_type_;  /*! string variable for generator type */
  NumberGeneratorFactory<T>* number_generator_factory_;  /* Pointer to generator factory */
  CommFactory<T>* communication_factory_;   /* Pointer to communication factory */
};

  template class GeneratorNodeHandler<uint32_t>;
  template class GeneratorNodeHandler<float>;

#endif /* GENERATOR_NODE_HANDLER_H */

