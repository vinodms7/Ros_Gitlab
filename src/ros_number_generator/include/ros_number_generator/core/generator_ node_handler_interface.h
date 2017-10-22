/****************************************************************************
*     Copyright (C) 2017 by KPIT Technologies                     *
*                                                                           *
****************************************************************************/

/**
* @file      generator_Node_Handler_ Interface.h
*
* @author    Sasi Kiran
*
* @date      18-Oct- 2017
*
* @brief     This is a Node Handler Interface class 
*
**/

#ifndef GENERATOR_NODE_HANDLER_INTERFACE_H
#define GENERATOR_NODE_HANDLER_INTERFACE_H
#include <string>

#include "ros_number_generator/core/communication_factory.h"
#include "ros_number_generator/core/number_generator_factory.h"

/* Class Declarations */

class NodeHandlerInterface {
 public:
  /**
  * Function name: GetNumber
  *
  * @brief Get number generated by the generator
  * 
  * @return  uint32_t  return value of result
  **/
  virtual uint32_t GetNumber() = 0;

  /**
  * Function name: Execute
  *
  * @brief Execute the internal functionality of Generator
  *
  * @return  void
  **/
  virtual void Execute() = 0;
};
#endif
