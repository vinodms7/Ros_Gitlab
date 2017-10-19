/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/

/**
* @file
* @author
* @date
* @brief
*
*
*/
#ifndef GENERATOR_NODE_H
#define GENERATOR_NODE_H

#include "ros_number_generator/core/communication_factory.h"
#include "ros_number_generator/core/number_generator_factory.h"
#include "ros_number_generator/app/number_generator_srand.h"
#include "ros_number_generator/app/number_generator_lcg.h"

class GeneratorNodeHandler {
 public:
  GeneratorNodeHandler();
  ~GeneratorNodeHandler();
  uint32_t GetNumber();
  CommFactory* GetCommunicationFactory();
 
 private:
  void CreateNumberFactory();
  void CreateCommunicationFactory();

  NumberGeneratorFactory* number_generator_;
  CommFactory*   comm_controller_factory_;
};
#endif /* GENERATOR_NODE_H */
