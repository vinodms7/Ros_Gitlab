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
#ifndef MULTIPLIER_NODE_H
#define MULTIPLIER_NODE_H

#include "ros_arithmetic/core/communication_factory.h"
#include "ros_arithmetic/core/number_arithmetic_interface.h"
#include "ros_arithmetic/core/number_arithmetic_factory.h"

class MultiplierNodeHandler {
public:
  MultiplierNodeHandler();
  ~MultiplierNodeHandler();

  CommFactory* GetCommunicationFactory();

  uint32_t GetResult(uint32_t, uint32_t);

private:
  void CreateMultiplierFactory();
  void CreateCommunicationFactory();

  NumberArithematicFactory* multiplier_factory_;
  CommFactory*   comm_controller_factory_;
};

#endif /*MULTIPLIER_NODE_H */
