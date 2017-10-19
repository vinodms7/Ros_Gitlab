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
#ifndef COMMUNICATION_FACTORY_H
#define COMMUNICATION_FACTORY_H

#include "ros_number_generator/core/communication_interface.h"

class CommFactory {
 public:
  CommFactory();
  ~CommFactory();
  void CreateCommunicator(CommunicationInterface* comm_int);
  CommunicationInterface* GetCommunicator() const;
 private:
  CommunicationInterface* communication_interface_;
};
#endif
