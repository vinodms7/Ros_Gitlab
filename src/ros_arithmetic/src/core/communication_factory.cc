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
#include <cstddef>

#include "ros_arithmetic/core/communication_factory.h"

CommFactory::CommFactory() {
  communication_interface_ = NULL;
}

CommFactory::~CommFactory() {
  if (NULL != communication_interface_) {
    delete communication_interface_;
    communication_interface_ = NULL;
  }
}

void CommFactory::CreateCommunicator(CommunicationInterface *comm_int) {
  if (NULL != communication_interface_) {
    delete communication_interface_;
    communication_interface_ = NULL;
  }  
  communication_interface_ = comm_int;
}

CommunicationInterface * CommFactory::GetCommunicator() const {
  return communication_interface_;
}
