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

#include "ros_number_generator/core/communication_factory.h"

CommFactory::CommFactory() {
  communication_interface_ = NULL;
}

CommFactory::~CommFactory() {
  if (NULL != communication_interface_) {
    delete communication_interface_;
    communication_interface_ = NULL;
  }
}
void CommFactory::CreateCommunicator(CommunicationInterface *pCommController) {
  if (NULL != communication_interface_) {
    delete communication_interface_;
    communication_interface_ = NULL;
  }
  communication_interface_ = pCommController; 
}
CommunicationInterface * CommFactory::GetCommunicator() const {
  return communication_interface_;
}
