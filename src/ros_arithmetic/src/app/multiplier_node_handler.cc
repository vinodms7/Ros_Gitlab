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
#include "ros_arithmetic/app/multiplier_node_handler.h"
#include "ros_arithmetic/app/publish_subscribe.h"
#include "ros_arithmetic/app/multiplier_arithmetic.h"

MultiplierNodeHandler::MultiplierNodeHandler() {
  CreateMultiplierFactory();
  CreateCommunicationFactory();
}

MultiplierNodeHandler::~MultiplierNodeHandler() {
  if(NULL != multiplier_factory_){
    delete  multiplier_factory_;
    multiplier_factory_ = NULL;
  }

  if(NULL != comm_controller_factory_){
    delete comm_controller_factory_;
    comm_controller_factory_ = NULL ;
  }
}

void MultiplierNodeHandler::CreateMultiplierFactory() {
  multiplier_factory_ = new NumberArithematicFactory();
  multiplier_factory_->CreateMultiplier(new NumberMultiplier());  
}

void MultiplierNodeHandler::CreateCommunicationFactory() {
  comm_controller_factory_ = new CommFactory();
  comm_controller_factory_->CreateCommunicator(new PublishSubscribe(this));
}

CommFactory* MultiplierNodeHandler::GetCommunicationFactory() {
  return comm_controller_factory_;
}

uint32_t MultiplierNodeHandler::GetResult(uint32_t value1, uint32_t value2) {  

 return multiplier_factory_->GetMultiplier()->DoArithematicOperation(value1, value2);
}
