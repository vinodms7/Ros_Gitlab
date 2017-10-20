/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file		Multiplier Node Handler
* @author       Sasi Kiran <Sasi.Alur@kpit.com>
* @author       Rajat Jayanth Shetty  <rajat.shetty@kpit.com>	
* @author       Sujeyndra Tummala	<Tummala.Sujeyendra@kpit.com>
* @date         18 oct 2017
* @brief        Perform factory creation and processing data functionalities
*
*
**/

/* include files */
#include "ros_arithmetic/app/multiplier_node_handler.h"
#include "ros_arithmetic/app/publish_subscribe.h"
#include "ros_arithmetic/app/multiplier_arithmetic.h"

MultiplierNodeHandler::MultiplierNodeHandler() {
  CreateMultiplierFactory();
  CreateCommunicationFactory();
}

MultiplierNodeHandler::~MultiplierNodeHandler() {
  if(NULL != arithmetic_factory_){
    delete  arithmetic_factory_;
    arithmetic_factory_ = NULL;
  }

  if(NULL != communication_factory_){
    delete communication_factory_;
    communication_factory_ = NULL ;
  }
}

void MultiplierNodeHandler::CreateMultiplierFactory() {
  arithmetic_factory_ = new NumberArithmeticFactory();
  arithmetic_factory_->CreateArithmeticOperation(new NumberMultiplier());  
}

void MultiplierNodeHandler::CreateCommunicationFactory() {
  communication_factory_ = new CommFactory();
  communication_factory_->CreateCommunicator(new PublishSubscribe(this));
}

void MultiplierNodeHandler::Execute() {
   if( communication_factory_->GetCommunicator() != NULL )
      communication_factory_->GetCommunicator()->SendMessage();
}

uint32_t MultiplierNodeHandler::ProcessData(uint32_t value1, uint32_t value2) {
  if( arithmetic_factory_->GetArithmeticOperation() != NULL )
    return arithmetic_factory_->GetArithmeticOperation()->DoArithmeticOperation(value1, value2);
  return 0;
}
