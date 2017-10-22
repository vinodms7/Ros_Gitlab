/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file        multiplier_node_handler.cc
* 
* @author      Sasi Kiran <Sasi.Alur@kpit.com>
* @author      Rajat Jayanth Shetty  <rajat.shetty@kpit.com>  
* @author      Sujeyndra Tummala  <Tummala.Sujeyendra@kpit.com>
*
* @date        18-Oct-2017
*
* @brief       Perform factory creation and processing data functionalities
**/

/*! include files */
#include <ros/ros.h>

#include "ros_arithmetic/app/multiplier_node_handler.h"
#include "ros_arithmetic/app/publish_subscribe.h"
#include "ros_arithmetic/app/multiplier_arithmetic.h"

/*! Class Definitions */
MultiplierNodeHandler::MultiplierNodeHandler(OperationType operation_type)
                                             :operation_type_(operation_type),
                                             arithmetic_factory_(nullptr),
                                             communication_factory_(nullptr) {
  CreateMultiplierFactory();
  CreateCommunicationFactory();
}

MultiplierNodeHandler::~MultiplierNodeHandler() {
  if (nullptr != arithmetic_factory_) {
    delete  arithmetic_factory_;
    arithmetic_factory_ = nullptr;
  }

  if (nullptr != communication_factory_) {
    delete communication_factory_;
    communication_factory_ = nullptr;
  }
}

/**
* @brief Create function for swithcing between arithmetic operation object creation     
**/
void MultiplierNodeHandler::CreateMultiplierFactory() {
  arithmetic_factory_ = new NumberArithmeticFactory();
  if (nullptr != arithmetic_factory_) {
    if (operation_type_ == OperationType::MUL) {
      arithmetic_factory_->CreateArithmeticOperation(new NumberMultiplier());
    } else {
      ROS_WARN("No object Created, Invalid type");
    }
  } else {
    ROS_WARN("Arithmetic Factory object not Created");
}
}

void MultiplierNodeHandler::CreateCommunicationFactory() {
  communication_factory_ = new CommFactory();
  if (nullptr != communication_factory_) {
    communication_factory_->CreateCommunicator(new PublishSubscribe(this));
  } else {
    ROS_WARN("Communication Factory object not Created");
  }
}

CommFactory* MultiplierNodeHandler::GetCommunicationFactory() {
  return communication_factory_;
}

NumberArithmeticFactory* MultiplierNodeHandler::GetArithmeticFactory() {
  return arithmetic_factory_;
}

void MultiplierNodeHandler::Execute() {
  if ( nullptr != communication_factory_ )
    communication_factory_->ExecuteCommunication();
  else
    ROS_WARN("Communication could not be established");
}

/**
* @brief Executes arithmetic operation execution and returns valid value
*        Returns value 0 when arithmetic operation is not executed    
**/
uint32_t MultiplierNodeHandler::ProcessData(uint32_t value1,
                                            uint32_t value2) {
  uint32_t arithmetic_number;
  if (nullptr != arithmetic_factory_) {
    arithmetic_number =
      arithmetic_factory_->ExecuteArithmeticOperation(value1, value2);
  } else {
    arithmetic_number = 0;
  }
  return arithmetic_number;
}

