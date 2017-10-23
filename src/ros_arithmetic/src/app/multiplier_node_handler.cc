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
template<class T, class RT>
MultiplierNodeHandler<T, RT>::MultiplierNodeHandler(std::string operation_type)
                                             :operation_type_(operation_type),
                                             arithmetic_factory_(nullptr),
                                             communication_factory_(nullptr) {
  CreateMultiplierFactory();
  CreateCommunicationFactory();
}

template<class T, class RT>
MultiplierNodeHandler<T, RT>::~MultiplierNodeHandler() {
  if ( nullptr != arithmetic_factory_ ) {
    delete  arithmetic_factory_;
    arithmetic_factory_ = nullptr;
  }

  if ( nullptr != communication_factory_ ) {
    delete communication_factory_;
    communication_factory_ = nullptr;
  }
}

/**
* @brief Create function for swithcing between arithmetic operation object creation     
**/
template<class T, class RT>
void MultiplierNodeHandler<T, RT>::CreateMultiplierFactory() {
  arithmetic_factory_ = new NumberArithmeticFactory<T, RT>();
  if ( nullptr != arithmetic_factory_ ) {
    if (operation_type_ == "MUL") {
      arithmetic_factory_->CreateArithmeticOperation(
                                    new NumberMultiplier<T, RT>());
    } else {
      ROS_WARN("No object Created, Invalid type");
    }
  } else {
    ROS_WARN("Arithmetic Factory object not Created");
  }
}

template<class T, class RT>
void MultiplierNodeHandler<T, RT>::CreateCommunicationFactory() {
  communication_factory_ = new CommFactory<T, RT>();
  if ( nullptr != communication_factory_ ) {
    communication_factory_->CreateCommunicator(
                               new PublishSubscribe<T, RT>(this));
  } else {
    ROS_WARN("Communication Factory object not Created");
  }
}

template<class T, class RT>
CommFactory<T, RT>* MultiplierNodeHandler<T, RT>::GetCommunicationFactory() {
  return communication_factory_;
}

template<class T, class RT>
NumberArithmeticFactory<T, RT>* MultiplierNodeHandler<T, RT>::
                                GetArithmeticFactory() {
  return arithmetic_factory_;
}

template<class T, class RT>
void MultiplierNodeHandler<T, RT>::Execute() {
  if ( nullptr != communication_factory_ )
    communication_factory_->ExecuteCommunication();
  else
    ROS_WARN("Communication could not be established");
}

/**
* @brief Executes arithmetic operation execution and returns valid value
*        Returns value 0 when arithmetic operation is not executed    
**/
template<class T, class RT>
RT MultiplierNodeHandler<T, RT>::ProcessData(T value1,
                                      T value2) {
  RT arithmetic_number;
  if ( nullptr != arithmetic_factory_ ) {
    arithmetic_number =
      arithmetic_factory_->ExecuteArithmeticOperation(value1, value2);
  } else {
    arithmetic_number = 0;
  }
  return arithmetic_number;
}

