/****************************************************************************
*     Copyright (C) 2017 by KPIT Technologies                     *
*                                                                           *
****************************************************************************/

/**
* @file      publish_subscribe.cc
*
* @author    Sujeyendra Tummala (Tummala.Sujeyendra@kpit.com)
*
* @date      18-Oct- 2017
*
* @brief     This files implements the communication interface class and
*            performs the below features
*              1) Assigns the generator node handler reference
*              2) Advertises the topic that is being published
*              3) Get the random number to be sent form the generator node
*                 handler
*              4) Sends message 
*              5) Receives message through subscribe
**/

/*! Include Files */
#include "ros_number_generator/app/publish_subscribe.h"
#include "ros_number_generator/core/generator_config.h"

/*! Class Declarations */
/**
* @brief Implements the constructor to the publishSubsribe class that 
*        assigns the received generator node handle reference 
*        and advertises the topic to be published
**/
template<class T>
PublishSubscribe<T>::PublishSubscribe(NodeHandlerInterface<T> *gen_node_handler) {
  if ( nullptr != gen_node_handler ) {
    generator_node_handler_ = gen_node_handler;

    ros::NodeHandle *handle = GeneratorConfig<T>::ConfigInstance().generator_handle_;

    rand_num_publisher_ = handle->advertise<ros_ran_num_msg::rand_num>("random_numbers", 100);

    float nDuration = (1.0/GeneratorConfig<T>::ConfigInstance().frequency_);

    timer_ = handle->createTimer(ros::Duration(nDuration), &GeneratorNodeHandler<T>::CommCallback);
  } else {
    ROS_WARN("No Generator Node instance. No messages to publish");
  }
}

/**
* @brief Implements the destructor to the PublishSubscribe class
**/
template<class T>
PublishSubscribe<T>::~PublishSubscribe() {
  GeneratorConfig<T>::ConfigInstance().generator_handle_->shutdown();
}

/**
* @brief Implements the functionality to get the generated 
*        random number and publish it
**/
template<class T>
void PublishSubscribe<T>::SendMessage(T value1, T value2) {

  ros_ran_num_msg::rand_num value;

  value.number1 = value1;
  value.number2 = value2;

  rand_num_publisher_.publish(value);

  ROS_INFO("Number Generator Value 1: [%u] and Value 2: [%u]",
                                    value.number1, value.number2);

  ros::spinOnce();
}

/**
* @brief Implements the functionality to receive the message
**/
template<class T>
void PublishSubscribe<T>::ReceiveMessage() {
}

