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

/*! Class Declarations */
/**
* @brief Implements the constructor to the publishSubsribe class that 
*        assigns the received generator node handle reference 
*        and advertises the topic to be published
**/
PublishSubscribe::PublishSubscribe(NodeHandlerInterface *gen_node_handler) {
  if (nullptr != gen_node_handler) {
    generator_node_handler_ = gen_node_handler;
    rand_num_publisher_ = node_handle_.advertise<ros_ran_num_msg::rand_num>
                                               ("random_numbers", 100);
  } else {
    ROS_WARN("No Generator Node instance. No messages to publish");
  }
}

/**
* @brief Implements the destructor to the PublishSubscribe class
**/
PublishSubscribe::~PublishSubscribe() {
  node_handle_.shutdown();
}

/**
* @brief Implements the functionality to get the generated 
*        random number and publish it
**/
void PublishSubscribe::SendMessage() {
  if (nullptr != generator_node_handler_) {
    float frequency = 0.1;
    if ( !node_handle_.getParam("frequency", frequency) ) {
      ROS_WARN("Frequency Param not received...");
    } else {
      ROS_INFO("Frequency Param is [%f]", frequency);
    }

    ros::Rate rate(frequency);
    ros_ran_num_msg::rand_num value;

    while ( node_handle_.ok() ) {
      value.number1 = generator_node_handler_->GetNumber();
      value.number2 = generator_node_handler_->GetNumber();

      rand_num_publisher_.publish(value);

      ROS_INFO("Number Generator Value 1: [%u] and Value 2: [%u]",
                                    value.number1, value.number2);

      ros::spinOnce();
      rate.sleep();
    }
  } else {
    ROS_WARN("No Generator Node instance. No messages to publish");
  }
}

/**
* @brief Implements the functionality to receive the message
**/
void PublishSubscribe::ReceiveMessage() {
}
