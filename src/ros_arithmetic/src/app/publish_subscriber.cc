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
*              1) Create a receiver callback object and assigs it with the 
*                 node handler reference
*              2) Gets the multiplier result for the received value
*              3) Sends message 
*              4) Receives message through subscribe
**/

/*! Include Files */
#include "ros_arithmetic/app/publish_subscribe.h"

/*! Class Declarations */
/**
* @brief Implements the constructor to the receiver callback class that assigns
*        the multiplier node handle reference to the received reference
**/
ReceiverCallback::ReceiverCallback(ArithmeticNodeHandlerInterface*
                                                    node_handler) {
  arithmetic_node_handler_ = node_handler;

  multiplier_publisher_ =
     node_handle_multiplier_.advertise<ros_ran_num_msg::mutliplier_num>("multiplier_output", 10);
}

/**
* @brief Implements the destructor to the receiver callback class
**/
ReceiverCallback::~ReceiverCallback() {
}

/**
* @brief Implements the callback function that get the multiplier of received values         
**/
void ReceiverCallback::MultiplierCallback(
                        const ros_ran_num_msg::rand_num::ConstPtr& value) {
  if ( nullptr != arithmetic_node_handler_ ) {
    uint32_t vresult = arithmetic_node_handler_->ProcessData(value->number1,
                                                             value->number2);
    ROS_INFO("The Multiplier value is [%u]", vresult);

    ros_ran_num_msg::mutliplier_num output;

    output.multiplier_value = vresult;

    multiplier_publisher_.publish(output);

    ros::spinOnce();
  } else {
    ROS_WARN("Invalid handler in Receiver Callback");
  }
}

/*! Class Declarations */
/**
* @brief Implements the constructor to the publishSubsribe class that creates an object
*        of receiver callback class and assigns the received multiplier node handle
*        reference to reference callback class
**/
PublishSubscribe::PublishSubscribe(
                  ArithmeticNodeHandlerInterface *multiplier_node_handler) {
  arithmetic_node_handler_ = multiplier_node_handler;

  receiver_callback_ = new ReceiverCallback(multiplier_node_handler);
}

/**
* @brief Implements the destructor to the PublishSubscribe class
**/
PublishSubscribe::~PublishSubscribe() {
  node_handle_.shutdown();
}

/**
* @brief Implements the functionality to send the message
**/
void PublishSubscribe::SendMessage() {
}

/**
* @brief Implements the functionality to receive the message through ros subscribe
*        and calls the receiver callback function
**/
void PublishSubscribe::ReceiveMessage() {
  if (nullptr != receiver_callback_) {
    multiplier_subscriber_ = node_handle_.subscribe("random_numbers",
              100, &ReceiverCallback::MultiplierCallback, receiver_callback_);
  } else {
    ROS_WARN("Receiver callback object not created");
  }
  ros::spinOnce();
}

