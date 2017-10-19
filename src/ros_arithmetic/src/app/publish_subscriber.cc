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
#include "ros_arithmetic/app/publish_subscribe.h"

ReceiverCallback::ReceiverCallback(MultiplierNodeHandler *myptr) {
  valueptr = myptr;
}

ReceiverCallback::~ReceiverCallback() {
}

void ReceiverCallback::CallbackFunction(const ros_ran_num_msg::rand_num::ConstPtr& value){
  ROS_INFO("Node Received Value 1: [%u] and Value 2: [%u]", value->number1, value->number2);

  uint32_t vresult = valueptr->GetResult(value->number1, value->number2);
  ROS_INFO("The Multiplier value is [%lu]", vresult);
}

PublishSubscribe::PublishSubscribe(MultiplierNodeHandler *multiplier_node_handler){
  multiplier_node_handler_ = multiplier_node_handler;
  
  receiver_callback = new ReceiverCallback(multiplier_node_handler);
}

PublishSubscribe::~PublishSubscribe() {
}

void PublishSubscribe:: SetHandler(MultiplierNodeHandler *multiplier_node_handler) {
}

void PublishSubscribe::ReceiveMessage() {
  multiplier_subscriber_ = node_handle_.subscribe("random_number_srand", 100, &ReceiverCallback::CallbackFunction, receiver_callback);
  
  ros::spinOnce();
}
void PublishSubscribe::SendMessage() {
}

