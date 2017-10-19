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
#include "ros_number_generator/app/publish_subscribe.h"


PublishSubscribe::PublishSubscribe(GeneratorNodeHandler *p_generator_node_handler){
  generator_node_handler = p_generator_node_handler;

  rand_num_publisher = nodeHandle.advertise<ros_ran_num_msg::rand_num>("random_number_srand", 100);
}

PublishSubscribe::~PublishSubscribe() {
}

void PublishSubscribe::SendMessage() {
  ros::Rate rate(1);
  ros_ran_num_msg::rand_num value;
  
  while(nodeHandle.ok()) {
    value.number1 = generator_node_handler->GetNumber();
    value.number2 = generator_node_handler->GetNumber();

    rand_num_publisher.publish(value);

    ROS_INFO("Number Generator Value 1: [%u] and Value 2: [%u]", value.number1, value.number2);
    
    ros::spinOnce();
    rate.sleep();
  }
}

void PublishSubscribe::ReceiveMessage() {
}
