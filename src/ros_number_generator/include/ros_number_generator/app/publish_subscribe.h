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
#ifndef PUBLISHER_SUBSCRIBE_H
#define PUBLISHER_SUBSCRIBE_H

#include "ros_number_generator/core/communication_interface.h"
#include "ros_number_generator/app/generator_node_handler.h"
#include "ros/ros.h"
#include "ros_ran_num_msg/rand_num.h"

class PublishSubscribe : public CommunicationInterface {
public:
  explicit PublishSubscribe(GeneratorNodeHandler* node_handler);
  virtual ~PublishSubscribe();
  void SendMessage();
  void ReceiveMessage();

private:
  GeneratorNodeHandler *generator_node_handler;
  ros::NodeHandle nodeHandle;
  ros::Publisher rand_num_publisher;
};
#endif /*PUBLISHER_SUBSCRIBE_H */

