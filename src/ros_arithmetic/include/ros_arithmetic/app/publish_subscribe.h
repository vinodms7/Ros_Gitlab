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

#include "ros_arithmetic/app/multiplier_node_handler.h"
#include "ros_arithmetic/core/communication_interface.h"
#include "ros/ros.h"
#include "ros_ran_num_msg/rand_num.h"

class ReceiverCallback {
 public:
  ReceiverCallback(MultiplierNodeHandler *);
  ~ReceiverCallback();

  void CallbackFunction(const ros_ran_num_msg::rand_num::ConstPtr& value);
  MultiplierNodeHandler *valueptr;
};

class PublishSubscribe : public CommunicationInterface {
 public:
  PublishSubscribe(MultiplierNodeHandler *);
  virtual ~PublishSubscribe();
  void SendMessage();
  void ReceiveMessage();
  void SetHandler(MultiplierNodeHandler *);

 private: 
  ReceiverCallback *receiver_callback;
  MultiplierNodeHandler *multiplier_node_handler_;
  ros::NodeHandle node_handle_;
  ros::Subscriber multiplier_subscriber_;
};
#endif /* PUBLISHER_SUBSCRIBE_H */
