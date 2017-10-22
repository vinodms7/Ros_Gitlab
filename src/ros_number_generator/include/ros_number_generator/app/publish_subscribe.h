/****************************************************************************
*     Copyright (C) 2017 by KPIT Technologies                     *
*                                                                           *
****************************************************************************/

/**
* @file      publish_subscribe.h
*
* @author    Sujeyendra Tummala (Tummala.Sujeyendra@kpit.com)
*
* @date      18-Oct- 2017
*
* @brief     This file declares the Publish subscribe 
*            class that implements the communication interface class
**/

#ifndef PUBLISHER_SUBSCRIBE_H
#define PUBLISHER_SUBSCRIBE_H

/** Include files */
#include "ros_number_generator/core/communication_interface.h"
#include "ros_number_generator/app/generator_node_handler.h"
#include "ros/ros.h"
#include "ros_ran_num_msg/rand_num.h"

/*! Class Declarations */
class PublishSubscribe : public CommunicationInterface {
 public:
  /**
  * Function name: PublishSubscribe
  *
  * @brief      Constructor to the publishSubsribe class that 
  *             assigns the received generator node handle reference 
  *             and advertises the topic to be published
  *
  * @param[in]  NodeHandlerInterface*
  *              Pointer to the reference generator node handler 
  *
  **/
  explicit PublishSubscribe(NodeHandlerInterface* node_handler_interface);

  /**
  * Function name: ~PublishSubscribe
  *
  * @brief      Defines the destructor to the PublishSubscribe class
  *
  * @param[in]  None
  *
  *
  **/
  ~PublishSubscribe();

  /**
  * Function name: SendMessage
  *
  * @brief      Defines the functionality to get the generated 
  *             random number and publish it
  *
  * @param[in]  None
  *
  * @param[out] None
  *
  * @return     Void
  *
  **/
  void SendMessage();

  /**
  * Function name: ReceiveMessage
  *
  * @brief      Defines the functionality to receive the message
  *
  * @param[in]  None
  *
  * @param[out] None
  *
  * @return     Void
  *
  **/
  void ReceiveMessage();

 private:
  /** Holds the reference to the generator node handler object */
  NodeHandlerInterface *generator_node_handler_;

  /** Holds the reference of the nodehandle object */
  ros::NodeHandle node_handle_;

  /** Holds the reference to the publisher object */
  ros::Publisher rand_num_publisher_;
};
#endif /*PUBLISHER_SUBSCRIBE_H */

