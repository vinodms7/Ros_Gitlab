/****************************************************************************
*     Copyright (C) 2017 by KPIT Technologies                               *
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
#include "ros/ros.h"
#include "ros_number_generator/core/communication_interface.h"
#include "ros_number_generator/app/generator_node_handler.h"
#include "ros_ran_num_msg/rand_num.h"

/*! Class Declarations */
template<class T>
class PublishSubscribe : public CommunicationInterface<T> {
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
  explicit PublishSubscribe(NodeHandlerInterface<T>* node_handler_interface);

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
  * @param[in]  T Value1
  *                Holds the first value to be sent
  *
  * @param[in]  T Value1
  *                Holds the second value to be sent
  *
  * @return     void
  **/
  void SendMessage(T value1, T value2);

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
  NodeHandlerInterface<T>* generator_node_handler_;

  /** Holds the reference to the publisher object */
  ros::Publisher rand_num_publisher_;

  /* Holds the timer for publish callback */
  ros::Timer timer_;
};
  template class PublishSubscribe<uint32_t>;
  template class PublishSubscribe<float>;

#endif /*PUBLISHER_SUBSCRIBE_H */

