/****************************************************************************
*     Copyright (C) 2017 by KPIT Technologies                     *
*                                                                           *
****************************************************************************/

/**
* @file      communication_factory.cc
*
* @author    Sujeyendra Tummala (Tummala.Sujeyendra@kpit.com)
*
* @date      18-Oct- 2017
*
* @brief     This files implements the communication factory class and
*            performs the below features
*              1) Assigns the instance of communication interface object
*              2) Returns the communication interface object
*              3) Deletes the instance of communication interface object
**/

/*! Include Files */
#include <ros/ros.h>
#include <cstddef>
#include "ros_arithmetic/core/communication_factory.h"

/*! Class Definitions */
/**
* @brief Implements the constructor that initialises the communication 
*        interface object to NULL
**/
template<class T, class RT>
CommFactory<T, RT>::CommFactory() {
  communication_interface_ = nullptr;
}

/**
* @brief Implements the destructor that deletes the communication 
*        interface object if existing and intializes it to NULL
**/
template<class T, class RT>
CommFactory<T, RT>::~CommFactory() {
  if ( nullptr != communication_interface_ ) {
    delete communication_interface_;
    communication_interface_ = nullptr;
  }
}

/**
* @brief Implements the functionality to delete the existing reference to 
*        the the communication object and initialize to the received object
**/
template<class T, class RT>
void CommFactory<T, RT>::CreateCommunicator(
                                   CommunicationInterface<T, RT> *comm_int) {
  if ( nullptr != communication_interface_ ) {
    delete communication_interface_;
    communication_interface_ = nullptr;
  }
  communication_interface_ = comm_int;
}

/**
* @brief Implements the functionality to get the reference of 
* CommunicationInterface     
**/
template<class T, class RT>
CommunicationInterface<T, RT>* CommFactory<T, RT>::GetCommunicator() {
  return communication_interface_;
}

/**
* @brief Implements the functionality receive the message    
**/
template<class T, class RT>
void CommFactory<T, RT>::ExecuteCommunication() {
  if ( nullptr != communication_interface_ )
    communication_interface_->ReceiveMessage();
  else
    ROS_WARN("Communication is not established");
}

