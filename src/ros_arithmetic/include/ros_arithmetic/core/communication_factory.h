/****************************************************************************
*     Copyright (C) 2017 by KPIT Technologies                     *
*                                                                           *
****************************************************************************/

/**
* @file      communication_factory.h
*
* @author    Sujeyendra Tummala (Tummala.Sujeyendra@kpit.com)
*
* @date      18-Oct- 2017
*
* @brief     This file declares the communication factory class 
*
**/

#ifndef COMMUNICATION_FACTORY_H
#define COMMUNICATION_FACTORY_H

/*! Include Files */
#include "ros_arithmetic/core/communication_interface.h"

/*! Class Declarations */
class CommFactory {
 public:
  /**
  * Function name: Constructor
  *
  * @brief      Declares the constructor that initialises the comm interface
  *             object to NULL
  *
  * @param[in]  None
  **/
  CommFactory();

  /**
  * Function name: Destructor
  *
  * @brief      Declares the destructor that deletes the communication 
  *             interface object if existing and intializes it to NULL
  *
  * @param[in]  None
  **/  
  ~CommFactory();

  /**
  * Function name: CreateCommunicator
  *
  * @brief      Creating communication object
  *
  * @param[in]  CommunicationInterface* 
  *               Holds the reference of communication interface object
  *
  * @return     void
  **/
  void CreateCommunicator(CommunicationInterface* );

  /**
  * Function name: GetCommunicator
  *
  * @brief Implements the functionality get the pointer to CommunicationInterface
  *
  * @param[in]  None
  *
  * @return     CommunicationInterface* 
  *               Holds the pointer to communication interface  
  **/
  CommunicationInterface* GetCommunicator();

  /**
  * Function name: ExecuteCommunication
  *
  * @brief Execute communication to send message
  *
  * @param[in]  None
  *
  * @return     void
  **/
  void ExecuteCommunication();

 private:
  /** Holds the pointer to communication interface object */
  CommunicationInterface* communication_interface_;
};

#endif

