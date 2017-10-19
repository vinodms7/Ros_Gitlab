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
#ifndef COMMUNICATION_INTERFACE_H
#define COMMUNICATION_INTERFACE_H

class CommunicationInterface {
 public:
  CommunicationInterface();
  virtual ~CommunicationInterface();
  virtual void SendMessage() = 0;
  virtual void ReceiveMessage() = 0;
};

#endif
