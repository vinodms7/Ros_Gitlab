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
#include <sstream>
#include <memory>

#include "ros_arithmetic/app/multiplier_node_handler.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MultiplierNode");

  std::unique_ptr<MultiplierNodeHandler> multiplierObj(new MultiplierNodeHandler());

  multiplierObj->GetCommunicationFactory()->GetCommunicator()->ReceiveMessage();

  ros::spin();

  return 0;
}
