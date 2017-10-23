/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file        Multiplier Node
*
* @author      Sasi Kiran <Sasi.Alur@kpit.com>
*
* @date        18-Oct-2017
*
* @brief       Main Function to create multiplier node handler
*
**/

/*! Include files */
#include <ros/ros.h>
#include <sstream>
#include <memory>

#include "ros_arithmetic/app/multiplier_node_handler.h"
#include "ros_arithmetic/core/arithmetic_node_handler_interface.h"

/*! Main Function */
int main(int argc, char **argv) {
  ros::init(argc, argv, "MultiplierNode");

  std::unique_ptr<ArithmeticNodeHandlerInterface<uint32_t, uint64_t>>
                   multiplierObj(
                   new MultiplierNodeHandler<uint32_t, uint64_t>("MUL"));
  multiplierObj->Execute();

  ros::spin();

  return 0;
}

