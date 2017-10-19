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
#include <memory>

#include "ros/ros.h"
#include "ros_number_generator/app/generator_node_handler.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "NumberGeneratorNode");

  std::unique_ptr<GeneratorNodeHandler> randNumberObj(new GeneratorNodeHandler());
  randNumberObj->GetCommunicationFactory()->GetCommunicator()->SendMessage();

  return 0;
}

