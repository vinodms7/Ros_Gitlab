/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file         number_generator_node.cc
* 
* @author       Rajat Jayanth Shetty <Rajat.Shetty@kpit.com>
* 
* @date         18 Oct 2017
* 
* @brief        Entry point for the Random Generator Node
*
**/

/*  include files  */
#include <memory>

#include <ros/ros.h>
#include "ros_number_generator/app/generator_node_handler.h"
#include "ros_number_generator/core/generator_ node_handler_interface.h"
#include "ros_number_generator/core/generator_config.h"

/*! Main function */
int main(int argc, char **argv) {
  float frequency = 0;
  std::string value;

  ros::init(argc, argv, "NumberGeneratorNode");

  GeneratorConfig<uint32_t> & generator_config = GeneratorConfig<uint32_t>::ConfigInstance();

  generator_config.generator_handle_ = new ros::NodeHandle();

  if ( !generator_config.generator_handle_->getParam("frequency", frequency) ) {
    ROS_WARN("Frequency Param not received...");
  } else {
    ROS_INFO("Frequency Param is [%f]", frequency);
    generator_config.frequency_ = frequency;
  }
  
  if ( !generator_config.generator_handle_->getParam("gentype", value) ) {
    ROS_WARN("Generator type not received, Using default generator interface.");
  } else {
    generator_config.generator_type_ = value;
    ROS_INFO("Generator type is [%s]", value.c_str());
  }
  
  if ( !generator_config.generator_handle_->getParam("commtype", value) ) {
    ROS_WARN("Communication type not received, Using default communication interface.");
  } else {
    ROS_INFO("Communication type is [%s]", value.c_str());
    generator_config.communication_type_ = value;
  }

  std::unique_ptr<NodeHandlerInterface<uint32_t>>randNumberObj(
                        new GeneratorNodeHandler<uint32_t>());

  ros::spin();
  return 0;
}

