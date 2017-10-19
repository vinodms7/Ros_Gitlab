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
#include "ros_number_generator/app/generator_node_handler.h"
#include "ros_number_generator/app/publish_subscribe.h"

namespace constVariables {
  constexpr uint32_t MAX_RANDOM_VALUE = 1000;
  constexpr uint32_t MIN_RANDOM_VALUE = 0;
}
GeneratorNodeHandler::GeneratorNodeHandler() {
  CreateNumberFactory();
  CreateCommunicationFactory();
}

GeneratorNodeHandler::~GeneratorNodeHandler() {
  if(NULL != number_generator_){
	  delete  number_generator_;
	  number_generator_ = NULL;
  }

  if(NULL != comm_controller_factory_) {
  	delete comm_controller_factory_;
	  comm_controller_factory_ = NULL;
  }
}

void GeneratorNodeHandler::CreateNumberFactory() {
  number_generator_ = new NumberGeneratorFactory();
  number_generator_->CreateGenerator(new NumberGeneratorLCG(constVariables::MAX_RANDOM_VALUE,constVariables::MIN_RANDOM_VALUE));
}

void GeneratorNodeHandler::CreateCommunicationFactory() {
  comm_controller_factory_ = new CommFactory();
  comm_controller_factory_->CreateCommunicator(new PublishSubscribe(this));
}

CommFactory* GeneratorNodeHandler::GetCommunicationFactory() {
  return comm_controller_factory_;
}

uint32_t GeneratorNodeHandler::GetNumber() {
  return number_generator_->GetGenerator()->GetNumber();
}
