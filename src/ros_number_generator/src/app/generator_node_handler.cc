/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file		Generator Node Handler
* @author       Rajat Jayanth Shetty <Rajat.Shetty@kpit.com>
* @author       Sujeyendra Tummala <Tummala.Sujeyendra@kpit.com>
* @author       Sasi Kiran Alur	<Sasi.Alur@kpit.com> 
* @date         18 Oct 2017
* @brief        Perform factory creation and processing data functionalities
*
*
**/

/*  include files */
#include "ros/ros.h"
#include "ros_number_generator/app/generator_node_handler.h"
#include "ros_number_generator/app/publish_subscribe.h"

/**
* @brief Default values for Random generator Range
**/
namespace constVariables {
  constexpr uint32_t MAX_RANDOM_VALUE = 1000;
  constexpr uint32_t MIN_RANDOM_VALUE = 0;
}

/**
 * Function name: GeneratorNodeHandler()
 *
 * @brief Constructor for the Node Handler
**/
GeneratorNodeHandler::GeneratorNodeHandler(GeneratorType generator_type) : generator_type_(generator_type) {
  CreateNumberFactory();
  CreateCommunicationFactory();
}

/**
 * Function name: ~GeneratorNodeHandler()
 * 
 * @brief Destructor for the Node Handler
 *
 * Deletes the instance of COmmunication Factory and
 * number generator factory 
**/
GeneratorNodeHandler::~GeneratorNodeHandler() {
  if(nullptr != number_generator_){
	  delete  number_generator_;
	  number_generator_ = nullptr;
  }

  if(nullptr != communication_factory_) {
  	delete communication_factory_;
	communication_factory_ = nullptr;
  }
}

/**
* Function name: CreateNumberFactory
*
* @brief Call number generator factory and Create generator object
* 
**/
void GeneratorNodeHandler::CreateNumberFactory() {
  number_generator_ = new NumberGeneratorFactory();
  if( generator_type_ == GeneratorType::LCG )
    number_generator_->CreateGenerator(new NumberGeneratorLCG(constVariables::MAX_RANDOM_VALUE,constVariables::MIN_RANDOM_VALUE)); 
  else if( generator_type_ == GeneratorType::SRAND )
    number_generator_->CreateGenerator(new NumberGeneratorSRand(constVariables::MAX_RANDOM_VALUE,constVariables::MIN_RANDOM_VALUE));
  else {
    std::cout<< "No object Created, Invalid type"<<std::endl;
  }
}

/**
* Function name: CreateCommunicationFactory
*
* @brief Get the created communication object by factory 
*   
**/
void GeneratorNodeHandler::CreateCommunicationFactory() {
  communication_factory_ = new CommFactory();
  if(nullptr != communication_factory_)
    communication_factory_->CreateCommunicator(new PublishSubscribe(this));
  else
    std::cout<< "Failed to  to create Communication Interface..."<<std::endl;
}

void GeneratorNodeHandler::Execute()
{
  if( nullptr != communication_factory_ )
    communication_factory_->ExecuteCommunication();
  else
    std::cout<< "No instance of Communication Interface available..."<<std::endl;
}
/**
* Function name: GetNumber
*
* @brief Get random value genrator by generator Factory Node
* 
**/
uint32_t GeneratorNodeHandler::GetNumber() {
  uint32_t gen_number = 0;
  if(nullptr != number_generator_){
    gen_number =  number_generator_->ExecuteGenerator();
  } else { 
    gen_number =  0;
  }
  return gen_number;
}

/**
* Function name: GetCommunicationFactory
*
* @brief Get pointer to communication object by factory 
*   
**/
CommFactory* GeneratorNodeHandler::GetCommunicationFactory() {
  return communication_factory_;
}

/**
* Function name: GetNumberFactory
*
* @brief Get the pointer to the generator Factory Node
* 
**/
NumberGeneratorFactory* GeneratorNodeHandler::GetNumberFactory() {
  return number_generator_;
}

