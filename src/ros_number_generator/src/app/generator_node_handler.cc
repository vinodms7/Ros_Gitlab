/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file         generator_node_handler.cc
* 
* @author       Rajat Jayanth Shetty <Rajat.Shetty@kpit.com>
* @author       Sujeyendra Tummala <Tummala.Sujeyendra@kpit.com>
* @author       Sasi Kiran Alur  <Sasi.Alur@kpit.com> 
* 
* @date         18 Oct 2017
* 
* @brief        Perform factory creation and processing data functionalities
*
*
**/

/*! Include files */
#include "ros/ros.h"
#include "ros_number_generator/app/generator_node_handler.h"
#include "ros_number_generator/app/publish_subscribe.h"
#include "ros_number_generator/core/generator_config.h"

/**
* @brief Default values for Random generator Range
**/
namespace constVariables {
  constexpr uint32_t MAX_RANDOM_VALUE = 1000;
  constexpr uint32_t MIN_RANDOM_VALUE = 0;
}

/*! Class Defintions */
/**
 * Function name: GeneratorNodeHandler()
 *
 * @brief Constructor for the Node Handler
**/
template<class T>
GeneratorNodeHandler<T>::GeneratorNodeHandler() {
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
template<class T>
GeneratorNodeHandler<T>::~GeneratorNodeHandler() {
  if ( nullptr != number_generator_factory_ ) {
    delete  number_generator_factory_;
    number_generator_factory_ = nullptr;
  }

  if ( nullptr != communication_factory_ ) {
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
template<class T>
void GeneratorNodeHandler<T>::CreateNumberFactory() {
  number_generator_factory_ = new NumberGeneratorFactory<T>();

  if ( nullptr != number_generator_factory_ ) {
    if ( (GeneratorConfig<T>::ConfigInstance().generator_type_) == "LCG" ) {
      number_generator_factory_->CreateGenerator(new NumberGeneratorLCG<T>(
        constVariables::MAX_RANDOM_VALUE, constVariables::MIN_RANDOM_VALUE));
    } else if (GeneratorConfig<T>::ConfigInstance().
                                                generator_type_ == "SRAND") {
      number_generator_factory_->CreateGenerator(new NumberGeneratorSRand<T>(
          constVariables::MAX_RANDOM_VALUE, constVariables::MIN_RANDOM_VALUE));
    } else {
      ROS_WARN("No Generator type object Created, Invalid type");
    }

    GeneratorConfig<T>::ConfigInstance().number_generator_ =
                                                  number_generator_factory_;
  } else {
    ROS_WARN("Number factory object could not be created");
  }
}

/**
* Function name: CreateCommunicationFactory
*
* @brief Get the created communication object by factory 
*   
**/
template<class T>
void GeneratorNodeHandler<T>::CreateCommunicationFactory() {
  communication_factory_ = new CommFactory<T>();

  if ( nullptr != communication_factory_ ) {
    if (GeneratorConfig<T>::ConfigInstance().
                                          communication_type_ == "PUB_SUB") {
    communication_factory_->CreateCommunicator(new PublishSubscribe<T>(this));
    } else {
      ROS_WARN("No Comm Factory type object Created. Invalid type");
    }
    GeneratorConfig<T>::ConfigInstance().communication_factory_ =
                                                   communication_factory_;
  } else {
    ROS_WARN("Commnication factory object could not be created");
  }
}

/**
* Function name: CommCallback
*
* @brief Call back function from Timer to publish random numbers
*   
**/
template<class T>
void GeneratorNodeHandler<T>::CommCallback(const ros::TimerEvent& evt) {
  T value1;
  T value2;
  value1 = GeneratorConfig<T>::ConfigInstance().
                                       number_generator_->ExecuteGenerator();
  value2 = GeneratorConfig<T>::ConfigInstance().
                                       number_generator_->ExecuteGenerator();

  GeneratorConfig<T>::ConfigInstance().communication_factory_->
                              GetCommunicator()->SendMessage(value1, value2);
}

/**
* Function name: Execute
*
* @brief Function to start communication
*   
**/
template <class T>
void GeneratorNodeHandler<T>::Execute() {
  if ( nullptr != communication_factory_ )
    communication_factory_->ExecuteCommunication();
  else
    ROS_WARN("No instance of Communication Interface available");
}

/**
* Function name: GetNumber
*
* @brief Get random value genrator by generator Factory Node
* 
**/
template<class T>
T GeneratorNodeHandler<T>::GetNumber() {
  T gen_number = 0;
  if ( nullptr != number_generator_factory_ ) {
    gen_number =  number_generator_factory_->ExecuteGenerator();
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
template<class T>
CommFactory<T>* GeneratorNodeHandler<T>::GetCommunicationFactory() {
  return communication_factory_;
}

/**
* Function name: GetNumberFactory
*
* @brief Get the pointer to the generator Factory Node
* 
**/
template<class T>
NumberGeneratorFactory<T>* GeneratorNodeHandler<T>::GetNumberFactory() {
  return number_generator_factory_;
}

