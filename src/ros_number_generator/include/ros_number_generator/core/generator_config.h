/****************************************************************************
*     Copyright (C) 2017 by KPIT Technologies                     *
*                                                                           *
****************************************************************************/

/**
* @file      generator_config.h
*
* @author    Sujeyendra Tummala (Tummala.Sujeyendra@kpit.com)
* @author    Rajat Jayanth Shetty (Rajat.Shetty@kpit.com)
*
* @date      18-Oct- 2017
*
* @brief     Holds the configuration parameters
*
**/
#ifndef _GENERATOR_CONFIG_H
#define _GENERATOR_CONFIG_H

/*! Include Files */
#include <string>
#include "ros_number_generator/app/generator_node_handler.h"

/*! Class Declarations */
template<typename T>
class GeneratorConfig {
 public:
  static GeneratorConfig<T> & ConfigInstance()
  {
    static GeneratorConfig<T> gconfig;
    return gconfig;
  }

  float frequency_;                              /*! Frequency of publishing the numbers */
  std::string generator_type_;                   /*! Type of message generation */
  std::string communication_type_;               /*! Type of communication interface */
  NumberGeneratorFactory<T>* number_generator_;  /*! Pointer to generator factory */
  CommFactory<T>*   communication_factory_;      /*! Pointer to communication factory */
  ros::NodeHandle* generator_handle_;            /* Publisher node handle */

 private:
  GeneratorConfig():frequency_(1.0),
                      generator_type_("SRAND"),
                      communication_type_("PUB_SUB"),
                      number_generator_(nullptr),
                      communication_factory_(nullptr),
                      generator_handle_(nullptr)  { }

  ~GeneratorConfig() { }

};
 template class GeneratorConfig<uint32_t>;

#endif /* _GENERATOR_CONFIG_H */

