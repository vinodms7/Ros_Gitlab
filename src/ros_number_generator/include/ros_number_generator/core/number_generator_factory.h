/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file     number_generator_factory.h
* 
* @author   Rajat Jayanth Shetty <rajat.shetty@kpit.com>
* 
* @date     18 Oct 2017
* 
* @brief    This file declares Number Generator Factory Class
*
*/
#ifndef NUMBER_GENERATOR_FACTORY_H
#define NUMBER_GENERATOR_FACTORY_H

/*! Include files */
#include "ros_number_generator/core/number_generator_interface.h"

template<class T>
class NumberGeneratorFactory {
 public:
  /**
  * Function name: NumberGeneratorFactory
  *
  * @brief Constructor for the Number Generator Factory class
  *
  * @param[in]  None
  *  
  **/
  NumberGeneratorFactory();

  /**
  * Function name: ~NumberGeneratorFactory
  *
  * @brief Destructor for the Number Generator Factory class
  *
  **/
  ~NumberGeneratorFactory();

  /**
  * Function name: CreateGenerator()
  *
  * @brief Receives an instance for the Number Generator
  *
  * @param[in]  NumberGenerator number_generator contains the data pointer to
  *              the instance of the Number generator
  *
  * @return    void
  **/
  void CreateGenerator(NumberGenerator<T>* number_generator);

  /**
  * Function name: ExecuteGenerator
  *
  * @brief Execute generator and return generated value
  *
  * @param[in]  None
  *
  * @return  T  return generated value
  **/
  T ExecuteGenerator() const;

  /**
  * Function name: getGenerator()
  *  
  * @brief If an instance of Number generator already exist, the current method will 
  * return the instance 
  *  
  * @returns Instance of the Number Generator
  **/
  NumberGenerator<T>* getGenerator();

 private:
  NumberGenerator<T>* number_generator_;   /*! Ptr to the number generator */
};
  template class NumberGeneratorFactory<uint32_t>;
  template class NumberGeneratorFactory<float>;

#endif /* NUMBER_GENERATOR_FACTORY_H */

