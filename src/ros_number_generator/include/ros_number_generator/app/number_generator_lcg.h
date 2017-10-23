/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file      number_generator_lcg.h
* 
* @author    Rajat Jayanth Shetty  <rajat.shetty@kpit.com>
* 
* @date      18 Oct 2017
* 
* @brief     Linear Congruential Generator Implementation
* 
* The Linear Congruential Generator (LCG) is based on Park & Miller 
* algorithm found in "Numerical Recipes".  
*
*/
#ifndef NUMBER_GENERATOR_LCG_H
#define NUMBER_GENERATOR_LCG_H

/*! Include files */
#include <string>
#include "ros_number_generator/core/number_generator_interface.h"

template<class T>
class NumberGeneratorLCG : public NumberGenerator<T> {
 public:
  /**
  * Function name: NumberGeneratorLCG()
  *
  * @brief Constructor for LCG generator
  * 
  * The constructor initializes the Random generator and specifies the range 
  * between which randomnumbers need to be generator.
  *
  * @param[in]  T max_random_value Maximum range for Random Generator
  * @param[in]  T min_random_value Minimum range for Random Generator
  *
  **/
  NumberGeneratorLCG(T max_random_value = 1000,
                          T min_random_value = 0);

  /**
  * Function name: ~NumberGeneratorLCG()
  *
  * @brief Destructor for LCG generator instance
  *
  *
  **/
  ~NumberGeneratorLCG();

  /**
  * Function name: SetRandomValRange()
  *
  * @brief Function call to set the the random genrator range
  *
  * The Range between which the random numbers need to be generator 
  *
  * @param[in]  T  max_random_value Maximum range for Random Generator
  * @param[in   T  min_random_value Minimum range for Random Generator
  *
  * @return     void
  **/
  void SetRandomValRange(T max_random_value,
                         T min_random_value);

  /**
  * Function name: GetGeneratedNumber()
  *
  * @brief Function call to query Number generator to provide a Random number 
  *
  * @param[in]  None
  *
  * @return     DataType Returns a Random number generated
  **/
  T GetGeneratedNumber();

  /**
  * Function name: GetGeneratorName()
  *
  * @brief Function call to query Generator name or Implementation name
  *
  * @param[in]  None
  *
  * @return     std::string  String containing the name of the Random 
  * Generator implementation
  **/
  std::string GetGeneratorName() const;

 protected:
   /**
   * Function name: GenerateNumber()
   *
   * @brief       Function call containing the actual implementation 
   *  of the Number Generator
   *
   * @param[in]    None
   *
   * @return      DataType Generates and returns the number generated 
   * by the implementation
   **/
  T GenerateNumber();

 private:
  T max_random_value_;       /* Max random value range */
  T min_random_value_;       /* Min random value range */
  T current_seed_;           /* Current random seed value */
  T current_random_number_;  /* current random number */
};
  template class NumberGeneratorLCG<uint32_t>;
  template class NumberGeneratorLCG<float>;

#endif  /* NUMBER_GENERATOR_LCG_H */

