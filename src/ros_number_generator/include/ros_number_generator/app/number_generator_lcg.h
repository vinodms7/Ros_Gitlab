/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file number_generator_lcg.h
* @author Rajat Jayanth Shetty  <rajat.shetty@kpit.com>
* @date 18 Oct 2017
* @brief    Linear Congruential Generator Implementation
* 
* The Linear Congruential Generator (LCG) is based on Park & Miller 
* algorithm found in "Numerical Recipes".  
*
*/
#ifndef NUMBER_GENERATOR_LCG_H
#define NUMBER_GENERATOR_LCG_H

/* include files */
#include "ros_number_generator/core/number_generator_interface.h"

class NumberGeneratorLCG : public NumberGenerator {
 public:
  /**
  * Function name: NumberGeneratorLCG()
  *
  * @brief Constructor for LCG generator
  * 
  * The constructor initializes the Random generator and specifies the range between which random 
  * numbers need to be generator.
  *
  * @param[in]	uint32_t nMaxRandomValue Maximum value the Random Generator  should generate
  * @param[in]	uint32_t nMinRandomValue Minimum value the Random Generator  should generate
  *
  * @return		void
  **/
  NumberGeneratorLCG(uint32_t nMaxRandomValue = 1000, uint32_t nMinRandomValue = 0);
  
  /**
  * Function name: ~NumberGeneratorLCG()
  *
  * @brief Destructor for LCG generator instance
  *
  * @return		void
  **/
  ~NumberGeneratorLCG();
  
  /**
  * Function name: SetRandomValRange()
  *
  * @brief Function call to set the the random genrator range
  *
  * The Range between which the random numbers need to be generator 
  *
  * @param[in]	uint32_t  _nMaxRandomValue Maximum range the Random Generator should generate values within
  * @param[in   uint32_t  _nMinRandomValue Minimum range the Random Generator should generate values within
  *
  * @return		 void
  **/
  void SetRandomValRange(uint32_t max_random_value, uint32_t min_random_value);
  
  /**
  * Function name: GetGeneratedNumber()
  *
  * @brief Function call to query for Number generator to provide a Random number 
  *
  * @param[in]	None
  *
  * @return		 uint32_t Returns a Random number generated
  **/
  uint32_t GetGeneratedNumber();

  /**
  * Function name: GetGeneratorName()
  *
  * @brief Function call to query Generator name or Implementation name
  *
  * @param[in]	None
  *
  * @return		 std::string  String containing the name of the Random Generator implementation
  **/
  std::string GetGeneratorName();

 protected:
   /**
   * Function name: GenerateNumber()
   *
   * @brief       Function call containing the actual implementation of the Number Generator
   *
   * @param[in]	  None
   *
   * @return		  uint32_t Generates and returns the number generated by the impelementation
   **/
  uint32_t GenerateNumber();

 private:
  uint32_t max_random_value_;       // Max random value range
  uint32_t min_random_value_;       // Min random value range
  uint32_t current_seed_;           // Current random seed value
  uint32_t current_random_number_;  // current random number
};
#endif  /* NUMBER_GENERATOR_LCG_H */
