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

#ifndef NUMBER_GENERATOR_INTERFACE_H
#define NUMBER_GENERATOR_INTERFACE_H

#include <string>

class NumberGenerator {
 public:
  explicit NumberGenerator(uint32_t _nMaxRandomValue, uint32_t _nMinRandomValue);
  virtual ~NumberGenerator();

  virtual void SetRandomValRange(uint32_t _nMaxRandomValue, uint32_t _nMinRandomValue);
  virtual uint32_t GetCurrentRandomNumber();
  virtual uint32_t GetNumber();
  virtual std::string GetGeneratorName() = 0;

 protected:
  virtual uint32_t GenerateNumber() = 0;

  uint32_t max_random_value_;  // Max random value
  uint32_t min_random_value_;  // Min random value
  uint32_t current_seed_;   // Current random seed value
  uint32_t current_random_number_; // current random number
};
#endif /* NUMBER_GENERATOR_INTERFACE_H*/
