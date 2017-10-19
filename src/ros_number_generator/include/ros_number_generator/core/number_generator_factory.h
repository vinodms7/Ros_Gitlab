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
#ifndef NUMBER_GENERATOR_FACTORY_H
#define NUMBER_GENERATOR_FACTORY_H

#include "ros_number_generator/core/number_generator_interface.h"

class NumberGeneratorFactory {
 public:
  NumberGeneratorFactory();
  ~NumberGeneratorFactory();

  void CreateGenerator(NumberGenerator* pGenerator);
  NumberGenerator* GetGenerator() const;

 private:
  NumberGenerator *number_generator_;
};
#endif
