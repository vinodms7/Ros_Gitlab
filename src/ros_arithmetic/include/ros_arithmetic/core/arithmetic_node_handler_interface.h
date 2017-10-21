/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file		Arithmetic Node Handler Interface
* @author       Sasi Kiran <Sasi.Alur@kpit.com>
* @date         18 oct 2017
* @brief        This is a interface class for Arithmetic Node Handler
*
*
**/

#ifndef ARITHMETIC_NODE_HANDLER_INTERFACE_H
#define ARITHMETIC_NODE_HANDLER_INTERFACE_H

/* include files */
#include <string>

class ArithmeticNodeHandlerInterface {
public:
  /**
  * Function name: Execute
  *
  * @brief Execute the operation and communication
  *
  * @return	void
  **/
  virtual void Execute() = 0;

  /**
  * Function name: ProcessData
  *
  * @brief process data and return value
  * 
  *
  * @param[in]	uint32_t  value1 This is first parameter
  *
  * @param[in]	uint32_t  value2 This is second parameter 
  *
  * @return	uint32_t  return value after result is computed
  **/
  virtual uint32_t ProcessData(uint32_t, uint32_t) = 0;

};

#endif /*MULTIPLIER_NODE_H */
