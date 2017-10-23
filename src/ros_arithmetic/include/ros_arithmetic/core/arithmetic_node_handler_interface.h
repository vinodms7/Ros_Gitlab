/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file       arithmetic_node_handler_interface.h
*
* @author     Sasi Kiran <Sasi.Alur@kpit.com>
*
* @date       18-Oct-2017
*
* @brief      This is a interface class for Arithmetic Node Handler
**/

#ifndef ARITHMETIC_NODE_HANDLER_INTERFACE_H
#define ARITHMETIC_NODE_HANDLER_INTERFACE_H

/*! Include files */
#include <string>

/*! Class Declarations */
template<class T, class RT>
class ArithmeticNodeHandlerInterface {
 public:
  /**
  * Function name: Execute
  *
  * @brief Execute the operation and communication
  *
  * @return  void
  **/
  virtual void Execute() = 0;

  /**
  * Function name: ProcessData
  *
  * @brief process data and return value
  * 
  *
  * @param[in]  T
  *               Holds the value1 parameter
  *
  * @param[in]  T
  *               Holds the value2 parameter
  *
  * @return     RT
  *               Holds the return value after result is computed
  **/
  virtual RT ProcessData(T, T) = 0;
};

#endif /*MULTIPLIER_NODE_H */

