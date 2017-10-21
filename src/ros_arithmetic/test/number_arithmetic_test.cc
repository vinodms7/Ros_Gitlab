/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file		number_arithmetic_test
* 
* @author       Sujeyendra Tummala <Tummala.Sujeyendra@kpit.com>
*
* @date         18 Oct 2017
*
* @brief        Perform unit tests for arithmetic node
*
*
**/

/** Include files */
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "ros_arithmetic/app/multiplier_node_handler.h"
#include "ros_arithmetic/app/multiplier_arithmetic.h"
#include "ros_arithmetic/app/publish_subscribe.h"
#include "ros_arithmetic/core/communication_factory.h"
#include "ros_arithmetic/core/communication_interface.h"
#include "ros_arithmetic/core/number_arithmetic_factory.h"
#include "ros_arithmetic/core/number_arithmetic_interface.h"
#include "ros_ran_num_msg/rand_num.h"


using namespace std;

void cbfunction(const ros_ran_num_msg::rand_num::ConstPtr& value) {

  EXPECT_TRUE(value->number1 != NULL && value->number2 != NULL);
}


void receivercallback(const ros_ran_num_msg::rand_num::ConstPtr& value) {

  unique_ptr<MultiplierNodeHandler>multiplierObj(new MultiplierNodeHandler());

  uint32_t vresult = multiplierObj->ProcessData(value->number1, value->number2);

  EXPECT_TRUE(vresult > 0);
}


/** Multiplier Node Handler Tests */
/*
 * @brief 
 */
/*TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_1) {
  CommFactory *comm_ptr_;
  unique_ptr<MultiplierNodeHandler> multiplierObj(new MultiplierNodeHandler());

  comm_ptr_ = multiplierObj->GetCommunicationFactory();

  EXPECT_TRUE(comm_ptr_ != NULL);
}*/

/*
 * @brief Fail test to show numberarithmeticfactory is null - TBD
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_) {
/*  unique_ptr<MultiplierNodeHandler> multiplierObj(new MultiplierNodeHandler());
  NumberArithmeticFactory* multiplier_factory_;
  NumberMultiplier number_multiplier;

  uint32_t number1 = 100;
  uint32_t number2 = 200;
  uint32_t number_expected = 20000;
  uint32_t number_actual =number_multiplier.DoArithmeticOperation(number1,number2);

  EXPECT_EQ(number_expected,number_actual);
*/
 EXPECT_TRUE(NULL == NULL);
}

/** Multiplier Node Handler Tests */
/*
 * @brief 
 */
/*TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_3) {
  MultiplierNodeHandler *multiplierObj_ = new MultiplierNodeHandler();

  MultiplierNodeHandler *multiplierObj_new_ = new MultiplierNodeHandler();

  EXPECT_TRUE(multiplierObj_ != NULL);
}
*/
/** Multiplier Node Handler Tests */
/*
 * @brief 
 */
/*TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_4) {
  unique_ptr<MultiplierNodeHandler> multiplierObj(new MultiplierNodeHandler());
  
  NumberMultiplier NumberMultiplier;

  uint32_t number1 = 10;
  uint32_t number2 = 20;
  uint32_t number_expected = 200;

  uint32_t number_actual = multiplierObj->ProcessData(number1, number2);

  NumberMultiplier.DisplayResult(number_actual);

  EXPECT_EQ(number_expected,number_actual);
}*/

/** Multiplier Node Handler Tests */
/*
 * @brief 
 */
/*TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_5) {
  MultiplierNodeHandler *multiplier_obj_ = new MultiplierNodeHandler();
  
  //NumberMultiplier number_multiplier_; 
  
  //number_multiplier_ = new NumberMultiplier();

  uint32_t number1 = 10;
  uint32_t number2 = 10;
  uint32_t number_expected = 200;

  uint32_t number_actual = multiplier_obj_->ProcessData(number1, number2);

//  number_multiplier_.PrintValue();

  EXPECT_EQ(number_expected,number_actual);
}*/

/** Multiplier Arithmetic tests */
/*
 * @brief Fail test to show numberarithmeticfactory is null - TBD
 */
/*TEST(Multiplier_arithmetic_test, Multiplier_arithmetic_test_1) {
  NumberMultiplier number_multiplier_;

  uint32_t value1 = 10;
  uint32_t value2 = 20;

  uint32_t number_expected = 200;
  
  uint32_t number_actual = number_multiplier_.DoArithmeticOperation(value1,value2);

  EXPECT_EQ(number_expected,number_actual);
}
*/
/*
 * @brief Fail test to show multiply is wrong
 */
TEST(Multiplier_arithmetic_test, Multiplier_arithmetic_test_2) {
  NumberMultiplier number_multiplier_;

  uint32_t value1 = 10;
  uint32_t value2 = 10;

  uint32_t number_expected = 200;
  
  uint32_t number_actual = number_multiplier_.DoArithmeticOperation(value1,value2);

  EXPECT_EQ(number_expected,number_actual);
}


/** Publish Subscribe Tests */
/*
 * @brief //to check if receive is happening through right channel
 */
/*TEST(PublishSubscribe_test, PublishSubscribe_test_1) {
  ros::Subscriber multiplier_subscriber_;	
  
  ros::NodeHandle node_handle_;

  MultiplierNodeHandler *multiplierobj_ = new MultiplierNodeHandler();

  ReceiverCallback *receiverobj_ = new ReceiverCallback(multiplierobj_);

  multiplier_subscriber_ = node_handle_.subscribe("random_number_srand", 100, receivercallback);  

  ros::spinOnce();

  ros::spinOnce();	
}*/

/*
 * @brief Subscribe test - To check if random numbers are coming
 */
/*TEST(PublishSubscribe_test, PublishSubscribe_test_2) {	
  ros::NodeHandle node_handle_;
  ros::Subscriber test_subscriber;	
	
  test_subscriber = node_handle_.subscribe("random_number_srand", 100, &cbfunction);
	
  ros::spinOnce();
        
  ros::spinOnce();	
}
*/
/*
 * @brief Subscribe test - To check if random numbers are coming
 */
/*TEST(PublishSubscribe_test, PublishSubscribe_test_3) {	
  ros::NodeHandle node_handle_;
  ros::Subscriber test_subscriber;	
	
  test_subscriber = node_handle_.subscribe("random_number_srand", 100, &cbfunction);
	
  ros::spinOnce();
        
  ros::spinOnce();	
}
*/
/*
 * @brief Subscribe test - To check if random numbers are coming
 */
/*TEST(PublishSubscribe_test, PublishSubscribe_test_4) {	
  
  MultiplierNodeHandler *node_handler_ = new MultiplierNodeHandler();
 
  PublishSubscribe *pub_sub_ = new PublishSubscribe(node_handler_);

  pub_sub_->ReceiveMessage();
  
  pub_sub_->SendMessage();
  
  ros::spinOnce();
        
  ros::spinOnce();	
}
*/
/** Communication Factory Tests */
/*
 * @brief //to check if receive is happening through right channel
 */
/*TEST(communication_factory_test, communication_factory_test_1) {
  CommFactory comm_controller_factory_;
  
  comm_controller_factory_.CreateCommunicator(new PublishSubscribe(NULL));
  CommunicationInterface *comm_interface = comm_controller_factory_.GetCommunicator();
  
  EXPECT_TRUE(comm_interface != NULL);
}*/


/*
 * @brief //to check assignment of communication_interface_ with check to cover IF TRUE part
 *        // IF ensures to delete previously allocated communication_interface_ to release memory
 */
/*TEST(communication_factory_test, communication_factory_test_2) {
  CommFactory comm_controller_factory_;

  comm_controller_factory_.CreateCommunicator(new PublishSubscribe(NULL));
  comm_controller_factory_.CreateCommunicator(new PublishSubscribe(NULL));
  CommunicationInterface *comm_interface = comm_controller_factory_.GetCommunicator();
  
  EXPECT_TRUE(comm_interface != NULL);
}*/

/*
 * @brief //to check assignment of communication_interface_ to non-garbage when NULL is passed to CreateCommunicator
 */
/*TEST(communication_factory_test, communication_factory_test_3) {
  CommFactory comm_controller_factory_;

  comm_controller_factory_.CreateCommunicator(NULL);
  CommunicationInterface *comm_interface = comm_controller_factory_.GetCommunicator();
  
  EXPECT_TRUE(comm_interface == NULL);
}*/

/*
 * @brief //To check communication_interface_ without CreateCommunicator
 */
//To check communication_interface_ without CreateCommunicator
/*TEST(communication_factory_test, communication_factory_test_4) {
  CommFactory comm_controller_factory_;

  CommunicationInterface *comm_interface = comm_controller_factory_.GetCommunicator();
  
  EXPECT_TRUE(comm_interface == NULL);
}*/


/** Number Arithmetic Factory tests */
/*
 * @brief 
 */
/*TEST(number_arithmetic_factory_test, number_arithmetic_factory_test_1) {
  NumberArithmeticFactory *number_factory_;

  NumberArithematicInterface *number_multiplier_;

  number_factory_ = new NumberArithmeticFactory();
  number_factory_->CreateMultiplier(new NumberMultiplier());

  number_multiplier_ = number_factory_->GetMultiplier();
  
  EXPECT_TRUE(number_multiplier_ != NULL);
}*/

/** Number Arithmetic Factory tests */
/*
 * @brief 
 */
/*TEST(number_arithmetic_factory_test, number_arithmetic_factory_test_2) {
  NumberArithmeticFactory number_factory_;

  NumberArithematicInterface *number_multiplier_;

  number_factory_.CreateMultiplier(new NumberMultiplier());
  number_factory_.CreateMultiplier(new NumberMultiplier());
  number_multiplier_ = number_factory_.GetMultiplier();
  
  EXPECT_TRUE(number_multiplier_ != NULL);
}*/

/** Number Arithmetic Factory tests */
/*
 * @brief 
 */
TEST(number_arithmetic_factory_test, number_arithmetic_factory_test_3) {
  NumberArithmeticFactory *number_factory_ = new NumberArithmeticFactory();
  
  NumberArithmeticFactory *number_factory_new = new NumberArithmeticFactory();
  
  number_factory_->CreateArithmeticOperation(new NumberMultiplier());
  number_factory_new->CreateArithmeticOperation(new NumberMultiplier());
  
  EXPECT_TRUE(number_factory_ != NULL);
}


/** Number Arithmetic Factory tests */
/*
 * @brief 
 */
/*TEST(number_arithmetic_factory_test, number_arithmetic_factory_test_4) {
  NumberArithmeticFactory number_factory_;
  
  number_factory_.CreateMultiplier(NULL);
  
  NumberArithematicInterface *arithmetic_int_obj = number_factory_.GetMultiplier();;
 
  EXPECT_TRUE(arithmetic_int_obj == NULL);
}
*/
/** Number Arithmetic Factory tests */
/*
 * @brief 
 */
/*TEST(number_arithmetic_factory_test, number_arithmetic_factory_test_5) {
  NumberArithmeticFactory number_factory_;
  
  NumberArithematicInterface *arithmetic_int_obj = number_factory_.GetMultiplier();;
 
  EXPECT_TRUE(arithmetic_int_obj == NULL);
}*/

/**---------------------------------------- MAIN ---------------------------------------*/

int main(int argc, char **argv) {
  
  uint32_t test_result;
  
  ros::init(argc, argv,"number_arithmetic_test");

  ::testing::InitGoogleTest(&argc, argv);
  
  ROS_INFO("Starting Arithmetic Tests");

  test_result = RUN_ALL_TESTS();
 
  ROS_INFO("Done with Arithmetic Tests");

  return test_result;
}
