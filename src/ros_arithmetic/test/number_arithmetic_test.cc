/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file    number_arithmetic_test
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
#include <boost/thread/thread.hpp>

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
  EXPECT_TRUE(value->number1 != 0 && value->number2 != 0);
}


void receivercallback(const ros_ran_num_msg::rand_num::ConstPtr& value) {
  unique_ptr<MultiplierNodeHandler>multiplierObj(
                                            new MultiplierNodeHandler());

  uint32_t vresult = multiplierObj->ProcessData(value->number1,
                                               value->number2);

  EXPECT_GT(vresult, 0);
}


/** Multiplier Node Handler Tests */
/*
 * @brief Check if created multiplier object is created for MUL
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_1) {
  unique_ptr<MultiplierNodeHandler>multiplierObj(
                   new MultiplierNodeHandler(MultiplierNodeHandler::MUL));

  EXPECT_TRUE(multiplierObj != nullptr);
}

/** Multiplier Node Handler Tests */
/*
 * @brief Check if created multiplier object is created for NONE
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test2) {
  unique_ptr<MultiplierNodeHandler>multiplierObj(
                   new MultiplierNodeHandler(MultiplierNodeHandler::NONE));

  EXPECT_FALSE(multiplierObj == nullptr);
}

/*
 * @brief Check if Check if object creation is fine and flow is correct
 * is valid and gets expected result
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_3) {
  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(
                     new MultiplierNodeHandler(MultiplierNodeHandler::MUL));

  uint32_t value1_ = 10;
  uint32_t value2_ = 20;

  uint32_t expected_result_= 200;

  uint32_t actual_result_ = multiplier_node_handler_->ProcessData(value1_,
                                                      value2_);

  EXPECT_TRUE(actual_result_ == expected_result_);
}

/*
 * @brief Check if object creation is not done and flow is valid
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_4) {
  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(
                     new MultiplierNodeHandler(MultiplierNodeHandler::NONE));

  uint32_t value1_ = 10;
  uint32_t value2_ = 20;

  uint32_t expected_result_= 200;

  uint32_t actual_result_ = multiplier_node_handler_->ProcessData(value1_,
                                                                  value2_);

  EXPECT_FALSE(actual_result_ == expected_result_);
}


/*
 * @brief Check if communication factory object is created
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_5) {
  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(
                      new MultiplierNodeHandler(MultiplierNodeHandler::MUL));

  CommFactory *communication_factory_ = 
                      multiplier_node_handler_->GetCommunicationFactory();

  EXPECT_TRUE(communication_factory_ != nullptr);
}

/*
 * @brief Check if arithmetic factory object is created
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_6) {
  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(
                    new MultiplierNodeHandler(MultiplierNodeHandler::MUL));

  EXPECT_TRUE(multiplier_node_handler_->GetArithmeticFactory() != nullptr);
}

/*
 * @brief Test to pass invalid value for multiplication
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_7) {
  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(
                         new MultiplierNodeHandler(MultiplierNodeHandler::MUL));

  uint32_t value1_;
  uint32_t value2_ = 20;

  uint32_t expected_result_;

  uint32_t actual_result_ = multiplier_node_handler_->ProcessData(value1_,
                                                                  value2_);

  EXPECT_FALSE(expected_result_ == actual_result_);
}

/*
 * @brief Test to show passing multiply type is not computing addition
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_8) {
  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(
                      new MultiplierNodeHandler(MultiplierNodeHandler::MUL));

  uint32_t value1_ = 10;
  uint32_t value2_ = 20;

  uint32_t expected_result_ = 30;

  uint32_t actual_result_ = multiplier_node_handler_->ProcessData(value1_,
                                                                  value2_);

  EXPECT_FALSE(expected_result_ == actual_result_);
}


/*
 * @brief Test to show passing multiply type is not computing substraction
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_9) {
  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(
                      new MultiplierNodeHandler(MultiplierNodeHandler::MUL));

  uint32_t value1_ = 10;
  uint32_t value2_ = 20;

  uint32_t expected_result_= 10;

  uint32_t actual_result_ = multiplier_node_handler_->ProcessData(value1_,
                                                                  value2_);

  EXPECT_FALSE(expected_result_ == actual_result_);
}

/*
 * @brief Test to execute
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_10) {
  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(
                      new MultiplierNodeHandler(MultiplierNodeHandler::MUL));
 multiplier_node_handler_->Execute();
 
 EXPECT_TRUE(multiplier_node_handler_->GetCommunicationFactory() != nullptr);
}

/*
 * @brief false , Test to execute
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_11) {
  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(
                      new MultiplierNodeHandler(MultiplierNodeHandler::MUL));
 multiplier_node_handler_->Execute();
 
 EXPECT_FALSE(multiplier_node_handler_->GetCommunicationFactory() == nullptr);
}

/*
 * @brief Fail test to show ADD doesnt create an object of factory
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_13) {
  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(
                 new MultiplierNodeHandler(MultiplierNodeHandler::ADD));
  NumberArithmeticFactory *num_factory_ = 
                       multiplier_node_handler_->GetArithmeticFactory();
  
  EXPECT_TRUE(num_factory_->GetArithmeticOperation() == nullptr);  
}

void PublishData() {
  /** Holds the reference to the publisher object */
  ros::Publisher rand_num_publisher1_;
  /** Holds the reference of the nodehandle object */
  ros::NodeHandle node_handle1_;

  rand_num_publisher1_ = node_handle1_.advertise<ros_ran_num_msg::rand_num>(
                                                  "random_number_srand", 100);

  ros::Rate loop_rate(1);

  ros_ran_num_msg::rand_num value;

  while (ros::ok()) {
    value.number1 = 10;
    value.number2 = 10;

    rand_num_publisher1_.publish(value);
    loop_rate.sleep();

    ROS_INFO("\n I am in publishing data");
  }
}

/*
 * @brief 
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_publish) {
  uint8_t initial_value = 0;

  uint8_t final_value_ = 1;

  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(
                       new MultiplierNodeHandler(MultiplierNodeHandler::MUL));

  boost::thread thread1(PublishData);

  ros::spinOnce();

  ros::NodeHandle node_handle_;

  multiplier_node_handler_->Execute();

  pthread_cancel(thread1.native_handle());

  ros::shutdown();

  final_value_ = 0;

  ROS_INFO("\n I am out of loop");

  EXPECT_EQ(initial_value, final_value_);
}

/** Communication Factory Tests */
/*
 * @brief Fail test to show communication factory deletes the previous 
 * instance of publish subscribe
 */
TEST(Communication_factory_test, Communication_factory_test_1) {
  CommFactory *communication_factory_ = new CommFactory();

  communication_factory_->CreateCommunicator(new PublishSubscribe(nullptr));
  EXPECT_TRUE(communication_factory_ != nullptr);

  communication_factory_->CreateCommunicator(nullptr);

  EXPECT_FALSE(communication_factory_->GetCommunicator() != nullptr);
}

/*
 * @brief Fail test to execute communication doesnt happen
 */
TEST(Communication_factory_test, Communication_factory_test_2) {
  CommFactory *communication_factory_ = new CommFactory();
  communication_factory_->CreateCommunicator(nullptr);
  communication_factory_->ExecuteCommunication();

  EXPECT_FALSE(communication_factory_->GetCommunicator() != nullptr);
}

/*
 * @brief Test to create communication factory created and communication
 * does not happen
 */
TEST(Communication_factory_test, Communication_factory_test_3) {
  CommFactory *communication_factory_ = new CommFactory();

  communication_factory_->CreateCommunicator(nullptr);

  EXPECT_TRUE(communication_factory_ != nullptr);
}

/*
 * @brief Failure Test to execute communication factory created and execute
 * communication ivoked without creating communication object
 */
TEST(Communication_factory_test, Communication_factory_test_4) {
  CommFactory *communication_factory_ = new CommFactory();

  communication_factory_->ExecuteCommunication();

  EXPECT_FALSE(communication_factory_->GetCommunicator() != nullptr);
}


/** Publish Subscribe Tests */
/*
 * @brief //to check if receive is happening through right channel
 */
TEST(PublishSubscribe_test, PublishSubscribe_test_1) {
  ros::Subscriber multiplier_subscriber_;

  ros::NodeHandle node_handle_;

  MultiplierNodeHandler *multiplierobj_ = new MultiplierNodeHandler();

  ReceiverCallback *receiverobj_ = new ReceiverCallback(multiplierobj_);

  multiplier_subscriber_ = node_handle_.subscribe("random_number_srand",
                                                  100, receivercallback);

  ros::spinOnce();

  ros::spinOnce();
}

/*
 * @brief Subscribe test - To check if random numbers are coming
 */
TEST(PublishSubscribe_test, PublishSubscribe_test_2) {
  ros::NodeHandle node_handle_;
  ros::Subscriber test_subscriber;

  test_subscriber = node_handle_.subscribe("random_number_srand",
                                              100, &cbfunction);

  ros::spinOnce();

  ros::spinOnce();
}

/*
 * @brief Subscribe test - To check if random numbers are coming
 */
TEST(PublishSubscribe_test, PublishSubscribe_test_3) {
  ros::NodeHandle node_handle_;
  ros::Subscriber test_subscriber;

  test_subscriber = node_handle_.subscribe("random_number_srand",
                                                100, &cbfunction);

  ros::spinOnce();

  ros::spinOnce();
}

/*
 * @brief Subscribe test - To check if random numbers are coming
 */
TEST(PublishSubscribe_test, PublishSubscribe_test_4) {
  PublishSubscribe *pub_sub_ = new PublishSubscribe(NULL);

  pub_sub_->ReceiveMessage();

  pub_sub_->SendMessage();

  ros::spinOnce();
  ros::spinOnce();

  delete pub_sub_;
}

/** Number Arithmetic Factory tests */
/*
 * @brief Check whether previous instance is not invalid
 */
TEST(number_arithmetic_factory_test, number_arithmetic_factory_test_1) {
  unique_ptr<NumberArithmeticFactory> number_factory_(
                               new NumberArithmeticFactory());

  unique_ptr<NumberArithmeticFactory> number_factory_new(
                               new NumberArithmeticFactory());

  number_factory_->CreateArithmeticOperation(new NumberMultiplier());
  number_factory_new->CreateArithmeticOperation(new NumberMultiplier());

  EXPECT_TRUE(number_factory_ != nullptr);
}

/*
 * @brief Fail Test to show numberarithmeticfactory does not create MUL object
 * due to wrong param in switch
 */
TEST(number_arithmetic_factory_test, number_arithmetic_factory_test_2) {
  unique_ptr<NumberArithmeticFactory> number_factory_(
                               new NumberArithmeticFactory());

  number_factory_->CreateArithmeticOperation(nullptr);   
  
  EXPECT_FALSE(number_factory_->GetArithmeticOperation() != nullptr);
}

/*
 * @brief failed case , Check if arithmetic factory perform execute
 * arithmetic operation without creating operation object
 */
TEST(number_arithmetic_factory_test, number_arithmetic_factory_test_3) {  
  unique_ptr<NumberArithmeticFactory> num_arth_factory_(
                                         new NumberArithmeticFactory());
  uint32_t value1_ = 10;
  uint32_t value2_ = 20;

  uint32_t expected_result_ = 200;

  uint32_t actual_result_ = 
                      num_arth_factory_->ExecuteArithmeticOperation(value1_,
                                                                  value2_);

  EXPECT_FALSE(expected_result_ == actual_result_);
}

/*
 * @brief failed case, Check if arithmetic factory perform execute
 * arithmetic operation when operation is add and returns add
 */
TEST(number_arithmetic_factory_test, number_arithmetic_factory_test_4) {  
  unique_ptr<NumberArithmeticFactory> num_arth_factory_(
                                      new NumberArithmeticFactory());
  num_arth_factory_->CreateArithmeticOperation(new NumberMultiplier()); 
  uint32_t value1_ = 10;
  uint32_t value2_ = 20;

  uint32_t expected_result_ = 30;

  uint32_t actual_result_ = 
                      num_arth_factory_->ExecuteArithmeticOperation(value1_,
                                                                  value2_);

  EXPECT_FALSE(expected_result_ == actual_result_);
}

/*
 * @brief Check if arithmetic factory perform execute
 * arithmetic operation and return value
 */
TEST(number_arithmetic_factory_test, number_arithmetic_factory_test_5) {  
  unique_ptr<NumberArithmeticFactory> num_arth_factory_(
                                      new NumberArithmeticFactory());
  num_arth_factory_->CreateArithmeticOperation(new NumberMultiplier()); 
  uint32_t value1_ = 10;
  uint32_t value2_ = 20;

  uint32_t expected_result_ = 200;

  uint32_t actual_result_ = 
                      num_arth_factory_->ExecuteArithmeticOperation(value1_,
                                                                  value2_);

  EXPECT_EQ(expected_result_,actual_result_);
}

/*
 * @brief Fail test to show create arithmetic is not null
 */
TEST(number_arithmetic_factory_test, number_arithmetic_factory_test_6) {
  unique_ptr<NumberArithmeticFactory> num_arth_factory_(
                                      new NumberArithmeticFactory());

  num_arth_factory_->CreateArithmeticOperation(new NumberMultiplier());
  EXPECT_FALSE(num_arth_factory_->GetArithmeticOperation() == nullptr);
}

/** Number Multiplier Test */
/*
 * @brief Check Valid data is returned in DoArithmetic Operation
 */
TEST(number_multiplier_test,number_multiplier_test_1) {
  unique_ptr<NumberMultiplier> number_factory_(
                               new NumberMultiplier());
  uint32_t value1_ = 10;
  uint32_t value2_ = 20;

  uint32_t expected_result_ = 200;

  uint32_t actual_result_ = 
                      number_factory_->DoArithmeticOperation(value1_,
                                                                  value2_);

  EXPECT_EQ(expected_result_,actual_result_);  
}

/*
 * @brief Check Valid data is returned in DoMultiplication Operation
 */
TEST(number_multiplier_test,number_multiplier_test_2) {
  unique_ptr<NumberMultiplier> number_factory_(
                               new NumberMultiplier());
  uint32_t value1_ = 10;
  uint32_t value2_ = 20;

  uint32_t expected_result_ = 200;

  uint32_t actual_result_ = 
                      number_factory_->DoArithmeticOperation(value1_,
                                                                  value2_);

  EXPECT_EQ(expected_result_,actual_result_);  
}

/*
 * @brief failure case, Check invalid data is returned in DoArithmetic Operation
 */
TEST(number_multiplier_test,number_multiplier_test_3) {
  unique_ptr<NumberMultiplier> number_factory_(
                               new NumberMultiplier());
  uint32_t value1_;
  uint32_t value2_ = 20;

  uint32_t expected_result_ = 200;

  uint32_t actual_result_ = 
                      number_factory_->DoArithmeticOperation(value1_,
                                                                  value2_);

  EXPECT_FALSE(expected_result_ == actual_result_);  
}

/*
 * @brief failure case, Check invalid data is returned in DoMultiplication Operation
 */
TEST(number_multiplier_test,number_multiplier_test_4) {
  unique_ptr<NumberMultiplier> number_factory_(
                               new NumberMultiplier());
  uint32_t value1_;
  uint32_t value2_ = 20;

  uint32_t expected_result_ = 200;

  uint32_t actual_result_ = 
                      number_factory_->DoArithmeticOperation(value1_,
                                                                  value2_);

  EXPECT_FALSE(expected_result_ == actual_result_);  
}

/**---------------------------------- MAIN ---------------------------------*/
int main(int argc, char **argv) {
  uint32_t test_result;

  ros::init(argc, argv, "number_arithmetic_test");

  ::testing::InitGoogleTest(&argc, argv);

  ROS_INFO("Starting Arithmetic Tests");

  test_result = RUN_ALL_TESTS();

  ROS_INFO("Done with Arithmetic Tests");

  return test_result;
}
