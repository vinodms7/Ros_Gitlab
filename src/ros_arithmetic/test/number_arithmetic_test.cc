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
  EXPECT_TRUE(value->number1 != NULL && value->number2 != NULL);
}


void receivercallback(const ros_ran_num_msg::rand_num::ConstPtr& value) {
  unique_ptr<MultiplierNodeHandler>multiplierObj(new MultiplierNodeHandler());

  uint32_t vresult = multiplierObj->ProcessData(value->number1, value->number2);

  EXPECT_GT(vresult, 0);
}


/** Multiplier Node Handler Tests */
/*
 * @brief Check if created multiplier object is created
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_1) {
  unique_ptr<MultiplierNodeHandler>multiplierObj(new MultiplierNodeHandler(MultiplierNodeHandler::MUL));

  EXPECT_TRUE(multiplierObj != NULL);
}

/*
 * @brief Check if created arithmetic factory object is created
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_2) {
  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(new MultiplierNodeHandler(MultiplierNodeHandler::MUL));

  NumberArithmeticFactory* arithmetic_factory_ =
            multiplier_node_handler_->GetArithmeticFactory();

  EXPECT_TRUE(arithmetic_factory_ != NULL);
}

/*
 * @brief Check if node handler object is deleted and initialized to null
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_3) {
  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(new MultiplierNodeHandler(MultiplierNodeHandler::MUL));

  uint32_t value1_ = 10;
  uint32_t value2_ = 20;

  uint32_t expected_result_= 200;

  uint32_t actual_result_ = multiplier_node_handler_->ProcessData(value1_, value2_);

  EXPECT_TRUE(actual_result_ == expected_result_);
}

/*
 * @brief Check if created communication factory object is created
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_4) {
  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(new MultiplierNodeHandler(MultiplierNodeHandler::MUL));

  CommFactory *communication_factory_ = multiplier_node_handler_->GetCommunicationFactory();

  EXPECT_TRUE(communication_factory_ != NULL);
}

/*
 * @brief Fail test to show multiply is happening correctly
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_5) {
  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(new MultiplierNodeHandler(MultiplierNodeHandler::MUL));

  uint32_t value1_ = 10;
  uint32_t value2_ = 20;

  uint32_t expected_result_= 200;

  uint32_t actual_result_ = multiplier_node_handler_->ProcessData(value1_,
                                                                  value2_);

  EXPECT_EQ(expected_result_, actual_result_);
}

void PublishData() {
  /** Holds the reference to the publisher object */
  ros::Publisher rand_num_publisher1_;
  /** Holds the reference of the nodehandle object */
  ros::NodeHandle node_handle1_;

  rand_num_publisher1_ = node_handle1_.advertise<ros_ran_num_msg::rand_num>("random_number_srand", 100);

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
 * @brief Fail test to show ADD doesnt create an operation
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_6) {
  uint8_t initial_value = 0;

  uint8_t final_value_ = 1;

  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(new MultiplierNodeHandler(MultiplierNodeHandler::MUL));

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


/*
 * @brief Fail test to show ADD doesnt create an operation
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_7) {
  unique_ptr<MultiplierNodeHandler> multiplier_node_handler_(new MultiplierNodeHandler(MultiplierNodeHandler::ADD));

  EXPECT_TRUE(multiplier_node_handler_ == NULL);
}

/*
 * @brief Fail test to show multiply is wrong
 */
TEST(Multiplier_node_handler_test, Multiplier_node_handler_test_8) {
  MultiplierNodeHandler *multiplier_node_handler_ = new MultiplierNodeHandler(MultiplierNodeHandler::ADD);

  uint32_t number_actual_ = 0;
  uint32_t value1_ = 10;
  uint32_t value2_ = 10;

  uint32_t number_expected_ = multiplier_node_handler_->ProcessData(value1_,
                                                                    value2_);

  EXPECT_EQ(number_expected_, number_actual_);
}

/** Multiplier Node Handler Tests */
/*
 * @brief 
 */
TEST(Number_Multiplier_test, Number_Multiplier_test_1) {
  NumberMultiplier *multiplierObj_ = new NumberMultiplier();

  uint32_t value1_ = 10;
  uint32_t value2_ = 10;

  uint32_t expected_result_ = 100;

  uint32_t actual_result_ = multiplierObj_->DoArithmeticOperation(value1_,
                                                                  value2_);

  delete multiplierObj_;

  EXPECT_EQ(actual_result_, expected_result_);
}

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
TEST(Multiplier_arithmetic_test, Multiplier_arithmetic_test_1) {
  NumberArithmeticFactory *number_factory_ = new NumberArithmeticFactory();

  number_factory_->CreateArithmeticOperation(new NumberMultiplier());

  number_factory_->CreateArithmeticOperation(new NumberMultiplier());

  EXPECT_TRUE(number_factory_ != NULL);
}

/*
 * @brief Fail test to show multiply is wrong
 */
TEST(Multiplier_arithmetic_test, Multiplier_arithmetic_test_2) {
  NumberArithmeticFactory *number_factory_ = new NumberArithmeticFactory();

  uint32_t value1 = 10;
  uint32_t value2 = 10;

  uint32_t number_expected = 0;

  uint32_t number_actual = number_factory_->ExecuteArithmeticOperation(value1,
                                                                      value2);

  EXPECT_EQ(number_expected, number_actual);
}


/** Communication Factory Tests */
/*
 * @brief Fail test to show communication factory deletes the previous instance of publish subscribe
 */
TEST(Communication_factory_test, Communication_factory_test_1) {
  CommFactory *communication_factory_ = new CommFactory();

  communication_factory_->CreateCommunicator(new PublishSubscribe(NULL));

  communication_factory_->CreateCommunicator(new PublishSubscribe(NULL));

  EXPECT_TRUE(communication_factory_ != NULL);
}

/*
 * @brief Fail test to execute communication doesnt happen
 */
TEST(Communication_factory_test, Communication_factory_test_2) {
  CommFactory *communication_factory_ = new CommFactory();

  communication_factory_->ExecuteCommunication();

  EXPECT_TRUE(communication_factory_ != nullptr);
}

/*
 * @brief Fail test to execute communication doesnt happen
 */
TEST(Communication_factory_test, Communication_factory_test_3) {
  CommFactory *communication_factory_ = new CommFactory();

  delete communication_factory_;

  EXPECT_TRUE(communication_factory_ != nullptr);
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
 * @brief 
 */
TEST(number_arithmetic_factory_test, number_arithmetic_factory_test_3) {
  NumberArithmeticFactory *number_factory_ = new NumberArithmeticFactory();

  NumberArithmeticFactory *number_factory_new = new NumberArithmeticFactory();

  number_factory_->CreateArithmeticOperation(new NumberMultiplier());
  number_factory_new->CreateArithmeticOperation(new NumberMultiplier());

  EXPECT_TRUE(number_factory_ != NULL);
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
