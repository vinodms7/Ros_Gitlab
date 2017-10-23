/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/
/**
* @file     number_generator_test.cc
* 
* @author   Rajat Jayanth Shetty <rajat.shetty@kpit.com>
* 
* @date     18 Oct 2017
* 
* @brief    This file declares Number Generator Factory Class
*
*/

/*  include files */
#include "ros/ros.h"
#include <string>
#include <gtest/gtest.h>
#include <boost/thread/thread.hpp>
#include "ros_ran_num_msg/rand_num.h"

#include "ros_number_generator/core/communication_factory.h"
#include "ros_number_generator/app/generator_node_handler.h"
#include "ros_number_generator/app/publish_subscribe.h"
#include "ros_number_generator/core/generator_config.h"

/*
 * @brief  Verify GetGeneratedNumber - GenerateNumber generating random number
 *         within given range using LCG Implementation
 */
TEST(NumberGeneratorLCG_test, GenerateNumber_test_1) {
  NumberGeneratorLCG<uint32_t> number_generator_lcg(100, 0);

  uint32_t number1 =  101;
  uint32_t number2 =  number_generator_lcg.GetGeneratedNumber();
  EXPECT_GT(number1, number2);
}

/*
 * @brief  To check setMaxRandomValLimit is setting max_random_value & 
 *         min_random_value with given value using LCG Implementation
*/
TEST(NumberGeneratorLCG_test, GenerateNumber_test_2) {
  NumberGeneratorLCG<uint32_t> number_generator_lcg(1000);

  number_generator_lcg.SetRandomValRange(100, 0);

  uint32_t number1 =  101;
  uint32_t number2 =  number_generator_lcg.GetGeneratedNumber();
  EXPECT_GT(number1, number2);
}
/*
 * @brief  To check setMaxRandomValLimit by setting max_random_value_ < 
 *         min_random_value_ using LCG Implementation
*/
TEST(NumberGeneratorLCG_test, GenerateNumber_test_3) {
  NumberGeneratorLCG<uint32_t> number_generator_lcg(1000, 0);

  number_generator_lcg.SetRandomValRange(100, 500);

  uint32_t expected1 =  99;
  uint32_t expected2 =  501;
  uint32_t actual =  number_generator_lcg.GetGeneratedNumber();
  EXPECT_TRUE((expected1 < actual) && (expected2 > actual));
}
/*
 * @brief   To check Number generator's name based on instance
*/
TEST(NumberGeneratorLCG_test, GenerateNumber_test_4) {
  NumberGeneratorLCG<uint32_t> number_generator_lcg(100, 0);

  std::string actual_string =  number_generator_lcg.GetGeneratorName();
  std::string expected_string = "Linear Congruential Generator";
  EXPECT_EQ(actual_string, expected_string);
}

/*
 * @brief  To check GetNumber - GenerateNumber generating random number 
 * within given value based on SRAND Implementation
*/
TEST(NumberGeneratorSRand_test, GenerateNumber_test_1) {
  NumberGeneratorSRand<uint32_t> number_generator_srand(100, 0);

  uint32_t number1 =  101;
  uint32_t number2 =  number_generator_srand.GetGeneratedNumber();
  EXPECT_GT(number1, number2);
}
/*
 * @brief  To check setMaxRandomValLimit by setting 
 *         max_random_value_ < min_random_value_
*/
TEST(NumberGeneratorSRand_test, GenerateNumber_test_2) {
  NumberGeneratorSRand<uint32_t> number_generator_srand(1000, 0);

  number_generator_srand.SetRandomValRange(500, 100);

  uint32_t expected1 =  99;
  uint32_t expected2 =  501;
  uint32_t actual =  number_generator_srand.GetGeneratedNumber();
  EXPECT_TRUE((expected1 < actual) && (expected2 > actual));
}
/*
 * @brief  To check setMaxRandomValLimit by setting 
 *         max_random_value_ < min_random_value_
*/
TEST(NumberGeneratorSRand_test, GenerateNumber_test_3) {
  NumberGeneratorSRand<uint32_t> number_generator_srand(1000, 800);

  number_generator_srand.SetRandomValRange(100, 500);

  uint32_t expected1 =  99;
  uint32_t expected2 =  501;
  uint32_t actual =  number_generator_srand.GetGeneratedNumber();
  EXPECT_TRUE((expected1 < actual) && (expected2 > actual));
}
/*
 * @brief   To check Number generator's name based on instance
*/
TEST(NumberGeneratorSRand_test, GenerateNumber_test_4) {
  NumberGeneratorSRand<uint32_t> number_generator_srand(1000, 800);

  std::string actual_string =  number_generator_srand.GetGeneratorName();
  std::string expected_string = "Default CPP SRand Generator";
  EXPECT_EQ(actual_string, expected_string);
}
/*
 * @brief  To check GetNumber using GeneratorNodeHandler instance
 * LCG Implementation 
*/
TEST(GeneratorNodeHandler_test, GetNumber_test_1) {
  ros::start();

  GeneratorConfig<uint32_t>::ConfigInstance().generator_type_ = "LCG";
  GeneratorNodeHandler<uint32_t> generator_node_handler;

  uint32_t number1 =  1001;
  uint32_t number2 =  generator_node_handler.GetNumber();

  EXPECT_GT(number1, number2);

  ros::shutdown();
}
/*
 * @brief  To check GetNumber using GeneratorNodeHandler instance
 * SRAND Implementation 
*/
TEST(GeneratorNodeHandler_test, GetNumber_test_3) {
  ros::start();
  GeneratorNodeHandler<uint32_t> generator_node_handler;

  uint32_t number1 =  1001;
  uint32_t number2 =  generator_node_handler.GetNumber();

  EXPECT_GT(number1, number2);

  ros::shutdown();
}
/*
 * @brief  To verify NumberGeneratorFactory instance creation
*/
TEST(GeneratorNodeHandler_test, GetNumberFactory_test_1) {
  ros::start();
  GeneratorNodeHandler<uint32_t>* generator_node_handler =
                    new GeneratorNodeHandler<uint32_t>();
  NumberGeneratorFactory<uint32_t> *number_factory =
                    generator_node_handler->GetNumberFactory();
  EXPECT_TRUE(number_factory != NULL);

  delete generator_node_handler;

  ros::shutdown();
}

/*
 * @brief Verify GetCommunicationFactory instance from GeneratorNodeHandler
*/
TEST(CommunicationFactory_test, GetCommunicationFactory_test_1) {
  ros::start();

  GeneratorNodeHandler<uint32_t> generator_node_handler;
  CommFactory<uint32_t> *comm_factory =
                          generator_node_handler.GetCommunicationFactory();
  EXPECT_TRUE(comm_factory != NULL);

  ros::shutdown();
}
/*
 * @brief  Verify GetCommunicationFactory instance from 
 *         GeneratorNodeHandler SRAND
*/
TEST(CommunicationFactory_test, GetCommunicationFactory_test_2) {
  ros::start();
  GeneratorNodeHandler<uint32_t> *generator_node_handler =
                        new GeneratorNodeHandler<uint32_t>();
  CommFactory<uint32_t> *comm_factory =
                            generator_node_handler->GetCommunicationFactory();
  EXPECT_TRUE(comm_factory != NULL);
  delete generator_node_handler;
  ros::shutdown();
}
/*
 * @brief  to check CommunicationInterface instance from  CommFactory
*/
TEST(CommunicationFactory_test, GetCommunicationFactory_test_3) {
  ros::start();
  CommFactory<uint32_t> *communication_factory_ = new CommFactory<uint32_t>();
  communication_factory_->CreateCommunicator(
                                     new PublishSubscribe<uint32_t>(nullptr));

  CommunicationInterface<uint32_t> *comm_interface =
                  communication_factory_->GetCommunicator();

  EXPECT_TRUE(comm_interface != NULL);
  ros::shutdown();
}
/*
 * @brief  to check CommunicationInterface instance creation
*/
TEST(CommunicationFactory_test, GetCommunicationFactory_test_4) {
  ros::start();
  CommFactory<uint32_t> *communication_factory_ = new CommFactory<uint32_t>();
  communication_factory_->CreateCommunicator(
                                     new PublishSubscribe<uint32_t>(nullptr));
  communication_factory_->CreateCommunicator(
                                     new PublishSubscribe<uint32_t>(nullptr));

  CommunicationInterface<uint32_t> *comm_interface =
                  communication_factory_->GetCommunicator();

  EXPECT_TRUE(comm_interface != NULL);
  ros::shutdown();
}


/*
 * @brief  receiveCallback implementattion for Subscriber based Test Cases 
 */
void receiveCallback(const ros_ran_num_msg::rand_num& msg) {
  std::cerr << "             Received message using Subscribe.." << std::endl;
  ros::shutdown();
  EXPECT_TRUE((msg.number1 == 20) && (msg.number2 == 10));
}

/*
 * @brief  timer_lapsed function callback in case negative unit test 
 */
void timer_lapsed_negative_test(const ros::TimerEvent& evt) {
  std::cerr<< "             Time elapsed and no message received"<< std::endl;
  std::cerr<< "             Negative Test case callback function"<< std::endl;
  ros::shutdown();

  EXPECT_TRUE(1);
}
/*
 * @brief  timer_lapsed function callback in case positive unit test 
 */
void timer_lapsed(const ros::TimerEvent& evt) {
  ros::shutdown();

  EXPECT_TRUE(1);
}

/*
 * @brief  to check CommFactory Publishing message with no Publisher instance
*/
TEST(CommunicationFactory_test, GetCommunicationFactory_test_6) {
  ros::start();
  unsigned int nTimer = 0;
  CommFactory<uint32_t> *communication_factory_ = new CommFactory<uint32_t>();

  GeneratorConfig<uint32_t> & generator_config =
                                  GeneratorConfig<uint32_t>::ConfigInstance();


  ros::Subscriber sub = generator_config.generator_handle_->subscribe(
                            "random_numbers", 1000, receiveCallback);
  std::cerr << "             Subscribed to topic random_numbers" << std::endl;

  communication_factory_->CreateCommunicator(nullptr);
  communication_factory_->ExecuteCommunication();

  std::cerr << "             Waiting for Publishers ";
  while ((sub.getNumPublishers() < 1) && (nTimer++ < 5)) {
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    std::cerr << ".";
  }
  if (nTimer >= 5)
    std::cerr << "             No Publishers available"<< std::endl;

  ros::shutdown();
  EXPECT_NE(nTimer, 5);
}

TEST(NumberGeneratorFactory_test, NumberGeneratorFactory_test_1) {
  NumberGeneratorFactory<uint32_t> *generator_factory =
                                       new NumberGeneratorFactory<uint32_t>();
  generator_factory->CreateGenerator(
                                    new NumberGeneratorLCG<uint32_t>(100, 0));
  NumberGenerator<uint32_t> *number_generator =
                                            generator_factory->getGenerator();

  EXPECT_TRUE(number_generator != NULL);

  delete number_generator;
}
/*
* @brief  To verify if 2nd instance of number generator is the actual
*       implementation currently used
*/

TEST(NumberGeneratorFactory_test, NumberGeneratorFactory_test_2) {
  NumberGeneratorFactory<uint32_t> *generator_factory =
                                       new NumberGeneratorFactory<uint32_t>();
  generator_factory->CreateGenerator(
                                   new NumberGeneratorLCG<uint32_t>(1000, 0));
  generator_factory->CreateGenerator(
                                  new NumberGeneratorSRand<uint32_t>(100, 0));

  NumberGenerator<uint32_t> *number_generator =
                                         generator_factory->getGenerator();

  std::string actual_string =  number_generator->GetGeneratorName();
  std::string expected_string = "Default CPP SRand Generator";
  EXPECT_EQ(actual_string, expected_string);

  delete generator_factory;
}
/** Multiplier Arithmetic tests */
/*
 * @brief Fail test to show numberarithmeticfactory is null - TBD
 */
TEST(NumberGeneratorFactory_test, NumberGeneratorFactory_test_3) {
  NumberGeneratorFactory<uint32_t> *generator_factory =
                                      new NumberGeneratorFactory<uint32_t>();

  generator_factory->CreateGenerator(
                                   new NumberGeneratorLCG<uint32_t>(1000, 0));
  generator_factory->CreateGenerator(
                                   new NumberGeneratorLCG<uint32_t>(1000, 0));

  EXPECT_TRUE(generator_factory != NULL);

  delete generator_factory;
}

/*
 * @brief Fail test to show multiply is wrong
 */
TEST(NumberGeneratorFactory_test, NumberGeneratorFactory_test_4) {
  NumberGeneratorFactory<uint32_t> *generator_factory =
                                       new NumberGeneratorFactory<uint32_t>();

  uint32_t number_expected = 10001;
  uint32_t number_actual = generator_factory->ExecuteGenerator();
  EXPECT_GT(number_expected, number_actual);
}


/*
 * @brief  to check if Publisher is able to publish messages
*/
TEST(PublisherSuscribe_test, SendMessage_test_1) {
  ros::start();

  GeneratorConfig<uint32_t> & generator_config =
                              GeneratorConfig<uint32_t>::ConfigInstance();

  ros::Subscriber sub = generator_config.generator_handle_->subscribe(
                                 "random_numbers", 1000, receiveCallback);
  std::cerr<< "             Testing Send Message using Publish."<< std::endl;

  GeneratorNodeHandler<uint32_t> generator_node_handler;
  PublishSubscribe<uint32_t> publish_subcribe(&generator_node_handler);

  ros::Timer timer = generator_config.generator_handle_->createTimer(
                   ros::Duration(5), &timer_lapsed, false, true);

  publish_subcribe.SendMessage(20, 10);

  ros::shutdown();
}
/*
 * @brief  to check if Publisher is able to publish messages
*/
TEST(PublisherSuscribe_test, SendMessage_test_2) {
  ros::start();

  GeneratorConfig<uint32_t> & generator_config =
                                GeneratorConfig<uint32_t>::ConfigInstance();

  generator_config.generator_handle_->setParam("frequency", 5);
  ros::Subscriber sub = generator_config.generator_handle_->subscribe(
                                  "random_numbers", 1000, receiveCallback);
  std::cerr << "             Testing Publish with setParam for frequency.";
  std::cerr << std::endl;

  GeneratorNodeHandler<uint32_t> generator_node_handler;
  PublishSubscribe<uint32_t> publish_subcribe(&generator_node_handler);

  publish_subcribe.SendMessage(20, 10);

  ros::spin();

  ros::shutdown();
}

/**********************negative cases**********************************/
/*
* @brief  Negative test case to verify if Subcriber receves messages from 
*         Publisher if advertised string doesnt match  
*/

TEST(PublisherSuscribe_Negtest, SendMessage_test_1) {
  ros::start();
  GeneratorConfig<uint32_t> & generator_config =
                       GeneratorConfig<uint32_t>::ConfigInstance();

  generator_config.generator_handle_->subscribe("not_random_number",
                                              1000, receiveCallback);
  std::cerr << "             Testing Send Message using Publish...";
  std::cerr << std::endl;
  GeneratorNodeHandler<uint32_t> generator_node_handler;
  PublishSubscribe<uint32_t> publish_subcribe(&generator_node_handler);

  ros::Timer timer = generator_config.generator_handle_->createTimer(
                 ros::Duration(5), &timer_lapsed_negative_test, false, true);

  publish_subcribe.SendMessage(20, 10);

  ros::spin();
}
/*
 * @brief  Entry point for Unit Test implementation
*/
int main(int argc, char **argv) {
  ros::init(argc, argv, "NumberGeneratorNode_Test");
  GeneratorConfig<uint32_t> & generator_config =
                              GeneratorConfig<uint32_t>::ConfigInstance();
  generator_config.generator_handle_ = new ros::NodeHandle();

  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  ros::shutdown();

  return result;
}
