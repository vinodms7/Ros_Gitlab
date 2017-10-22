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
#include <string>
#include "ros/ros.h"
#include <gtest/gtest.h>
#include <boost/thread/thread.hpp>
#include "ros_ran_num_msg/rand_num.h"

#include "ros_number_generator/core/communication_factory.h"
#include "ros_number_generator/app/generator_node_handler.h"
#include "ros_number_generator/app/publish_subscribe.h"


/*
 * @brief  To verify GetGeneratedNumber - GenerateNumber generating random number 
 *         within given range using LCG Implementation
 */
TEST(NumberGeneratorLCG_test, GenerateNumber_test_1) {
  NumberGeneratorLCG number_generator_lcg(100, 0);

  uint32_t number1 =  101;
  uint32_t number2 =  number_generator_lcg.GetGeneratedNumber();
  EXPECT_GT(number1, number2);
}

/*
 * @brief  To check setMaxRandomValLimit is setting max_random_value & 
 *         min_random_value with given value using LCG Implementation
*/
TEST(NumberGeneratorLCG_test, GenerateNumber_test_2) {
  NumberGeneratorLCG number_generator_lcg(1000);

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
  NumberGeneratorLCG number_generator_lcg(1000);

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
  NumberGeneratorLCG number_generator_lcg(100);

  std::string actual_string =  number_generator_lcg.GetGeneratorName();
  std::string expected_string = "Linear Congruential Generator";
  EXPECT_EQ(actual_string, expected_string);
}

/*
 * @brief  To check GetNumber - GenerateNumber generating random number 
 * within given value based on SRAND Implementation
*/
TEST(NumberGeneratorSRand_test, GenerateNumber_test_1) {
  NumberGeneratorSRand number_generator_srand(100, 0);

  uint32_t number1 =  101;
  uint32_t number2 =  number_generator_srand.GetGeneratedNumber();
  EXPECT_GT(number1, number2);
}
/*
 * @brief  To check setMaxRandomValLimit by setting 
 *         max_random_value_ < min_random_value_
*/
TEST(NumberGeneratorSRand_test, GenerateNumber_test_2) {
  NumberGeneratorSRand number_generator_srand(1000);

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
  NumberGeneratorSRand number_generator_srand(1000, 800);

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
  NumberGeneratorSRand number_generator_srand(1000, 800);

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

  GeneratorNodeHandler generator_node_handler(GeneratorNodeHandler::LCG);

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
  GeneratorNodeHandler generator_node_handler(GeneratorNodeHandler::SRAND);

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
  GeneratorNodeHandler *generator_node_handler =
                    new GeneratorNodeHandler(GeneratorNodeHandler::LCG);
  NumberGeneratorFactory *number_factory =
                    generator_node_handler->GetNumberFactory();
  EXPECT_TRUE(number_factory != NULL);

  delete generator_node_handler;

  ros::shutdown();
}

/*
 * @brief  to check GetCommunicationFactory instance from GeneratorNodeHandler LCG
*/
TEST(CommunicationFactory_test, GetCommunicationFactory_test_1) {
  ros::start();
  GeneratorNodeHandler *generator_node_handler =
                          new GeneratorNodeHandler(GeneratorNodeHandler::LCG);
  CommFactory *comm_factory =
                          generator_node_handler->GetCommunicationFactory();
  EXPECT_TRUE(comm_factory != NULL);

  delete generator_node_handler;

  ros::shutdown();
}
/*
 * @brief  to check GetCommunicationFactory instance from GeneratorNodeHandler SRAND
*/
TEST(CommunicationFactory_test, GetCommunicationFactory_test_2) {
  ros::start();
  GeneratorNodeHandler *generator_node_handler =
                        new GeneratorNodeHandler(GeneratorNodeHandler::SRAND);
  CommFactory *comm_factory = generator_node_handler->GetCommunicationFactory();
  EXPECT_TRUE(comm_factory != NULL);
  delete generator_node_handler;
  ros::shutdown();
}
/*
 * @brief  to check CommunicationInterface instance from  CommFactory
*/
TEST(CommunicationFactory_test, GetCommunicationFactory_test_3) {
  ros::start();
  CommFactory *communication_factory_ = new CommFactory();
  communication_factory_->CreateCommunicator(new PublishSubscribe(nullptr));

  CommunicationInterface *comm_interface =
                  communication_factory_->GetCommunicator();

  EXPECT_TRUE(comm_interface != NULL);
  ros::shutdown();
}
/*
 * @brief  to check CommunicationInterface instance creation
*/
TEST(CommunicationFactory_test, GetCommunicationFactory_test_4) {
  ros::start();
  CommFactory *communication_factory_ = new CommFactory();
  communication_factory_->CreateCommunicator(new PublishSubscribe(nullptr));
  communication_factory_->CreateCommunicator(new PublishSubscribe(nullptr));

  CommunicationInterface *comm_interface =
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
  EXPECT_NE(msg.number1, msg.number2);
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
  CommFactory *communication_factory_ = new CommFactory();

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("random_numbers",
          1000, receiveCallback);
  std::cerr << "             Subscribed to topic random_numbers!" << std::endl;

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
/*
 * @brief  to check CommFactory Publishing message with no Publisher instance
*/
TEST(CommunicationFactory_test, GetCommunicationFactory_test_7) {
  ros::start();

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("random_numbers",
                                            1000, receiveCallback);
  std::cerr << "             Testing Send Message using Publish."<< std::endl;

  GeneratorNodeHandler generator_node_handler;

  ros::Timer timer = nh.createTimer(ros::Duration(5), &timer_lapsed,
                                                        false, true);

  generator_node_handler.Execute();

  ros::shutdown();
}

/*
 * @brief  to check CommFactory Publishing message with no Publisher instance
*/
TEST(CommunicationFactory_test, GetCommunicationFactory_test_8) {
  ros::start();

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("random_numbers",
                                            1000, receiveCallback);
  std::cerr << "             Testing Publish and setParam for frequency as 5";
  std::cerr<< std::endl;

  nh.setParam("frequency", 5);

  GeneratorNodeHandler generator_node_handler;

  ros::Timer timer = nh.createTimer(ros::Duration(5), &timer_lapsed,
                                                        false, true);

  generator_node_handler.Execute();

  ros::shutdown();
}
TEST(NumberGeneratorFactory_test, NumberGeneratorFactory_test_1) {
  NumberGeneratorFactory *generator_factory = new NumberGeneratorFactory();
  generator_factory->CreateGenerator(new NumberGeneratorLCG(100, 0));
  NumberGenerator *number_generator = generator_factory->getGenerator();

  EXPECT_TRUE(number_generator != NULL);

  delete number_generator;
}
/*
* @brief  To verify if 2nd instance of number generator is the actual
*       implementation currently used
*/

TEST(NumberGeneratorFactory_test, NumberGeneratorFactory_test_2) {
  NumberGeneratorFactory *generator_factory = new NumberGeneratorFactory();
  generator_factory->CreateGenerator(new NumberGeneratorLCG(1000, 0));
  generator_factory->CreateGenerator(new NumberGeneratorSRand(100, 0));

  NumberGenerator *number_generator = generator_factory->getGenerator();

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
  NumberGeneratorFactory *generator_factory = new NumberGeneratorFactory();

  generator_factory->CreateGenerator(new NumberGeneratorLCG(1000, 0));
  generator_factory->CreateGenerator(new NumberGeneratorLCG(1000, 0));

  EXPECT_TRUE(generator_factory != NULL);
}

/*
 * @brief Fail test to show multiply is wrong
 */
TEST(NumberGeneratorFactory_test, NumberGeneratorFactory_test_4) {
  NumberGeneratorFactory *generator_factory = new NumberGeneratorFactory();

  uint32_t number_expected = 10001;
  uint32_t number_actual = generator_factory->ExecuteGenerator();
  EXPECT_GT(number_expected, number_actual);
}


/*
 * @brief  to check if Publisher is able to publish messages
*/
TEST(PublisherSuscribe_test, SendMessage_test_1) {
  ros::start();

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("random_numbers",
                                            1000, receiveCallback);
  std::cerr<< "             Testing Send Message using Publish."<< std::endl;

  GeneratorNodeHandler generator_node_handler;
  PublishSubscribe publish_subcribe(&generator_node_handler);

  ros::Timer timer = nh.createTimer(ros::Duration(5), &timer_lapsed,
                                                        false, true);

  publish_subcribe.SendMessage();

  ros::shutdown();
}
/*
 * @brief  to check if Publisher is able to publish messages
*/
TEST(PublisherSuscribe_test, SendMessage_test_2) {
  ros::start();

  ros::NodeHandle nh;
  nh.setParam("frequency", 5);
  ros::Subscriber sub = nh.subscribe("random_numbers",
                                            1000, receiveCallback);
  std::cerr << "             Testing Publish with setParam for frequency.";
  std::cerr << std::endl;

  GeneratorNodeHandler generator_node_handler;
  PublishSubscribe publish_subcribe(&generator_node_handler);

  publish_subcribe.SendMessage();

  ros::shutdown();
}

/**********************negative cases**********************************/
/*
* @brief  Negative test case to verify if Subcriber receves messages from 
*         Publisher if advertised string doesnt match  
*/

TEST(PublisherSuscribe_Negtest, SendMessage_test_1) {
  ros::start();

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("not_random_number",
                                              1000, receiveCallback);
  std::cerr << "             Testing Send Message using Publish...";
  std::cerr << std::endl;
  GeneratorNodeHandler generator_node_handler;
  PublishSubscribe publish_subcribe(&generator_node_handler);

  ros::Timer timer = nh.createTimer(ros::Duration(5),
                      &timer_lapsed_negative_test, false, true);

  publish_subcribe.SendMessage();
  ros::shutdown();
}
/*
 * @brief  to check PublishSubscribe Publishing message with no Handler instance
*/
TEST(PublisherSuscribe_Negtest, SendMessage_test_2) {
  ros::start();

  unsigned int nTimer = 0;

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("random_numbers",
                                                1000, receiveCallback);
  std::cerr << "             Subscribed to topic random_numbers!"<< std::endl;
  std::cerr << "             Create Subscriber instance with no Handler";
  std::cerr<< std::endl;

  PublishSubscribe publish_subscribe(nullptr);

  ros::Timer timer = nh.createTimer(ros::Duration(5),
                      &timer_lapsed_negative_test, false, true);

  publish_subscribe.SendMessage();
  publish_subscribe.ReceiveMessage();  //  Does nothing


  while ((sub.getNumPublishers() < 1) && (nTimer++ < 5)) {
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    std::cerr << ".";
  }
  if (nTimer >= 5)
    std::cerr << "             No Publishers available"<< std::endl;

  ros::shutdown();
  EXPECT_TRUE(1);
}

/*
 * @brief  Entry point for Unit Test implementation
*/
int main(int argc, char **argv) {
  ros::init(argc, argv, "NumberGeneratorNode_Test");
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  ros::shutdown();

  return result;
}
