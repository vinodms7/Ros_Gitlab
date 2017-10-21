/****************************************************************************
* Copyright (C) 2017 by KPIT Technologies                                  *
*                                                                          *
****************************************************************************/

#include <gtest/gtest.h>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"

#include "ros_number_generator/app/generator_node_handler.h"
#include "ros_number_generator/core/communication_factory.h"
#include "ros_number_generator/app/publish_subscribe.h"
#include "ros_ran_num_msg/rand_num.h"

/*
 * @brief  To check GetNumber - GenerateNumber generating random number 
 *         within given value
 */
TEST(NumberGeneratorLCG_test, GenerateNumber_test_1) {
  NumberGeneratorLCG number_generator_lcg(100);

  uint32_t number1 =  101;
  uint32_t number2 =  number_generator_lcg.GetGeneratedNumber();
  ASSERT_GT(number1,number2);
}

/*
 * @brief  To check setMaxRandomValLimit is setting max_random_value & 
 *         min_random_value with given value
*/
TEST(NumberGeneratorLCG_test, GenerateNumber_test_2) {
  NumberGeneratorLCG number_generator_lcg(1000);

  number_generator_lcg.SetRandomValRange(0, 100);
  
  uint32_t number1 =  101;
  uint32_t number2 =  number_generator_lcg.GetGeneratedNumber();
  ASSERT_GT(number1,number2);
}
/*
 * @brief  To check setMaxRandomValLimit by setting max_random_value_ < 
 *         min_random_value_
*/
TEST(NumberGeneratorLCG_test, GenerateNumber_test_3) {
  NumberGeneratorLCG number_generator_lcg(1000);

  number_generator_lcg.SetRandomValRange(500, 100);
  
  uint32_t expected1 =  99;
  uint32_t expected2 =  501;
  uint32_t actual =  number_generator_lcg.GetGeneratedNumber();
  EXPECT_TRUE((expected1<actual) && (expected2>actual));
}
/*
 * @brief  
*/
TEST(NumberGeneratorLCG_test, GenerateNumber_test_4) {
  NumberGeneratorLCG number_generator_lcg(100);

  std::string actual_string =  number_generator_lcg.GetGeneratorName();
  std::string expected_string = "Linear Congruential Generator";
  ASSERT_EQ(actual_string,expected_string);
}

/*
 * @brief  To check GetNumber - GenerateNumber generating random number 
 * within given value
*/
TEST(NumberGeneratorSRand_test, GenerateNumber_test_1) {
  NumberGeneratorSRand number_generator_srand(100);

  uint32_t number1 =  101;
  uint32_t number2 =  number_generator_srand.GetGeneratedNumber();
  ASSERT_GT(number1,number2);
}
/*
 * @brief  To check setMaxRandomValLimit by setting 
 *         max_random_value_ < min_random_value_
*/
TEST(NumberGeneratorSRand_test, GenerateNumber_test_2) {
  NumberGeneratorLCG number_generator_srand(1000);

  number_generator_srand.SetRandomValRange(500, 100);
  
  uint32_t expected1 =  99;
  uint32_t expected2 =  501;
  uint32_t actual =  number_generator_srand.GetGeneratedNumber();
  EXPECT_TRUE((expected1<actual) && (expected2>actual));
}
/*
 * @brief  
*/
TEST(NumberGeneratorSRand_test, GenerateNumber_test_3) {
  NumberGeneratorSRand number_generator_srand(100);

  std::string actual_string =  number_generator_srand.GetGeneratorName();
  std::string expected_string = "Default CPP SRand Generator";
  ASSERT_EQ(actual_string,expected_string);
}
  
/*
 * @brief  To check GetNumber of GeneratorNodeHandler
*/
/*TEST(GeneratorNodeHandler_test, GetNumber_test_1) {  
  GeneratorNodeHandler generator_node_handler;

  uint32_t number1 =  1001;
  uint32_t number2 =  generator_node_handler.GetNumber();
  
  ASSERT_GT(number1,number2);
}  */

/*
 * @brief  to check GetCommunicationFactory of GeneratorNodeHandler
*/
/*TEST(GeneratorNodeHandler_test, GetCommunicationFactory_test_1) {

  GeneratorNodeHandler generator_node_handler;
  
  CommFactory *comm_factory_obj = NULL;  
  comm_factory_obj =   generator_node_handler.GetCommunicationFactory();  
  
  EXPECT_TRUE(comm_factory_obj != NULL);
}
*/

/*
 * @brief  
*/
PublishSubscribe* publish_subcribe;

void receiveCallback(const ros_ran_num_msg::rand_num& msg) {
  std::cerr << "Received message using Subscribe.." << std::endl;
  
  delete publish_subcribe;
  ASSERT_NE(msg.number1,msg.number2);
}

void timer_lapsed(const ros::TimerEvent& evt) {
  std::cerr << "Time elapsed and no message received.." << std::endl;
  delete publish_subcribe;

  EXPECT_TRUE(0);
}

TEST(DISABLED_PublisherSuscribe_test, SendMessage_test_1) {
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("random_number_srand", 1000, receiveCallback);
  std::cerr << "Testing Send Message using Publish..." << std::endl;

  GeneratorNodeHandler generator_node_handler;
  publish_subcribe = new PublishSubscribe(&generator_node_handler);

  ros::Timer timer = nh.createTimer(ros::Duration(5), &timer_lapsed, false, true);

  publish_subcribe->SendMessage();
 
}

/**********************negative cases**********************************/

TEST(DISABLED_PublisherSuscribe_Negtest, SendMessage_test_1) {
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("Wrong_random_number", 1000, receiveCallback);
  std::cerr << "Testing Send Message using Publish..." << std::endl;

  GeneratorNodeHandler generator_node_handler;
  publish_subcribe = new PublishSubscribe(&generator_node_handler);

  ros::Timer timer = nh.createTimer(ros::Duration(5), &timer_lapsed, false, true);
 
  publish_subcribe->SendMessage();
 
}
/*
 * @brief  To check GetNumber - GenerateNumber generating random 
 *          number within given value
*/
TEST(NumberGeneratorLCG_Negtest, GenerateNumber_test_1) {
  NumberGeneratorLCG number_generator_lcg(100);

  uint32_t number1 =  101;
  uint32_t number2 =  number_generator_lcg.GetGeneratedNumber();
  EXPECT_GT(number1,number2);
}

/*
 * @brief  To check setMaxRandomValLimit is setting max_random_value
 *  & min_random_value with given value
*/
TEST(NumberGeneratorLCG_Negtest, GenerateNumber_test_2) {
  NumberGeneratorLCG number_generator_lcg(1000);

  number_generator_lcg.SetRandomValRange(0, 100);
  
  uint32_t number1 =  101;
  uint32_t number2 =  number_generator_lcg.GetGeneratedNumber();
  EXPECT_GT(number1,number2);
}
/*
 * @brief  To check setMaxRandomValLimit by setting 
 * max_random_value_ < min_random_value_
*/
TEST(NumberGeneratorLCG_Negtest, GenerateNumber_test_3) {
  NumberGeneratorLCG number_generator_lcg(1000);

  number_generator_lcg.SetRandomValRange(500, 100);
  
  uint32_t expected1 =  99;
  uint32_t expected2 =  501;
  uint32_t actual =  number_generator_lcg.GetGeneratedNumber();
  EXPECT_FALSE((expected1<actual) && (expected2>actual));
}
/*
 * @brief  
*/
TEST(NumberGeneratorLCG_Negtest, GenerateNumber_test_4) {
  NumberGeneratorLCG number_generator_lcg(100);

  std::string actual_string =  number_generator_lcg.GetGeneratorName();
  std::string expected_string = "Default CPP SRand Generator";
  EXPECT_EQ(actual_string,expected_string);
}

/*
 * @brief  To check GetNumber - GenerateNumber generating 
 * random number within given value
*/
TEST(NumberGeneratorSRand_Negtest, GenerateNumber_test_1) {
  NumberGeneratorSRand number_generator_srand(100);

  uint32_t number1 =  101;
  uint32_t number2 =  number_generator_srand.GetGeneratedNumber();
  EXPECT_GT(number1,number2);
}
/*
 * @brief  To check setMaxRandomValLimit by setting 
 * max_random_value_ < min_random_value_
*/
TEST(NumberGeneratorSRand_Negtest, GenerateNumber_test_2) {
  NumberGeneratorLCG number_generator_srand(1000);

  number_generator_srand.SetRandomValRange(500, 100);
  
  uint32_t expected1 =  99;
  uint32_t expected2 =  501;
  uint32_t actual =  number_generator_srand.GetGeneratedNumber();
  EXPECT_FALSE((expected1<actual) && (expected2>actual));
}
/*
 * @brief  
*/
TEST(NumberGeneratorSRand_Negtest, GenerateNumber_test_3) {
  NumberGeneratorSRand number_generator_srand(100);

  std::string actual_string =  number_generator_srand.GetGeneratorName();
  std::string expected_string = "Linear Congruential Generator";
  EXPECT_EQ(actual_string,expected_string);
}
 
/*
 * @brief  
*/
int main(int argc, char **argv) {
  
  ros::init(argc, argv, "NumberGeneratorNode_Test");
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
 
  return result;
}
