#include <gtest/gtest.h>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"

#include "ros_number_generator/app/generator_node_handler.h"
#include "ros_number_generator/core/communication_factory.h"
#include "ros_number_generator/app/publish_subscribe.h"
#include "ros_ran_num_msg/rand_num.h"

//to check assignment of communication_interface_
/*TEST(CreateCommunicator_test, CreateCommunicator_test_1) {
  CommFactory comm_controller_factory_;

  comm_controller_factory_.CreateCommunicator(new PublishSubscribe(NULL));
  CommunicationInterface *comm_interface = comm_controller_factory_.GetCommunicator();
  
  EXPECT_TRUE(comm_interface != NULL);
}

//to check assignment of communication_interface_ with check to cover IF TRUE part
// IF ensures to delete previously allocated communication_interface_ to release memory
TEST(CreateCommunicator_test, CreateCommunicator_test_2) {
  CommFactory comm_controller_factory_;

  comm_controller_factory_.CreateCommunicator(new PublishSubscribe(NULL));
  comm_controller_factory_.CreateCommunicator(new PublishSubscribe(NULL));
  CommunicationInterface *comm_interface = comm_controller_factory_.GetCommunicator();
  
  EXPECT_TRUE(comm_interface != NULL);
}

//to check assignment of communication_interface_ to non-garbage when NULL is passed to CreateCommunicator
TEST(CreateCommunicator_test, CreateCommunicator_test_3) {
  CommFactory comm_controller_factory_;

  comm_controller_factory_.CreateCommunicator(NULL);
  CommunicationInterface *comm_interface = comm_controller_factory_.GetCommunicator();
  
  EXPECT_TRUE(comm_interface == NULL);
}

//To check communication_interface_ without CreateCommunicator
TEST(GetCommunicator_test, GetCommunicator_test_1) {
  CommFactory comm_controller_factory_;

  CommunicationInterface *comm_interface = comm_controller_factory_.GetCommunicator();
  
  EXPECT_TRUE(comm_interface == NULL);
}
*/
//To recreate number generator using alternate implementation
/*TEST(NumberGenerator_test, GenerateNumber_test_1) {
  NumberGeneratorFactory *number_generator = new NumberGeneratorFactory();
  number_generator->CreateGenerator(new NumberGeneratorLCG(0,1000));
  number_generator->CreateGenerator(new NumberGeneratorSRand(0,1000));

  uint32_t number1 =	1001;
  uint32_t number2 = number_generator->GetGenerator()->GetGeneratedNumber();	

  delete number_generator;

  ASSERT_GT(number1,number2);
}
*/
//To check GetNumber - GenerateNumber generating random number within given value
TEST(NumberGeneratorLCG_test, GenerateNumber_test_1) {
  NumberGeneratorLCG number_generator_lcg(100);

  uint32_t number1 =	101;
  uint32_t number2 =	number_generator_lcg.GetGeneratedNumber();
  ASSERT_GT(number1,number2);
}

//To check setMaxRandomValLimit is setting max_random_value & min_random_value with given value
TEST(NumberGeneratorLCG_test, GenerateNumber_test_2) {
  NumberGeneratorLCG number_generator_lcg(1000);

  number_generator_lcg.SetRandomValRange(0, 100);
	
  uint32_t number1 =	101;
  uint32_t number2 =	number_generator_lcg.GetGeneratedNumber();
  ASSERT_GT(number1,number2);
}
//To check setMaxRandomValLimit by setting max_random_value_ < min_random_value_
TEST(NumberGeneratorLCG_test, GenerateNumber_test_3) {
  NumberGeneratorLCG number_generator_lcg(1000);

  number_generator_lcg.SetRandomValRange(500, 100);
	
  uint32_t expected1 =	99;
  uint32_t expected2 =	501;
  uint32_t actual =	number_generator_lcg.GetGeneratedNumber();
  EXPECT_TRUE((expected1<actual) && (expected2>actual));
}

TEST(NumberGeneratorLCG_test, GenerateNumber_test_4) {
  NumberGeneratorLCG number_generator_lcg(100);

  std::string actual_string =	number_generator_lcg.GetGeneratorName();
  std::string expected_string = "Linear Congruential Generator";
  ASSERT_EQ(actual_string,expected_string);
}


//To check GetNumber - GenerateNumber generating random number within given value
TEST(NumberGeneratorSRand_test, GenerateNumber_test_1) {
  NumberGeneratorSRand number_generator_srand(100);

  uint32_t number1 =	101;
  uint32_t number2 =	number_generator_srand.GetGeneratedNumber();
  ASSERT_GT(number1,number2);
}
//To check setMaxRandomValLimit by setting max_random_value_ < min_random_value_
TEST(NumberGeneratorSRand_test, GenerateNumber_test_2) {
  NumberGeneratorLCG number_generator_srand(1000);

  number_generator_srand.SetRandomValRange(500, 100);
	
  uint32_t expected1 =	99;
  uint32_t expected2 =	501;
  uint32_t actual =	number_generator_srand.GetGeneratedNumber();
  EXPECT_TRUE((expected1<actual) && (expected2>actual));
}

TEST(NumberGeneratorSRand_test, GenerateNumber_test_3) {
  NumberGeneratorSRand number_generator_srand(100);

  std::string actual_string =	number_generator_srand.GetGeneratorName();
  std::string expected_string = "Default CPP SRand Generator";
  ASSERT_EQ(actual_string,expected_string);
}
  
//to check GetNumber of GeneratorNodeHandler
/*TEST(GeneratorNodeHandler_test, GetNumber_test_1) {  
  GeneratorNodeHandler generator_node_handler;

  uint32_t number1 =	1001;
  uint32_t number2 =	generator_node_handler.GetNumber();
	
  ASSERT_GT(number1,number2);
}  */

//to check GetCommunicationFactory of GeneratorNodeHandler
/*TEST(GeneratorNodeHandler_test, GetCommunicationFactory_test_1) {

  GeneratorNodeHandler generator_node_handler;
	
  CommFactory *comm_factory_obj = NULL;	
  comm_factory_obj = 	generator_node_handler.GetCommunicationFactory();	
  
  EXPECT_TRUE(comm_factory_obj != NULL);
}
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

/*TEST(PublisherSuscribe_test, SendMessage_test_1) {
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("random_number_srand", 1000, receiveCallback);
  std::cerr << "Testing Send Message using Publish..." << std::endl;

  GeneratorNodeHandler generator_node_handler;
  publish_subcribe = new PublishSubscribe(&generator_node_handler);

  ros::Timer timer = nh.createTimer(ros::Duration(5), &timer_lapsed, false, true);

  publish_subcribe->SendMessage();
 
}
*/
/**********************negative cases**********************************/


/*
TEST(PublisherSuscribe_Negtest, SendMessage_test_1) {
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("Wrong_random_number", 1000, receiveCallback);
  std::cerr << "Testing Send Message using Publish..." << std::endl;

  GeneratorNodeHandler generator_node_handler;
  publish_subcribe = new PublishSubscribe(&generator_node_handler);

  ros::Timer timer = nh.createTimer(ros::Duration(5), &timer_lapsed, false, true);
 
  publish_subcribe->SendMessage();
 
}
*/
//To recreate number generator using alternate implementation
/*TEST(NumberGenerator_Negtest, GenerateNumber_test_1) {
  NumberGeneratorFactory *number_generator = new NumberGeneratorFactory();
  number_generator->CreateGenerator(new NumberGeneratorLCG(0,1000));
  number_generator->CreateGenerator(new NumberGeneratorSRand(0,100));

  uint32_t number1 =	100;
  uint32_t number2 = number_generator->GetGenerator()->GetGeneratedNumber();	

  delete number_generator;

  ASSERT_LT(number1,number2);
}*/

//To check GetNumber - GenerateNumber generating random number within given value
TEST(NumberGeneratorLCG_Negtest, GenerateNumber_test_1) {
  NumberGeneratorLCG number_generator_lcg(100);

  uint32_t number1 =	101;
  uint32_t number2 =	number_generator_lcg.GetGeneratedNumber();
  ASSERT_LT(number1,number2);
}

//To check setMaxRandomValLimit is setting max_random_value & min_random_value with given value
TEST(NumberGeneratorLCG_Negtest, GenerateNumber_test_2) {
  NumberGeneratorLCG number_generator_lcg(1000);

  number_generator_lcg.SetRandomValRange(0, 100);
	
  uint32_t number1 =	101;
  uint32_t number2 =	number_generator_lcg.GetGeneratedNumber();
  ASSERT_LT(number1,number2);
}
//To check setMaxRandomValLimit by setting max_random_value_ < min_random_value_
TEST(NumberGeneratorLCG_Negtest, GenerateNumber_test_3) {
  NumberGeneratorLCG number_generator_lcg(1000);

  number_generator_lcg.SetRandomValRange(500, 100);
	
  uint32_t expected1 =	99;
  uint32_t expected2 =	501;
  uint32_t actual =	number_generator_lcg.GetGeneratedNumber();
  EXPECT_FALSE((expected1<actual) && (expected2>actual));
}

TEST(NumberGeneratorLCG_Negtest, GenerateNumber_test_4) {
  NumberGeneratorLCG number_generator_lcg(100);

  std::string actual_string =	number_generator_lcg.GetGeneratorName();
  std::string expected_string = "Default CPP SRand Generator";
  ASSERT_EQ(actual_string,expected_string);
}


//To check GetNumber - GenerateNumber generating random number within given value
TEST(NumberGeneratorSRand_Negtest, GenerateNumber_test_1) {
  NumberGeneratorSRand number_generator_srand(100);

  uint32_t number1 =	101;
  uint32_t number2 =	number_generator_srand.GetGeneratedNumber();
  ASSERT_LT(number1,number2);
}
//To check setMaxRandomValLimit by setting max_random_value_ < min_random_value_
TEST(NumberGeneratorSRand_Negtest, GenerateNumber_test_2) {
  NumberGeneratorLCG number_generator_srand(1000);

  number_generator_srand.SetRandomValRange(500, 100);
	
  uint32_t expected1 =	99;
  uint32_t expected2 =	501;
  uint32_t actual =	number_generator_srand.GetGeneratedNumber();
  EXPECT_FALSE((expected1<actual) && (expected2>actual));
}

TEST(NumberGeneratorSRand_Negtest, GenerateNumber_test_3) {
  NumberGeneratorSRand number_generator_srand(100);

  std::string actual_string =	number_generator_srand.GetGeneratorName();
  std::string expected_string = "Linear Congruential Generator";
  ASSERT_EQ(actual_string,expected_string);
}
  
//to check GetNumber of GeneratorNodeHandler
/*(TEST(GeneratorNodeHandler_Negtest, GetNumber_test_1) {  
  GeneratorNodeHandler generator_node_handler;

  uint32_t number1 =	1001;
  uint32_t number2 =	generator_node_handler.GetNumber();
	
  ASSERT_LT(number1,number2);
}  */

//to check GetCommunicationFactory of GeneratorNodeHandler
/*TEST(GeneratorNodeHandler_Negtest, GetCommunicationFactory_test_1) {

  GeneratorNodeHandler generator_node_handler;
	
  CommFactory *comm_factory_obj = generator_node_handler.GetCommunicationFactory();	
  
  EXPECT_TRUE(comm_factory_obj == NULL);
}
*/


int main(int argc, char **argv) {
  
  ros::init(argc, argv, "NumberGeneratorNode_Test");
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
 
  return result;
}