#include <gtest/gtest.h>

#include "ros/ros.h"

#include "ros_arithmetic/app/multiplier_node_handler.h"

TEST(MYTEST , TESTVAL)
{
 int val1 = 10;
 int val2 = 10;
 
 EXPECT_EQ(val1,val2);
}


TEST(MYTEST , TESTVALFAIL)
{
 int val1 = 10;
 int val2 = 20;
 
 EXPECT_NE(val1,val2);
}

int main(int argc, char **argv) {
  
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
 
  return result;
}
