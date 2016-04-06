#include <gtest/gtest.h>
#include "ros/ros.h"


TEST(WatchdogServices, enable){

   int exist = ros::service::waitForService("/laser_watchdog/enable", ros::Duration(1));
   EXPECT_TRUE(exist);
}


int main(int argc, char ** argv){

  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_topic");
  ros::NodeHandle nh;

  int ret = RUN_ALL_TESTS();

  return ret;

}
