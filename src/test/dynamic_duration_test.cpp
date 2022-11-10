/**
 * @file dynamic_duration_test.cpp
 * @author Robert Valner
 * @brief
 * @version 0.1
 * @date 2022-11-01
 *
 * @copyright Copyright 2022 Robert Valner
 *
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "ros_ci_test/SetDuration.h"
#include "std_msgs/String.h"

double avg_duration{ 0 };
double new_duration{ 0.05 };
double rate_before;
ros::Subscriber rate_sub;

/*
 * Callback for measuring the duration of string messages
 */
void stringMsgCb(const std_msgs::String& msg)
{
  auto current_time = ros::Time::now();
  static auto previous_time = current_time;

  ros::Duration dt = current_time - previous_time;
  avg_duration = 0.7 * avg_duration + 0.3 * dt.toSec();

  previous_time = current_time;
}

/*
 * Tests whether the service is working properly
 */
TEST(TestSuite, service_call_test)
{
  // First, measure current publishing rate
  ros::Duration(4).sleep();
  rate_before = 1.0 / avg_duration;

  // Set up the service client for invoking the set_duration service
  ros::NodeHandle nh;
  ros::ServiceClient scl = nh.serviceClient<ros_ci_test::SetDuration>("set_duration");

  // Compose the service message
  ros_ci_test::SetDuration srv_msg;
  srv_msg.request.duration = new_duration;

  // The result we are expecting to receive from the service call
  double expected_result = new_duration;

  // Invoke the service call
  if (scl.call(srv_msg))
  {
    // Check if the response is equal to the expected result
    EXPECT_EQ(srv_msg.response.duration, expected_result + 10)
        << "Expected " << expected_result << " but got " << srv_msg.response.duration;
  }
  else
  {
    // Assert false if the client was not able to reach the server
    ASSERT_TRUE(false) << "Could not reach the 'set_duration' server";
  }
}

/*
 * Tests if the sleep duration has changed
 */
TEST(TestSuite, dynamic_rate_test)
{
  // Measure current publishing rate
  ros::Duration(4).sleep();
  double rate_after = 1.0 / avg_duration;
  double expected_rate = 1.0 / new_duration;

  std::cerr << "rate_before: " << rate_before << " Hz, expected_rate: " << expected_rate
            << " Hz, current_rate: " << rate_after << std::endl;

  EXPECT_TRUE((rate_after <= expected_rate + 1) && (rate_after >= expected_rate - 1))
      << "Expected " << expected_rate << " Hz but got " << rate_after << " Hz";
}

/*
 * Main
 */
int main(int argc, char** argv)
{
  // Initialize the gtes
  testing::InitGoogleTest(&argc, argv);

  // Set up the test node
  ros::init(argc, argv, "DynamicRateChangeTest");
  ros::NodeHandle nh;
  rate_sub = nh.subscribe("my_topic", 10, stringMsgCb);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Run the tests
  return RUN_ALL_TESTS();
}
