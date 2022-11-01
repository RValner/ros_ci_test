/**
 * @file dummy_pub_srv.cpp
 * @author Robert Valner
 * @brief 
 * @version 0.1
 * @date 2022-11-01
 * 
 * @copyright Copyright 2022 Robert Valner
 * 
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_ci_test/SetDuration.h"
#include <mutex> // NOLINT [build/c++11]

double sleep_duration{1.0};
std::mutex sleep_duration_mutex;

/**
 * @brief Service callback for setting the sleep duration
 * 
 * @param req 
 * @param res 
 * @return true 
 * @return false 
 */
bool serviceCb(ros_ci_test::SetDuration::Request& req, ros_ci_test::SetDuration::Response& res)
{
  std::lock_guard<std::mutex> l(sleep_duration_mutex);
  sleep_duration = req.duration;
  res.duration = sleep_duration;
  ROS_INFO_STREAM("set sleep duration to: " << sleep_duration << " s"); 
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_srv_test");
  ros::NodeHandle nh;

  if (!ros::param::get("sleep_duration", sleep_duration))
  {
    ROS_WARN_STREAM("'sleep_duration' param not set, default = " << sleep_duration << " s");
  }

  ros::Publisher pub = nh.advertise<std_msgs::String>("my_topic", 10);
  std_msgs::String string_msg;

  ros::ServiceServer srv = nh.advertiseService("set_duration", serviceCb);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (ros::ok())
  {
    static unsigned int i{0};
    string_msg.data = "Hi, " + std::to_string(i++);
    ROS_INFO_STREAM("sending: " << string_msg.data);
    pub.publish(string_msg);

    std::lock_guard<std::mutex> l(sleep_duration_mutex);
    ros::Duration(sleep_duration).sleep();
  }

  return 0;
}
