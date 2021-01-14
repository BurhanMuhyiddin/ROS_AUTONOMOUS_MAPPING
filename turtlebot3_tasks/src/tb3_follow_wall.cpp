#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>

#include <iostream>
#include <algorithm>

using namespace std;

class FollowWall
{
private:
  ros::Subscriber scan_subscriber;
  std_msgs::Float32MultiArray scan_data;

public:
  FollowWall(ros::NodeHandle *nh);
  void callback_scan_subs(const sensor_msgs::LaserScan& msg);
};

FollowWall::FollowWall(ros::NodeHandle *nh)
{
  this->scan_subscriber = nh->subscribe("/scan", 1000, &FollowWall::callback_scan_subs, this);
}

void FollowWall::callback_scan_subs(const sensor_msgs::LaserScan& msg)
{
  this->scan_data.data = msg.ranges;
  //int *min = min_element( begin(this->scan_data.data), end(this->scan_data.data) );
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wall_follower_node");
  ros::NodeHandle nh;
  FollowWall fw = FollowWall(&nh);
  ros::spin();
}
