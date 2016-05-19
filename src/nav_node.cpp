#include "nav.h"
#include "ros/ros.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  RoboState robot = RoboState(nh);
  geometry_msgs::Twist cmd_vel;
  
  ros::Rate loopRate(.5); // 10 hz

  while(ros::ok())
    {
      switch (robot.getCurrentState()){
      case NEUTRAL:
	ROS_INFO("Nothing to do.");
	break;
      default:
	robot.faceDestination();
      }
      loopRate.sleep();
      ros::spinOnce();
     
    }
	
  return 0;
}


