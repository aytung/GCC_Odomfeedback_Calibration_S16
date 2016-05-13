#include "nav.h"
#include "ros/ros.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

  /* This is a visualization of what the x and y coordinates represent on 
   relative to the direction that the turtlebot is facing.
|         X+        . (destination)
|        |
|        |
|      (forward)
|        __
|      /   \
|      |___|    
|        ____________ Y-
  */


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  RoboState robot = RoboState(nh);
  geometry_msgs::Twist cmd_vel;
  
  ros::Rate loopRate(1); // 10 hz
  int count = 0; 
  
  while(ros::ok())
    {
      robot.goRobotGo();
      ros::spinOnce();
      loopRate.sleep();
      count++; 
    }

  return 0;
}

