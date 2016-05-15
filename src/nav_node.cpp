#include "nav.h"
#include "ros/ros.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#define DEBUG 1
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
  
  ros::Rate loopRate(.5); // 10 hz
  int count = 0; 
  
  while(ros::ok())
    {
      
#if DEBUG
      if(!robot.getTurnNegX()){
      robot.turn_180();
      }
#endif

#if !DEBUG
      if(isMessageSet()){
	
	if(getInitialXnegative()){
	  if(!getTurnNegX()){
	    ROS_INFO("Turning 180 degrees.");
	    turn_180();
	  }
	  else{
	    if(!getRotated()){
	    ROS_INFO("Moving forward in -X.");
	    goForwardX();
	  }
	    else{
	    ROS_INFO("Moving forward in Y.");
	    goForwardY();
	  }
	  }
	} // end if
	else{
	  if(!getRotated()){
	    ROS_INFO("Moving forward in X.");
	    goForwardX();
	  }
	  else{
	    ROS_INFO("Moving forward in Y.");
	    goForwardY();
	  }
	} // end else
      } // end if
#endif
	//      robot.goRobotGo();
      loopRate.sleep();
      ros::spinOnce();
     
    }

  return 0;
}


