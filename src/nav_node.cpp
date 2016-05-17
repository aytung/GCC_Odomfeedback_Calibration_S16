#include "nav.h"
#include "ros/ros.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

// debug is 1 when debugging, 0 when not

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
     
      // use this code only when DEBUG is true; use for testing
#if DEBUG

      char movement;
      double velocity;
      ROS_INFO("Enter 'l' for left 'r' for right and something else for yaw");
      cin >> movement;
	switch (movement){
	case 'l': 
	  ROS_INFO("Enter a velocity to use.");
	  cin >> velocity;
	  robot.rotateLeft(velocity);

	  break;
	case 'r':
	  ROS_INFO("Enter a velocity to use.");
	  cin >> velocity;
	  robot.rotateRight(velocity);

	  break;
	default:
	  cout << "The yaw was " << robot.getYaw();

	}

      robot.incrementInternalCount();
      //      count++;
     pp
      /*      if(!robot.getTurnNegX()){
	robot.turn_180();
      }
      */
#endif

      // use this code only when DEBUG is false; default state
#if !DEBUG
	robot.incrementInternalCount();

	switch (robot.getCurrentState()){
	case NEUTRAL:
	  ROS_INFO("Our yaw was %f", robot.getYaw());
	// do nothing
	break;
	case TURN_NEG_X:
	// ROS has difficulty with updating yaw
	  //	  if(robot.getInternalCount() % 10 == 5){
	ROS_INFO("Rotating to face backwards.");
	robot.rotate_180();
	//	}
	break;
	ROS_INFO("Moving forward in x coordinate.");
	case MOVE_FORWARD_X:
	robot.goForwardX();
	break;
	case FACE_DESTINATION:
	// ROS has difficulty with updating yaw
	  //	  if(robot.getInternalCount() % 10 == 5){
	ROS_INFO("Facing final destination.");
	robot.faceDestination();
	//	}
	break;
	case MOVE_FORWARD_Y:
	ROS_INFO("Moving forward in y coordinate.");
	robot.goForwardY();
	break;
	default:
	ROS_INFO("Something is broken. We should not reach this point.");
	}
#endif
	
	//robot.goRobotGo();
      loopRate.sleep();
      ros::spinOnce();
     
    }
	
  return 0;
}


