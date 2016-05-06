#include "nav.h"
#include "ros/ros.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>

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
double xOdom=0;
double yOdom=0;
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  xOdom = msg->pose.pose.position.x;
  yOdom = msg->pose.pose.position.y;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  RoboState robot = RoboState(nh);
  //ros::Subscriber sub = nh.subscribe("my_msg", 1000, messageCallback);
  geometry_msgs::Twist cmd_vel;
  ros::Subscriber sub = nh.subscribe("odom", 1000, chatterCallback);
  
  ros::Rate loopRate(1); // 10 hz
  int count = 0; 
  
  /* while(ros::ok())
    {
	
      robot.setXodom(xOdom);
      robot.setYodom(yOdom);
  */
      // turnThenForward go is invoked when we want TurtleBot to turn in direction of destination
      // then go forward
      //      robot.testForward();
      //robot.goRobotGo();
  
      /*
	if(count < 3)
	  robot.testForward();

      */
      // goRobotGo is invoked when we want to move forward, xCoord amount, then
      // rotate towards destination and then move forward
	//	if(count==0)
	//	  robot.rotateRight();
		//	count++;
	//robot.goRobotGo();

    robot.turn_180();
    ros::spinOnce();
    loopRate.sleep();

    //    }  

  return 0;
}




