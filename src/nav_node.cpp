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


double xOdom=0;
double yOdom=0;
/*
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  xOdom = msg->pose.pose.position.x;
  yOdom = msg->pose.pose.position.y;
}
*/

/*
bool rotated = false;
double xOdom=0;
double yOdom=0;
double returnAngle=0;
double returnYaw=0;

ros::Publisher velocityPublisher = nh.advertise <geometry_msgs::Twist>"/mobile_base/commands/velocity", 1);


  void rotateLeft()
  {
    ROS_INFO("Rotating left.");
    usleep(500000);
    velocityPublisher.linear.x = 0.0;
    velocityPublisher.angular.z = left_90;
	
    velocityPublisher.publish(this->velocityCommand);
    usleep(100000);
  }

  void rotateRight()
  {
    ROS_INFO("Rotating right.");
    usleep(500000);
    velocityPublisher.linear.x = 0.0;
    velocityPublisher.angular.z = right_90;
    velocityPublisher.publish(this->velocityCommand);
    usleep(100000);
  }


void rotate(yawOffset)
{
  if(yawOffset >= 1.5708)
    rotateLeft();
  else if(yawOffset <= -1.5708)
    rotateRight();
  if(yawOffset>= yawErr || yawOffset <= -yawErr){
    velocityPublisher.linear.x = 0.0;
    velocityPublisher.angular.z = yawOffset*57.88/90*left_180;
  }
  else
    setRotated(true);
  }

}

*/

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  
  double zQuat = 0;
  double wQuat = 0;
  xOdom = msg->pose.pose.position.x;
  yOdom = msg->pose.pose.position.y;
  //  zQuat = msg->pose.pose.position.z;
  //  wQuat = msg->pose.pose.orientation;
  //  returnAngle = detAngle(zQuat, wQuat);
  /*  
tf::Quaternion q(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
tf::Matrix3x3 m(q);
double roll, pitch, yaw;
m.getRPY(roll, pitch, yaw);
 if (yawNeg=false){
   if(returnYaw < 0)
     returnYaw = 3.1416 - yaw;
   else
     returnYaw = yaw;
       }
 else{
   if(returnYaw > 0)
     returnYaw = -3.1416 - yaw;
   else
     returnYaw = yaw;
 }
  */
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  RoboState robot = RoboState(nh);
  //  ros::Subscriber sub = nh.subscribe("my_msg", 1000, messageCallback);
  geometry_msgs::Twist cmd_vel;
  //  ros::Subscriber sub = nh.subscribe("odom", 1000, chatterCallback);
  
  ros::Rate loopRate(1); // 10 hz
  int count = 0; 
  
  while(ros::ok()) //&& rotated==false)
    {
      //    yawOffset = goalYaw - returnYaw;
      //      rotate(yawOffset);
            robot.setXodom(xOdom);
            robot.setYodom(yOdom);
	    //            robot.setYaw(returnYaw);
      //robot.turn_180();
      robot.goRobotGo();
    ros::spinOnce();
    loopRate.sleep();
    count++;
  }

    //    }  

  return 0;
}




