#ifndef NAV_CPP
#define NAV_CPP
#include <nav_simple/mymsg.h>
#include "nav.h"
#include "ros/ros.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#define DEBUG 0
// the least amount the turtlebot can move, otherwise odometry breaks;
// serves as our acceptable error
const double ANGLE_ERR = 15;
// how much we need in velocity commands to move INCREMENT_AMT forward
const double MOVEMENT_MULTIPLE = 1.8; 
// how much we move forward/backward each increment
const double INCREMENT_AMT = .1;
// how much angular velocity we need to move right 90 degrees
const double LEFT_90 = 2.54629;
// how much angular velocity we need to move right 90 degrees
const double RIGHT_90 = -2.56;
// this is how long the TurtleBot takes to move INCREMENT_AMT distance forward
const double MOVEMENT_INTERVAL = 500000;
// how much we need to multiply radians by in order to get degrees
const double ANGLE_CONVERT = 57.2958;
// approximately Pi, used to indicate around 180 degrees (must be under pi, incase of overshooting
const double YAW_180 = 3.1415;

const double MIN_LEFT = .4;

const double MIN_RIGHT = -.4;

// initializer for RoboState::State (with default values)
RoboState::RoboState(ros::NodeHandle rosNode): xCoord(0), yCoord(0), xOdomOld(0), yOdomOld(0), yaw(0), yawGoal(0), count(0), currentState(NEUTRAL), internalCount(0)
{
  // declare which ROS node we use
  this->node = rosNode;
  // publishes velocity messages
  this->velocityPublisher = this->node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  // subscribes to messages from ui_node
  this->messageSubscriber= this->node.subscribe("my_msg", 1000, &RoboState::messageCallback, this);
  // subscribes to data from bumpers
  this->bumperSubscriber = this->node.subscribe("/mobile_base/sensors/core", 100, &RoboState::bumperCallback, this);
  // subscribes to encoders
  this->odomSubscriber = this->node.subscribe("/odom", 100, &RoboState::odomCallback, this);
}

void RoboState::faceDestination()
{
  ROS_INFO("Calling face destination.");
  
  // how much we are off the goal
  double yawOffset = getYawGoal() - getYaw();

  // means that we are off by more than 90 degrees
  if(yawOffset > 90) {
    ROS_INFO("We are turning left because we were off by %f)", yawOffset);
    ROS_INFO("(Should be more than 90 degrees)");
    rotateLeft_90();
    ROS_INFO("Assume that we turned 90 degrees to the leftproperly.");
  }
  // means that we are off by less than -90 degrees
  else if(yawOffset <  -90){
    ROS_INFO("We are turning right because yawOffset was %f", yawOffset);
    ROS_INFO("(Should be less than -90 degrees)");
    rotateRight_90();
  }
  // means that we are off by between 90 and -90 degrees
  else if(yawOffset >= ANGLE_ERR || yawOffset <= -ANGLE_ERR){
    ROS_INFO("We are turning less because yawOffset was %f", yawOffset);
    ROS_INFO("(Should be between 90 and -90 degrees)");
    usleep(MOVEMENT_INTERVAL);
    this->velocityCommand.linear.x = 0;
    if(yawOffset >= 0){
      this->velocityCommand.angular.z = MIN_LEFT;
    }
    else
      this->velocityCommand.angular.z = MIN_RIGHT;
    velocityPublisher.publish(this->velocityCommand);
  }
  else{
    setCurrentState(NEUTRAL);
  }
}

// we use this because it has been calibrated to turn 90 degrees left
void RoboState::rotateLeft_90()
{
  ROS_INFO("Rotating left.");
  usleep(MOVEMENT_INTERVAL);
  this->velocityCommand.linear.x = 0.0;
  this->velocityCommand.angular.z = LEFT_90;
  velocityPublisher.publish(this->velocityCommand);
  usleep(MOVEMENT_INTERVAL/5);
}

// we use this because it has been calibrated to turn 90 degrees right
void RoboState::rotateRight_90()
{
  ROS_INFO("Rotating right.");
  usleep(MOVEMENT_INTERVAL);
  this->velocityCommand.linear.x = 0.0;
  this->velocityCommand.angular.z = RIGHT_90;
  velocityPublisher.publish(this->velocityCommand);
  usleep(MOVEMENT_INTERVAL/5);
}

void RoboState::messageCallback(const nav_simple::mymsg::ConstPtr& msg)
{
  // only accept message if movement is not in progress
  if(getCurrentState()==NEUTRAL)
    {
      if(msg->x==0 && msg->y==0)
	ROS_INFO("No reason to move a distance of 0. Message not sent.");
      else{	  
	ROS_INFO("X and Y coordinates sent were: x:%f y:%f", msg->x, msg->y);
	setX(msg->x);
	setY(msg->y);
	ROS_INFO("xCoord is: %f. yCoord is: %f", getX(), getY());
	
	// we don't need to face backward since initial movement is forward
	if(getX() >= 0){
	  setCurrentState(TURN_LEFT_90);
	}
	// need to face backward since initial movement is backward
	// (want bumper sensors to be useful)
       	else{
	  //setCurrentState(TURN_NEG_X);
	  setCurrentState(TURN_LEFT_90);
	}
	//setErr(sqrt(pow(getX(),2)+pow(getY(),2))*.1);
	setErr(.1);
      }
      // need to determine what direction we will ultimately face
      setYawGoal(90);
      //      determineYawGoal();

    }
  else
    ROS_INFO("Cannot accept message. Movement still in progress.");
}


void RoboState::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  
  // set odometry
  setXodom(msg->pose.pose.position.x);
  setYodom(msg->pose.pose.position.y);
  
  // convert from quaternions to angles
  tf::Quaternion q(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, tempYaw;
  m.getRPY(roll, pitch, tempYaw);
  
  tempYaw*=ANGLE_CONVERT;
  // determine whether to send positive or negative angles
  bool isTempYawNeg = (tempYaw < 0);
  
  // we always want a positive yaw value (for consistency)
  if(getY() >= 0){
    if(isTempYawNeg)
      setYaw(360+tempYaw);
    else
      setYaw(tempYaw);
  }
  // we always want a negative yaw value (for consistency)
  else{
    if(isTempYawNeg)
      setYaw(tempYaw);
    else
      setYaw(-360+tempYaw);
  }
  //spams you
  /*
  // we do not want to get spammed with messages about yaw.
    ROS_INFO("The yaw was %f", getYaw());

    if(getY() >= 0)
      ROS_INFO("The yaw is supposed to be positive.");
    else
      ROS_INFO("The yaw is supposed to be negative.");
}
  */
}


void RoboState::bumperCallback(const create_node::TurtlebotSensorState::ConstPtr& msg)
{
  // if bumpers don't complain, don't run the loop
  if(msg->bumps_wheeldrops != 0){
    ROS_INFO("You hit an object! Motion terminating.");
    ROS_INFO("The remaining x was:%f and the remaining y was: %f.", getX(), getY());
    setX(0);
    setY(0);
    // allow RoboState::State to receive messages again
    setCurrentState(NEUTRAL);
  }
}

/* 

   Get and set functions

*/

void RoboState::setCurrentState(State newState)
{
  currentState = newState;
}

State RoboState::getCurrentState()
{
  return currentState;
}

double RoboState::getX()
{
  return xCoord;
}

void RoboState::setX(double x)
{
  xCoord=x;
}

void RoboState::setY(double y)
{
  yCoord=y;
}

double RoboState::getY()
{
  return yCoord;
}

double RoboState::getXodom()
{
  return xOdom;
}

double RoboState::getYodom()
{
  return yOdom;
}

void RoboState::setXodom(double xOdomCurrent)
{
  xOdom = xOdomCurrent;
}

void RoboState::setYodom(double yOdomCurrent)
{
  yOdom = yOdomCurrent;
}

double RoboState::getYodomOld()
{
  return yOdomOld;
}

double RoboState::getXodomOld()
{
  return xOdomOld;
}

void RoboState::setYodomOld(double yOdomCurrent)
{
  yOdomOld = yOdomCurrent;
}

void RoboState::setXodomOld(double xOdomCurrent)
{
  xOdomOld = xOdomCurrent;
}

double RoboState::getErr()
{
  return acceptErr;
}

void RoboState::setErr(double err)
{
  acceptErr = err;
}

void RoboState::setYaw(double newYaw)
{
  yaw = newYaw;
}

double RoboState::getYaw()
{
  return yaw;
}

void RoboState::setYawGoal(double newYawGoal)
{
  yawGoal = newYawGoal;
}

double RoboState::getYawGoal()
{
  return yawGoal;
}

#endif

