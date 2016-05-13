#ifndef NAV_CPP
#define NAV_CPP
#include <sample_pubsub/mymsg.h>
#include "nav.h"
#include "ros/ros.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

// how much we need in velocity commands to move incrementAmt forward
const double movementMultiple = 1.8; 
// how much we move forward/backward each increment
const double incrementAmt = .1;
// how much angular velocity we need to move right 90 degrees
const double left_90 = 2.54629;
// how much angular velocity we need to move right 90 degrees
const double right_90 = -2.56;
// this is how long the TurtleBot takes to move incrementAmt distance forward
const double movementInterval = 500000;
// how much we need to multiply radians by in order to get degrees
const double angleConvert = 57.2958;

const double yaw_180 = 3.1415;

// initializer for RoboState (with default values)
RoboState::RoboState(ros::NodeHandle rosNode): xCoord(0), yCoord(0), messageStatus(false), xOdomOld(0), yOdomOld(0), negXturn(false), initXneg(false), yaw(0), rotated(false), yawErr(.1), isXswapped(false), yawGoal(0)
{
  this->node = rosNode;
  // publishes velocity messages
  this->velocityPublisher = this->node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  // subscribes to messages from ui_node
  this->messageSubscriber= this->node.subscribe("my_msg", 1000, &RoboState::messageCallback, this);
  // subscribes to data from bumpers
  this->bumperSubscriber = this->node.subscribe("/mobile_base/sensors/core", 100, &RoboState::bumperCallback, this);
  // still need to find out how to subscribe to encoders
  this->odomSubscriber = this->node.subscribe("/odom", 100, &RoboState::odomCallback, this);


}

void RoboState::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  
  xCoord = msg->pose.pose.position.x;
  yCoord = msg->pose.pose.position.y;
  
tf::Quaternion q(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
tf::Matrix3x3 m(q);
 double roll, pitch, tempYaw;
m.getRPY(roll, pitch, tempYaw);
bool yawNeg = (yaw < 0);
 if (yawNeg=false){
   if(getY() < 0)
     setYaw(-2*yaw_180+tempYaw);
   else
     setYaw(tempYaw);
       }
 else{
   if(getY() > 0)
     setYaw(tempYaw);
   else
     setYaw(2*yaw_180+tempYaw);
 }
 ROS_INFO("The odometry was %f", getYaw()*57.2958);

}


// face the final destination, which is determined by yawGoal
void RoboState::faceDestination()
{

  double yawOffset = getYawGoal() - getYaw();
  if(yawOffset >= 1.5708)
    rotateLeft();
  else if(yawOffset <= -1.5708)
    rotateRight();
  if(yawOffset>= yawErr || yawOffset <= -yawErr){
    usleep(movementInterval);
    this->velocityCommand.linear.x = 0;
    this->velocityCommand.angular.z = yawOffset*57.88/90*left_90;
  }
  else
    setIsXswapped(true);
}

/*
void RoboState::turn_180()
{
  double yawOffset = getTempYaw() - getYaw();
  if(yawOffset >= 1.5708)
    rotateLeft();
  else if(yawOffset <= -1.5708)
    rotateRight();
  if(yawOffset>= yawErr || yawOffset <= -yawErr){
    usleep(movementInterval);
    this->velocityCommand.linear.x = 0.0;
    this->velocityCommand.angular.z = yawOffset*57.88/90*left_90;
  }
  else
    setTurnNegX(true);
}
*/


void RoboState::goRobotGo()
{
  if(isMessageSet()){

    if(getInitialXnegative()){
      if(!getTurnNegX()){
	//	turn_180();
      }
      else if(!getIsXswapped()){
	goForwardX();
      }
      else if(getIsXswapped()){
	goForwardY();
      }
    } // end if
    else{
      if(!getIsXswapped())
	goForwardX();
      else if(getIsXswapped())
	goForwardY();
    } // end else
  } // end if

} // end goRobotGo


  // assumes that x is zero and y is nonzero. May want to add cases for when method is called in error
void RoboState::determineYawGoal()
{
  ROS_INFO("Switching x and y coordinates.");
  usleep(500000);

    if( getY() > 0){ // means that destination is on right
      setYawGoal(1.5708);
    }
    else if ( getY() < 0){ // means that destination is on right
      setYawGoal(-1.5708);
    }
    else 
      setYawGoal(0); // means that y is zero

    if(getY() > 0){
      ROS_INFO("Your y-coordinate is positive.");
    }
    else if(getY() < 0){
      ROS_INFO("Your y-coordinate is negative.");
    }
    else{
      ROS_INFO("Your y-coordinate is zero.");
    }
}

void RoboState::goForwardX()
{

  if(getX() <= -incrementAmt || getX() >= incrementAmt )
    {
      double xMoveCommand; 
      usleep(movementInterval);
      xMoveCommand = incrementAmt*movementMultiple;
      this->velocityCommand.linear.x = xMoveCommand;
      this->velocityCommand.angular.z = 0.0;
      velocityPublisher.publish(this->velocityCommand);
      double currentXodom = getXodom();
      double amountMoved = currentXodom-getXodomOld();
      setX(getX()-amountMoved);
      setXodomOld(currentXodom);
      usleep(50000);
      ROS_INFO("We moved %f", amountMoved);
      ROS_INFO("The remaining amount to move is %f", getX());
    }
  else if(getX() <= getErr() && getX() >= -getErr()){
    faceDestination();
  }
  else{
    usleep(movementInterval);
    double xMoveCommand = abs(getX())*movementMultiple;
    this->velocityCommand.linear.x = xMoveCommand;
    this->velocityCommand.angular.z = 0.0;
    velocityPublisher.publish(this->velocityCommand);
    ROS_INFO("We moved forward %f", getX());
    double currentXodom = getXodom();
    double amountMoved = currentXodom-getXodomOld();
    setX(getX()-amountMoved);
    setXodomOld(currentXodom);
    ROS_INFO("The remaining amount to move is %f (should be around zero)", getX());
  }
}

  
	      
void RoboState::goForwardY()
{

  if(getY() <= -incrementAmt || getY() >= incrementAmt )
    {
      double xMoveCommand; 
      usleep(movementInterval);
      xMoveCommand = incrementAmt*movementMultiple;
      this->velocityCommand.linear.x = xMoveCommand;
      this->velocityCommand.angular.z = 0.0;
      velocityPublisher.publish(this->velocityCommand);
      double currentYodom = getYodom();
      double amountMoved = currentYodom-getYodomOld();
      setY(getY()-amountMoved);
      setYodomOld(currentYodom);
      usleep(50000);
      ROS_INFO("We moved %f", amountMoved);
      ROS_INFO("The remaining amount to move is %f", getY());
    }
  else if(getX() <= getErr() && getX() >= -getErr()){
    setMessageStatus(false);
  }
  else{
    
    double xMoveCommand = abs(getY())*movementMultiple;
    usleep(movementInterval);
    this->velocityCommand.linear.x = xMoveCommand;
    this->velocityCommand.angular.z = 0.0;
    velocityPublisher.publish(this->velocityCommand);
    double currentYodom = getXodom();
    double amountMoved = currentYodom-getYodomOld();
    setY(getY()-amountMoved);
    setYodomOld(currentYodom);
    ROS_INFO("The remaining amount to move is %f (should be around zero)", getY());
  }
}





// Change the value of Movement multiple until turtlebot moves forward by incrementAmt.
// And stops for movementInterval.
void RoboState::testForward()
{
  // may need to adjust value for whatever reason
  usleep(movementInterval);
  double xMoveCommand; 
  // only move forward incrementAmt if the amount left to move is greater than incrementAmt

  // ideally, this should result in forward (or backward movement)
  // in the x direction by incrementAmt

  this->velocityCommand.linear.x = .1;
  this->velocityCommand.angular.z = 0;
	  
  velocityPublisher.publish(this->velocityCommand);
  // ideally, this is the amount that x has changed
  // we should wait until forward movement has finished before we go on
  usleep(movementInterval);

}

void RoboState::messageCallback(const turtlebot::mymsg::ConstPtr& msg)
{
  //ROS_INFO("got packet: [%s]", msg->data.c_str());
  //ROS_INFO("got packet: ");
  if(!isMessageSet())
    {

      if(msg->x==0 && msg->y==0)
	ROS_INFO("No reason to move a distance of 0. Message not sent.");
      else{	  
	ROS_INFO("X and Y coordinates sent were: x:%f y:%f", msg->x, msg->y);
	setX(msg->x);
	setY(msg->y);
	ROS_INFO("xCoord is: %f. yCoord is: %f", getX(), getY());
	setMessageStatus(true);
	setIsXswapped(false);
	if(getX() >= 0){
	  setInitialXnegative(false);
	  setTurnNegX(false);
	}
	else{
	  setInitialXnegative(true);
	  setTurnNegX(false);
	}
	//setErr(sqrt(pow(getX(),2)+pow(getY(),2))*.1);
	setErr(.1);
	setRotated(false);
      }
      determineYawGoal();

    }
  else
    ROS_INFO("Cannot accept message. Movement still in progress.");

}

void RoboState::bumperCallback(const create_node::TurtlebotSensorState::ConstPtr& msg)
{
  // if bumpers don't complain, don't run the loop
  if(msg->bumps_wheeldrops != 0){
    ROS_INFO("You hit an object! Motion terminating.");
    ROS_INFO("The remaining x was:%f and the remaining y was: %f.", getX(), getY());
    setX(0);
    setY(0);
    // allow RoboState to receive messages again
    setMessageStatus(false);
  }
}

  

  
double RoboState::xIsNegative()
{

  if( getX() >= 0)
    return 1;
  else
    return -1;
}


double RoboState::yIsNegative()
{

  if( getY() >= 0)
    return 1;
  else
    return -1;
}

  
void RoboState::rotateLeft()
{
  ROS_INFO("Rotating left.");
  usleep(500000);
  this->velocityCommand.linear.x = 0.0;
  this->velocityCommand.angular.z = left_90;
    
	
  velocityPublisher.publish(this->velocityCommand);
  usleep(100000);
}

void RoboState::rotateRight()
{
  ROS_INFO("Rotating right.");
  usleep(500000);
  this->velocityCommand.linear.x = 0.0;
  this->velocityCommand.angular.z = right_90;
  velocityPublisher.publish(this->velocityCommand);
  usleep(100000);
}


void RoboState::setMessageStatus(bool status)
{
  messageStatus=status;
}

bool RoboState::isMessageSet()
{
  return messageStatus;
}
  
void RoboState::incrementX(double x)
{
  xCoord += x;
}

void RoboState::incrementY(double y)
{
  yCoord += y;
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


bool RoboState::getIsXswapped()
{
  return isXswapped;
}

void RoboState::setIsXswapped(bool xSwapValue)
{
  isXswapped = xSwapValue;
}

void RoboState::setInitialXnegative(bool initialXstatus)
{
  initXneg = initialXstatus;
}

bool RoboState::getInitialXnegative()
{
  return initXneg;
}

bool RoboState::getTurnNegX()
{
  return negXturn;
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

void RoboState::setTurnNegX(bool xTurnStatus)
{
  negXturn = xTurnStatus;
}



void RoboState::setRotated(bool newRotated)
{
  rotated = newRotated;
}
bool RoboState::getRotated()
{
  return rotated;
}

/*
  void RoboState::xOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
  {
  setXodom(odom->pose.pose.position.x);
  }
*/


/*
  void RoboState::yOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
  {
  setYodom(odom->pose.pose.position.y);
  }
*/



/*
  void RoboState::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
  {
  setXodom(odom->pose.pose.position.x);

  setYodom(odom->pose.pose.position.y);
  }
*/

/*
  void RoboState::yOdomCallback(const nav_msgs::Odometry::ConstPtr& yOdom)
  {


  }

  void RoboState::xOdomCallback(const nav_msgs::Odometry::ConstPtr& xOdom)
  {
  setOdom

  }
  
*/

/*
  void RoboState::goForwardXneg()
  {
  // may need to adjust value for whatever reason

  //    usleep(100000);

  // amount left to move is greater than or equal to incrementAmt, so we move forward incrementAmt
  if(getX() <= -incrementAmt || getX() >= incrementAmt )
  {
	
  double xMoveCommand; 
	
  // only move forward incrementAmt if the amount left to move is greater than incrementAmt
  //	if (std::abs(getX()) > incrementAmt){
  // ideally, this should result in forward (or backward movement)
  usleep(movementInterval);
  // in the x direction by incrementAmt
	  
  xMoveCommand = incrementAmt*movementMultiple*xIsNegative();
  this->velocityCommand.linear.x = xMoveCommand;
  this->velocityCommand.angular.z = 0.0;
	  
  velocityPublisher.publish(this->velocityCommand);
  //ROS_INFO("The amount we published was %f", xMoveCommand);
  // ideally, this is the amount that x has changed
	  
  double currentXodom = getXodom();
  double amountMoved = currentXodom-getXodomOld();
  setX(getX()-amountMoved);
  setXodomOld(currentXodom);
	  
  // we should wait until forward movement has finished before we go on
  usleep(50000);

  ROS_INFO("We moved %f", amountMoved);
  ROS_INFO("The remaining amount to move is %f", getX());
  }
  else if(getX() <= getErr() && getX() >= -getErr()){
  rotateLR();
  setIsXswapped(true);
  }
  else{
  // we have less than the incrementAmt left, so we move however much remaining
  // do not need to know if xCoord is negative or not, since we get the actual value
  double xMoveCommand = getX()*movementMultiple;
  this->velocityCommand.linear.x = xMoveCommand;
  this->velocityCommand.angular.z = 0.0;

  velocityPublisher.publish(this->velocityCommand);
  ROS_INFO("We moved forward %f", getX());
  // assume we moved forward or backward in the exactly how much was left in xCoord

  double currentXodom = getXodom();
  double amountMoved = currentXodom-getXodomOld();
  setX(getX()-amountMoved);
  setXodomOld(currentXodom);

  ROS_INFO("The remaining amount to move is %f (should be zero)", getX());
  }

  }

  void RoboState::goForwardYneg()
  {

  if(getY() <= -incrementAmt || getY() >= incrementAmt )
  {
	
  double xMoveCommand; 
	
  // only move forward incrementAmt if the amount left to move is greater than incrementAmt
  //if (std::abs(getX()) > incrementAmt){
  // ideally, this should result in forward (or backward movement)
  usleep(movementInterval);
  // in the x direction by incrementAmt
	  
  xMoveCommand = incrementAmt*movementMultiple*yIsNegative();
  this->velocityCommand.linear.x = xMoveCommand;
  this->velocityCommand.angular.z = 0.0;
	  
  velocityPublisher.publish(this->velocityCommand);

  // ideally, this is the amount that x has changed
  double currentYodom = getYodom();
  double amountMoved = currentYodom-getYodomOld();
  setY(getY()-amountMoved);
  setYodomOld(currentYodom);

  // we should wait until forward movement has finished before we go on
  usleep(50000);

  ROS_INFO("We moved %f", amountMoved);
  ROS_INFO("The remaining amount to move is %f", getY());
  }
  else if(getX() <= getErr() && getX() >= -getErr()){
	
  setMessageStatus(false);

  }

  else{
  // we have less than the incrementAmt left, so we move however much remaining
  // do not need to know if xCoord is negative or not, since we get the actual value
	  
  double xMoveCommand = getY()*movementMultiple;
  this->velocityCommand.linear.x = xMoveCommand;
  this->velocityCommand.angular.z = 0.0;

  velocityPublisher.publish(this->velocityCommand);
  ROS_INFO("We moved forward %f", getY());

  // assume we moved forward or backward in the exactly how much was left in xCoord
  double currentYodom = getXodom();
  double amountMoved = currentYodom-getYodomOld();
  setY(getY()-amountMoved);
  setYodomOld(currentYodom);

  ROS_INFO("The remaining amount to move is %f (should be around zero)", getY());
  }
  }



  double detAngle(double zQuat, double wQuat)
  {
  double theta_z = 2*asin(zQuat)*angleConvert;
  double theta_w = 2*acos(wQuat)*angleConvert;
  if(theta_w <= 180){
  return theta_z;
  }
  else{
  return -theta_z;
  }
  }


  // useless odometry
  // this->odomSubscriber = this->node.subscribe("/odom", 100, &RoboState::odomCallback, this);
  //    odomSubscriber = node.subscribe<nav_msgs::Odometry>("/odom", 100, &RoboState::odomCallback);
  //                this->xOdomSubscriber = this->node.subscribe("/odom/pose/pose/position/x", 100, &RoboState::xOdomCallback, this);
  //    this->sub = this-> node.subscribe("odom", 1000, &RoboState::chatterCallback, this);
  //  this->yOdomSubscriber = this->node.subscribe("/odom/pose/pose/position/y", 100, &RoboState::yOdomCallback, this);
  //odomSubscriber = this->subscribe("odom", 1000, RoboState::odomCallback);

  */
#endif
