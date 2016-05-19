#ifndef NAV_H
#define NAV_H
// ROS includes.
#include "ros/ros.h"
#include <nav_simple/mymsg.h>
#include <iostream>
//#include "node_example/listener.h"
#include "geometry_msgs/Twist.h"
#include "create_node/TurtlebotSensorState.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>

using namespace std;

enum State {NEUTRAL, TURN_LEFT_90};
    
class RoboState
  {
  public:

    // test function
    void testForward();
    void goRobotGo();
    // constructor
    RoboState(ros::NodeHandle rosNode);
    double getYaw();
    State getCurrentState();
    void faceDestination();

    void rotateLeft_90();
    void rotateRight_90();
    
  private:
    // the ros node being used by RoboState
    ros::NodeHandle node;    

    // publishers and subscribers 
    geometry_msgs::Twist velocityCommand;
    ros::Publisher velocityPublisher;
    geometry_msgs::Twist command;
    ros::Subscriber messageSubscriber;
    ros::Subscriber bumperSubscriber;

    State currentState;
    // various callback functions
    void bumperCallback(const create_node::TurtlebotSensorState::ConstPtr& msg);
    void messageCallback(const nav_simple::mymsg::ConstPtr& msg);
    void setMessageStatus(bool status);    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    ros::Subscriber odomSubscriber;
    int count;
    int internalCount;
    void determineYawGoal();

    // private variables
    double xTarget;
    double yTarget;
    double xOdom;
    double yOdom;
    double xOdomOld;
    double yOdomOld;
    double xCoord;
    double yCoord;
    double acceptErr;
    double yawErr;
    double yaw;
    double yawGoal;

    // get and set functions
    void setCurrentState(State newState);
    void setXodom(double xOdom);
    void setYaw(double newYaw);
    void setYodom(double yOdom);
    double getYawGoal();
    void setErr(double err);
    double getErr();
    double getXodom();
    double getYodom();
    double getXodomOld();
    double getYodomOld();
    void setXodomOld(double xOdomCurrent);
    void setYodomOld(double yOdomCurrent);
    void setYawGoal(double newYawGoal);
    double getX();
    void setX(double x);
    void setY(double y);
    double getY();
  };
#include "nav.cpp"
#endif // NAV_NODE_H
