#ifndef NAV_H
#define NAV_H
// ROS includes.
#include "ros/ros.h"
#include <turtlebot/mymsg.h>
#include <iostream>
//#include "node_example/listener.h"
#include "geometry_msgs/Twist.h"

#include "create_node/TurtlebotSensorState.h"
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
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

using namespace std;

class RoboState
  {
    bool initXneg;
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    ros::Subscriber odomSubscriber;
    //        ros::Subscriber xOdomSubscriber;
    //    ros::Subscriber yOdomSubscriber;
     //void xOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    //    void yOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    
    double xTarget;
    double yTarget;
    double xOdom;
    double yOdom;
    double xOdomOld;
    double yOdomOld;
    double xCoord;
    double yCoord;
    bool messageStatus;
    bool negXturn;
    bool isXswapped;
    double acceptErr;
    double yawErr;
    void faceDestination();
    void determineYawGoal();
    void rotateLR();
    double yaw;
    double yawGoal;
    double tempYaw;
    void setTurnNegX(bool xTurnStatus);
    bool getTurnNegX();
    //    ros::Subscriber yOdomSubscriber;
    //    ros::Subscriber xOdomSubscriber;
    ros::NodeHandle node;
    geometry_msgs::Twist velocityCommand;
    ros::Publisher velocityPublisher;
    geometry_msgs::Twist command;
    ros::Subscriber messageSubscriber;
    ros::Subscriber bumperSubscriber;
    double xIsNegative();
    void incrementX(double x);
    void incrementY(double y);

    void setMessageStatus(bool status);
    bool isMessageSet();
    double getX();
    void setX(double x);
    void setInitialXnegative(bool initialXstatus);
    bool getInitialXnegative();
    bool rotated;
    void setRotated(bool newRotated);
    bool getRotated();

    void setY(double y);
    double getY();
    bool getTurnAndGoForward();
    void setTurnAndGoForward(bool value);
    void bumperCallback(const create_node::TurtlebotSensorState::ConstPtr& msg);
    void messageCallback(const turtlebot::mymsg::ConstPtr& msg);
    void turnForward();
    void goForwardX();
    void goForwardY();
    void rotateLeft();
    void rotateRight();
    void setErr(double err);
    double getErr();
    double getXodom();
    double getYodom();
    double getXodomOld();
    double getYodomOld();
    void setXodomOld(double xOdomCurrent);
    void setYodomOld(double yOdomCurrent);
    bool getIsXswapped();
    void setIsXswapped(bool xSwapValue);
    double yIsNegative();
    void setYawGoal(double newYawGoal);

  public:
    double getYawGoal();
    //    void turn_180();
    void testForward();
    void setXodom(double xOdom);
    void setYaw(double newYaw);
    double getYaw();
    
    void setYodom(double yOdom);

    RoboState(ros::NodeHandle rosNode);
    void goRobotGo();

  };
#include "nav.cpp"
#endif // NAV_NODE_H
