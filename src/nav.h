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
  private:
    ros::Subscriber odomSubscriber;
    double xTarget;
    double yTarget;
    double xOdom;
    double yOdom;
    double oldXodom;
    double oldYodom;
    
    double acceptErr;
    bool isXswapped;
    bool getIsXswapped();
    ros::NodeHandle node;
    geometry_msgs::Twist velocityCommand;
    ros::Publisher velocityPublisher;
    geometry_msgs::Twist command;
    ros::Subscriber messageSubscriber;
    ros::Subscriber bumperSubscriber;
    double xCoord;
    double yCoord;
    bool messageStatus;
    double xIsNegative();
    void incrementX(double x);
    void incrementY(double y);
    void rotateLR();
    void setMessageStatus(bool status);
    bool isMessageSet();
    double getX();
    void setX(double x);
    void setY(double y);
    double getY();
    
    // we still don't know what SpinOnce does
    //void SpinOnce();
    bool getTurnAndGoForward();
    void setTurnAndGoForward(bool value);
    bool turnAndGoForward;
    void bumperCallback(const create_node::TurtlebotSensorState::ConstPtr& msg);
    void messageCallback(const turtlebot::mymsg::ConstPtr& msg);
    void turnForward();
    void  goForward();
    void rotateLeft();
    void rotateRight();
    void setErr(double err);
    double getErr();
    double getXodom();
    double getYodom();
    void setXodom(double xOdom);
    void setYodom(double yOdom);
    double getOldXodom();
    double getOldYodom();
    void setOldXodom();
    void setOldYodom();
    double getOldXodom();
    double getOldYodom();
    void setOldXodom(double xOdom);
    void setOldYodom(double yOdom);
    bool getIsXswapped();
    
    
  public:
    void testForward();
    void turnThenForwardGo();
    RoboState(ros::NodeHandle rosNode);
    void goRobotGo();

  };
#include "nav.cpp"
#endif // NAV_NODE_H
