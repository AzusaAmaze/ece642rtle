#include <ros/ros.h>
#include <boost/bind.hpp>
#include <ece642rtle/timeInt8.h>
#include <std_msgs/Empty.h>
#include <ece642rtle/RTIbump.h>
#include <ece642rtle/RTIatend.h>
#include <ece642rtle/PoseOrntBundle.h>
#include <ece642rtle/bumpEcho.h>
#include <ece642rtle/aendEcho.h>
#include <QPointF>

// Functions to interface with ROS. Don't change these lines!
bool bumped(int x1,int y1,int x2,int y2);
bool atend(int x, int y);
void displayVisits(int visits);
bool moveTurtle(QPointF& pos_, int& nw_or);

// Scope-preserving changes to these lines permitted (see p5 writeup)
typedef enum {
  LEFT, 
  RIGHT, 
  MOVE, 
  STOP
} turtleMove;

typedef enum : int32_t {
  left=0, 
  up=1, 
  right=2, 
  down=3
} directions;  // turtle directions

QPointF translatePos(QPointF pos_, int32_t orientation, turtleMove next_move);
int32_t translateOrnt(int32_t orientation, turtleMove next_move);

turtleMove studentTurtleStep();
int32_t studentTurtleTransit(bool bumped, bool goal);

int32_t turnRight(int32_t turtle_orient);
int32_t turnLeft(int32_t turtle_orient);


