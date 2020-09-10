/*
EXECUTE THE CIRCULAR MANEUVER
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//HEADER FILES

#include "maneuvers/circle_maneuver.h"
#include <iostream>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CONSTRUCTOR
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
circle_traverse::circle_traverse(const ros::NodeHandle& nh): nh_(nh)
{

  //Obtaining the radius of the circular trajectory to be traversed from the parameter server. The
  //default radius is 1.0m
  nh_.param<double>("/trajectory_generator/circle_radius", circle_radius, 1.0);

  //Obtaining the angular velocity of the circular trajectory to be traversed from the parameter 
  //server. The default angular velocity is 0.1 rad/s
  nh_.param<double>("/trajectory_generator/circle_angvel", circle_ang_velocity, 0.1);

  //Obtaining the height of the circular trajectory to be traversed from the parameter 
  //server. The default height is 5.0m
  nh_.param<double>("/trajectory_generator/circle_height", circle_height, 5.0);

  //Number of revolutions of the given circle to be executed. Default is 5
  nh_.param<int>("/trajectory_generator/circle_rev", circle_rev, 5.0);

}

circle_traverse::~circle_traverse() {
  //Destructor
}


int circle_traverse::maneuver_init()
{
  return circle_rev*(2*M_PI/circle_ang_velocity);
}

void circle_traverse::trajectory_generator(double time)
{
  target_position     << circle_radius*sin(circle_ang_velocity*time), circle_radius*cos(circle_ang_velocity*time), circle_height;
  
  target_velocity     << cos(circle_ang_velocity*time), -sin(circle_ang_velocity*time), 0;
  target_velocity     *= circle_radius*circle_ang_velocity;

  target_acceleration << sin(circle_ang_velocity*time), cos(circle_ang_velocity*time), 0;
  target_acceleration *= -circle_radius*circle_ang_velocity*circle_ang_velocity;

  target_jerk          = -target_velocity*circle_ang_velocity*circle_ang_velocity;

  target_angvel        = calculate_trajectory_angvel();

  target_yaw           = atan2(target_position(1),target_position(0)) - M_PI/2.0;

  //target_yaw = 0.0;
}

Eigen::Vector3d circle_traverse::calculate_trajectory_angvel()
{
  Eigen::Vector3d acc, jerk, h, zb, w, xc, yb, xb;

  float m = 0.95;

  Eigen::Vector3d g;

  g << 0, 0, 9.8;
  
  acc  = target_acceleration + g;
  jerk = target_jerk;
  
  double u = acc.norm();

  zb = acc/u;

////////////NEEDS UPDATE//////////////////////// 
  xc << 1,0,0;
///////////////////////////////////////////////

  yb = zb.cross(xc) / (zb.cross(xc)).norm();
  xb = yb.cross(zb) / (yb.cross(zb)).norm();

  h = (m/u)*(jerk - (zb.dot(jerk))*zb);

  w(0) = -h.dot(yb);

  w(1) = h.dot(xb);

  w(2) = 0;

  return w;
}

Eigen::Vector3d circle_traverse::get_target_pos()
{
  return target_position;
}

Eigen::Vector3d circle_traverse::get_target_vel()
{
  return target_velocity;
}

Eigen::Vector3d circle_traverse::get_target_acc()
{
  return target_acceleration;
}

Eigen::Vector3d circle_traverse::get_target_angvel()
{
  return target_angvel;
}

Eigen::Vector3d circle_traverse::get_target_jerk()
{
  return target_jerk;
}

double circle_traverse::get_target_yaw()
{
  return target_yaw;
}