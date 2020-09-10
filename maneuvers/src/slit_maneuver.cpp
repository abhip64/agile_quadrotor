/*


*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//HEADER FILES

#include "maneuvers/slit_maneuver.h"
#include <iostream>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace Eigen;

bool started = 0;

int phase = 0;

bool traj_complete = 1;

double time_to_intersection = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CONSTRUCTOR
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
slit_traverse::slit_traverse(const ros::NodeHandle& nh) :
  nh_(nh),
  motion_selector_(0),publish_data(0){
 
    //Variable Definitions
  position_.resize(2);

  velocity_.resize(2);

  T.resize(1);

  image_id = 0;

  vehPos_ << 5, 0, 2.0;

  //position_.at(0) <<0, 0, 5.0;

  //position_.at(1) << 5.0, 0.0 ,5.0;

  //std::cout<<position_.at(1)<<"\n"<<" ****** "<<"\n";

  //position_.at(1) << 1.0, 0.0, 10.0;
  //eth_set_pos(position_.at(0),position_.at(1));

  //eth_trajectory_init();

}


Eigen::Vector3d trajectory_pos(double time)
{
int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
return trajectory.evaluate(time, derivative_order);
}


Eigen::Vector3d trajectory_vel(double time)
{
int derivative_order = mav_trajectory_generation::derivative_order::VELOCITY;
return trajectory.evaluate(time, derivative_order);
}


Eigen::Vector3d trajectory_acc(double time)
{
int derivative_order = mav_trajectory_generation::derivative_order::ACCELERATION;
return trajectory.evaluate(time, derivative_order);
}

Eigen::Vector3d trajectory_jerk(double time)
{
int derivative_order = mav_trajectory_generation::derivative_order::JERK;
return trajectory.evaluate(time, derivative_order);
}

Eigen::Vector3d trajectory_angvel(double time)
{
  Eigen::Vector3d acc, jerk, h, zb, w, xc, yb, xb;

  float m = 0.95;

  Eigen::Vector3d g;

  g << 0, 0, 9.8;
  
  acc  = eth_trajectory_acc(time) + g;
  jerk = eth_trajectory_jerk(time);
  
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