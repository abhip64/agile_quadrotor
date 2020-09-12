/*
EXECUTE THE SLIT TRAVERSAL MANEUVER
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//HEADER FILES

#include "maneuvers/slit_maneuver.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CONSTRUCTOR
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
slit_traverse::slit_traverse(const ros::NodeHandle& nh): nh_(nh)
{
  //Initial position from which the slit traversal maneuver is initiated
  init_pos << 0.0, 0.0, 0.0;

  //Initial velocity from which the slit traversal maneuver is initiated
  init_vel << 0.0, 0.0, 0.0;

  //Acceleration due to gravity in NED frame
  g_       << 0.0, 0.0, 9.81;

  //Roll angle of the slit which is kept at a specific position
  nh_.param<double>("/trajectory_generator/slit_roll", slit_roll, 0.0);

  //Pitch angle of the slit which is kept at a specific position. Currently the quadrotor does not 
  //pass through slit with pitch angle variation
  nh_.param<double>("/trajectory_generator/slit_pitch", slit_pitch, 0.0);

  //X coordinate of the position at which the slit is placed
  nh_.param<double>("/trajectory_generator/slit_pos_x", slit_pos[0], 5.0);
  
  //Y coordinate of the position at which the slit is placed
  nh_.param<double>("/trajectory_generator/slit_pos_y", slit_pos[1], 0.0);

  //Z coordinate of the position at which the slit is placed
  nh_.param<double>("/trajectory_generator/slit_pos_z", slit_pos[2], 10.0);

  //Height at which the sit traversal maneuver is initiated
  nh.param<double>("/quadrotor_sim/take_off_height", init_pos[2], 5.0);

}

slit_traverse::~slit_traverse() {
  //Destructor
}


double slit_traverse::maneuver_init()
{
  //The trajectory to pass through the slit involves the definition of three waypoints. The first waypoint
  //is the position at which the quadrotor is present immediately after takeoff. The second is the center
  //of the slit. THe third waypoint is a convienently choosen point to bring the quadrotor to rest. Trajectory
  //generation is carried out using the mav_trajectory_generation package
  mav_trajectory_generation::Vertex::Vector vertices;

  //Number of dimensions that need to be considered for each waypoint e.g. Dimension of 3 requires the definition
  //of the waypoint derivatives along X, Y and Z.
  const int dimension = 3;

  //Defintion of three waypoints
  mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

  //The starting point of the trajectory. This is the position that the quadrotor is at immediately after
  //takeoff. The velocity of the quadrotor at this position is 0.
  start.makeStartOrEnd(init_pos, derivative_to_optimize);
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, init_vel);
  vertices.push_back(start);

  //The second waypoint passes through the center of the slit. But in order to pass through the gap the quadrotor
  //must make its bady plane perpendicular to the slit plane. To achieve this the acceleration direction at 
  //the second waypoint is suitably defined. The desired acceleration at this point is rotated to make it 
  //parallel to the slit plane.
  Eigen::Matrix3d R;

  R << 1, 0                   , 0                ,
       0, cos(slit_roll*M_PI/180.0), -sin(slit_roll*M_PI/180.0),
       0, sin(slit_roll*M_PI/180.0), cos(slit_roll*M_PI/180.0) ;

  Eigen::Vector3d slit_acc = R*g_ - g_;

  middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, slit_pos);
  middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, slit_acc);
  vertices.push_back(middle);

  //The last waypoint has been determined using trial and error to obtain the best performance for
  //slit traversal. The objectove is to bring the quadrotor to rest in a smooth manner.
  Eigen::Vector3d final_vel(0.0,0.0,0.0);
  Eigen::Vector3d final_pos(8.0,0.0,10.0);

  end.makeStartOrEnd(final_pos, derivative_to_optimize); 
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, final_vel);
  vertices.push_back(end);

  return eth_trajectory_init(vertices);
}

void slit_traverse::trajectory_generator(double time)
{

  //Target position obtained from the generated trajectory object using the mav_trajectory_generation package
  target_position     = eth_trajectory_pos(time);
  
  //Target velocity obtained from the generated trajectory object using the mav_trajectory_generation package
  target_velocity     = eth_trajectory_vel(time);

  //Target acceleration obtained from the generated trajectory object using the mav_trajectory_generation package
  target_acceleration = eth_trajectory_acc(time);

  //Target jerk obtained from the generated trajectory object using the mav_trajectory_generation package
  target_jerk         = eth_trajectory_jerk(time);

  //Target jerk obtained from the generated trajectory object using the mav_trajectory_generation package
  target_angvel       = calculate_trajectory_angvel();

  //Target Yaw is taken to be 0.0
  target_yaw          = 0.0;

  //This parameter decides the mode of operation of the controller. The controller can work in 5 modes
  //MODE 0 - IGNORE ALL TRAJECTORY TARGET INPUTS (POSITION, VELOCITY, ACCELERATION, ANGULAR VELOCITY, YAW)
  //MODE 1 - ACCEPT ALL TRAJECTORY TARGET INPUTS FOR CONTROL OUTPUT CALCULATION
  //MODE 2 - IGNORE ONLY POSITION FOR CONTROL OUTPUT CALCULATION (ERROR IS TAKEN AS 0)
  //MODE 3 - IGNORE ONLY VELOCITY FOR CONTROL OUTPUT CALCULATION (ERROR IS TAKEN AS 0)
  //MODE 4 - IGNORE POSITION AND VELOCITY FOR CONTROL OUTPUT CALCULATION (ERROR IS TAKEN AS 0)
  type = 1;

}

Eigen::Vector3d slit_traverse::calculate_trajectory_angvel()
{
  Eigen::Vector3d acc, jerk, h, zb, w, xc, yb, xb;

  float m = 0.95;

  Eigen::Vector3d g;

  g << 0, 0, 9.8;
  
  acc  = target_acceleration + g;
  jerk = target_jerk;
  
  double u = acc.norm();

  zb = acc/u;

  xc << cos(target_yaw),sin(target_yaw),0;

  yb = zb.cross(xc) / (zb.cross(xc)).norm();
  xb = yb.cross(zb) / (yb.cross(zb)).norm();

  h = (m/u)*(jerk - (zb.dot(jerk))*zb);

  w(0) = -h.dot(yb);

  w(1) = h.dot(xb);

  w(2) = 0;

  return w;
}

Eigen::Vector3d slit_traverse::get_target_pos()
{
  //Return the desired position
  return target_position;
}

Eigen::Vector3d slit_traverse::get_target_vel()
{
  //Return the desired velocity
  return target_velocity;
}

Eigen::Vector3d slit_traverse::get_target_acc()
{
  //Return the desired acceleration  
  return target_acceleration;
}

Eigen::Vector3d slit_traverse::get_target_angvel()
{
  //Return the desired angular velocity
  return target_angvel;
}

Eigen::Vector3d slit_traverse::get_target_jerk()
{
  //Return the desired jerk
  return target_jerk;
}

double slit_traverse::get_target_yaw()
{
  //Return the desired yaw
  return target_yaw;
}

int slit_traverse::get_type()
{
  //Return the desired mode of operation of the controller
  return type;
}