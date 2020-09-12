

#include "maneuvers/eth_trajectory.h"

mav_trajectory_generation::Trajectory trajectory;

double eth_trajectory_init(mav_trajectory_generation::Vertex::Vector vertices)
{
  //This function calculates the snap minimal optimal trajectory that passes through the given vertices.
  //Using the generated trajectory object the desired position, velocity, accelration and jerk along
  //the trajectory can be convienently calculated. Desired angular acceleration can also be determined
  //taking into consideration the differenetial flatnees nature of the quadrotor. Yaw planning is not
  //coupled with trajectory design

  //Generated trajectory has to be a snap minimal trajectory
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

  //X Y Z
  const int dimension = 3;

  double T_ = 0;

  //Maximum possible allowable velocity along the trajectory
  const double v_max = 20.0;

  //Maximum possible allowable acceleration along the trajectory
  const double a_max = 10.0;

  //Segment time calculation using the data of the maximum possible velocity and acceleration
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, v_max, a_max);
  //Total time for traversing the trajectory
  for(int i=0;i<segment_times.size();i++)
	T_ += segment_times.at(i);

  //Number of unknowns in the polynomial
  const int N = 10;

  //Determine the optimal trajectory satisfying the waypoints and the optimality condition
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();

  opt.getTrajectory(&trajectory);

  //Return the time it takes for traversing the trajectory
  return T_;
}

Eigen::Vector3d eth_trajectory_pos(double time)
{
  //Return the desired position along the trajectory for the given input time
  int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
  return trajectory.evaluate(time, derivative_order);
}


Eigen::Vector3d eth_trajectory_vel(double time)
{
  //Return the desired velocity along the trajectory for the given input time
  int derivative_order = mav_trajectory_generation::derivative_order::VELOCITY;
  return trajectory.evaluate(time, derivative_order);
}


Eigen::Vector3d eth_trajectory_acc(double time)
{
  //Return the desired acceleration along the trajectory for the given input time
  int derivative_order = mav_trajectory_generation::derivative_order::ACCELERATION;
  return trajectory.evaluate(time, derivative_order);
}

Eigen::Vector3d eth_trajectory_jerk(double time)
{
  //Return the desired jerk along the trajectory for the given input time
  int derivative_order = mav_trajectory_generation::derivative_order::JERK;
  return trajectory.evaluate(time, derivative_order);
}

