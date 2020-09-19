

#include "maneuvers/eth_trajectory.h"

mav_trajectory_generation::Trajectory trajectory;

extern const int derivative_to_optimize;


void eth_trajectory_init(mav_trajectory_generation::Vertex::Vector vertices, std::vector<double> segment_times, int derv_opt)
{
  //Generated trajectory has to be a snap minimal trajectory
  const int dimension = 3;

  
  switch(derv_opt)
  {
    case 0:
    {
      const int derivative_to_optimize = mav_trajectory_generation::derivative_order::POSITION;
      const int N = 2*(derivative_to_optimize + 1);
        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  opt.solveLinear();

  opt.getTrajectory(&trajectory);
      break;
    }
    case 1:
    {
      const int derivative_to_optimize = mav_trajectory_generation::derivative_order::VELOCITY;
            const int N = 2*(derivative_to_optimize + 1);

        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  opt.solveLinear();

  opt.getTrajectory(&trajectory);
      break;
    }
    case 2:
    {
      const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
            const int N = 2*(derivative_to_optimize + 1);

        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  opt.solveLinear();

  opt.getTrajectory(&trajectory);
      break;
    }   
    case 3:
    {
      const int derivative_to_optimize = mav_trajectory_generation::derivative_order::JERK;
            const int N = 2*(derivative_to_optimize + 1);

        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  opt.solveLinear();

  opt.getTrajectory(&trajectory);
      break;
    }  
    case 4:
    {
      const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
            const int N = 2*(derivative_to_optimize + 1);

        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  opt.solveLinear();

  opt.getTrajectory(&trajectory);
      break;
    } 
  }

  //const int dummy = 2*(derivative_to_optimize + 1);
  //Number of unknowns in the polynomial


  //Number of dimensions that need to be considered for each waypoint e.g. Dimension of 3 requires the definition
  //of the waypoint derivatives along X, Y and Z.


  //Determine the optimal trajectory satisfying the waypoints and the optimality condition

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

