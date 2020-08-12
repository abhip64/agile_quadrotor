

#include "quad_perch/eth_trajectory.h"

mav_trajectory_generation::Trajectory trajectory;

Eigen::Vector3d init_pos, mid_pos, final_pos;

Eigen::Vector3d init_vel, final_acc, final_vel;


void eth_set_pos(Eigen::Vector3d p_init, Eigen::Vector3d p_final)
{
	init_pos  = p_init;
	final_pos = p_final;
}

void eth_set_vel(Eigen::Vector3d v_init, Eigen::Vector3d v_final)
{
	init_vel  = v_init;
	final_vel = v_final;  
}

void eth_set_acc(Eigen::Vector3d acc)
{
	final_acc = acc;
}

double eth_trajectory_init(double time_to_intersect)
{
mav_trajectory_generation::Vertex::Vector vertices;
const int dimension = 3;
const int derivative_to_optimize = mav_trajectory_generation::derivative_order::VELOCITY;
//const int derivative_to_optimize = opt_deriv;
mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension), flip_node1(dimension), flip_node2(dimension), flip_node3(dimension);

double T_ = 0;

start.makeStartOrEnd(init_pos, derivative_to_optimize);
start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, init_vel);
//start.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, final_vel);
vertices.push_back(start);

//middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, mid_pos);
//middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, mid_vel);
//vertices.push_back(middle);

end.makeStartOrEnd(final_pos, derivative_to_optimize); 
end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, final_vel);
//end.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, final_acc);
vertices.push_back(end);

std::vector<double> segment_times;

const double v_max = 10.0;
const double a_max = 2.0;

//segment_times = estimateSegmentTimes(vertices, v_max, a_max);
segment_times.push_back(time_to_intersect);

for(int i=0;i<segment_times.size();i++)
	T_ += segment_times.at(i);


const int N = 4;
//const int N = 6;
mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
opt.solveLinear();

//mav_trajectory_generation::Segment::Vector segments;
//opt.getSegments(&segments);

opt.getTrajectory(&trajectory);
return T_;
}

Eigen::Vector3d eth_trajectory_pos(double time)
{
int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
return trajectory.evaluate(time, derivative_order);
}


Eigen::Vector3d eth_trajectory_vel(double time)
{
int derivative_order = mav_trajectory_generation::derivative_order::VELOCITY;
return trajectory.evaluate(time, derivative_order);
}


Eigen::Vector3d eth_trajectory_acc(double time)
{
int derivative_order = mav_trajectory_generation::derivative_order::ACCELERATION;
return trajectory.evaluate(time, derivative_order);
}

Eigen::Vector3d eth_trajectory_jerk(double time)
{
int derivative_order = mav_trajectory_generation::derivative_order::JERK;
return trajectory.evaluate(time, derivative_order);
}

Eigen::Vector3d eth_trajectory_angvel(double time)
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

