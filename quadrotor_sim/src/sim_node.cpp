//The Node can run with different controllers. The controller which needs to be
//taken is specified in the launch file

#include "quadrotor_sim/quad_sim.h"

int main(int argc, char** argv) {
  ros::init(argc,argv,"quadrotor_sim");
  ros::NodeHandle nh("");

  quadrotor_sim *control_obj = new quadrotor_sim(nh);

  ros::spin();
  return 0;
}
