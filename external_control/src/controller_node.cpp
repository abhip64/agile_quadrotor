//The Node can run with different controllers. The controller which needs to be
//taken is specified in the launch file

#include "quad_perch/controller.h"

int main(int argc, char** argv) {
  ros::init(argc,argv,"upenn_control");
  ros::NodeHandle nh("");

  controller *control_obj = new controller(nh);

  ros::spin();
  return 0;
}
