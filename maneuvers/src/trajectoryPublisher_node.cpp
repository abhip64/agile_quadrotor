
#include "maneuvers/trajectoryPublisher.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh("");

    trajectoryPublisher referencePublisher(nh);
    ros::spin();
    return 0;
}
