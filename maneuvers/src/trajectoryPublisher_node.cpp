
#include "quad_perch/trajectoryPublisher.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "vehicle_track_traj");
    ros::NodeHandle nh("");

    trajectoryPublisher referencePublisher(nh);
    ros::spin();
    return 0;
}
