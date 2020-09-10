
#ifndef CONROLLER_CLASS_H
#define CONROLLER_CLASS_H

#include <ros/ros.h>
#include <Eigen/Dense>

class control_class{

private:

	ros::NodeHandle nh_;

public:

	control_class();
    
    virtual ~control_class();

	virtual Eigen::Vector4d calculate_control_fb(Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, double&, Eigen::Vector4d&, Eigen::Vector4d&) = 0;
	virtual Eigen::Vector4d attcontroller(const Eigen::Vector4d&, const Eigen::Vector3d&, Eigen::Vector4d&, Eigen::Vector3d&, Eigen::Vector3d&, double) = 0;

};



#endif