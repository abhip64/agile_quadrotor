
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

	virtual Eigen::Vector4d pos_control(Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, double&, Eigen::Vector4d&, Eigen::Vector4d&) = 0;
	virtual Eigen::Vector4d hover_control(Eigen::Vector3d&, Eigen::Vector3d&, double&, Eigen::Vector4d&, Eigen::Vector4d&) = 0;
	virtual Eigen::Vector4d vel_control(Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector4d&, Eigen::Vector4d&) = 0;
    virtual Eigen::Vector4d ang_control(double&, double, Eigen::Vector3d&, Eigen::Vector4d&, Eigen::Vector4d&) = 0;
    virtual Eigen::Vector4d acc2quaternion(const Eigen::Vector3d vector_acc, double yaw) = 0;
	virtual Eigen::Vector4d attcontroller(const Eigen::Vector4d&, const Eigen::Vector3d&, Eigen::Vector4d&, Eigen::Vector3d&, Eigen::Vector3d&, double) = 0;
    virtual Eigen::Vector4d flip_control(Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector4d&, Eigen::Vector4d&) = 0;

};



#endif