#include "external_control/upenn_controller.h"

upenn_controller::upenn_controller(const ros::NodeHandle& nh): nh_(nh){

  //Variable initialisation from the values loaded from the parameter server. If values
  //are not available in the parameter server then default values are set.

  //Maximum feedback acceleration 
  nh_.param<double>("max_acc", max_fb_acc_,5.0);

  //Thrust scaling factor
  nh_.param<double>("thrust_scaling_factor", norm_thrust_const_, 0.040);

  //Position control gains along each axis
  nh_.param<double>("Kp_x", Kpos_x_, 15.0);
  nh_.param<double>("Kp_y", Kpos_y_, 15.0);
  nh_.param<double>("Kp_z", Kpos_z_, 15.0);

  //Velocity control gains along each axis
  nh_.param<double>("Kv_x", Kvel_x_, 10.0);
  nh_.param<double>("Kv_y", Kvel_y_, 10.0);
  nh_.param<double>("Kv_z", Kvel_z_, 10.0);

  //Rotation Matrix error gains
  nh_.param<double>("Kr"  , Kr     , 14.0);
  //Angular velocity error gains
  nh_.param<double>("Kw"  , Kw     ,  0.3);

  //Gravitational Vector(NED frame)
  g_ << 0.0, 0.0, 9.81;

  //Control Gain Matrices
  //Position Control
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  //Velocity Control
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
  //Orientation Control
  Kr_   << -Kr     , -Kr     , -Kr;
  //Rate Control
  Kw_   << -Kw     , -Kw     , -Kw;

//Inertia Matix Definition
/*
  J << 0.00406087, 4.157e-05 ,  2.335e-05,
       -4.157e-05, 0.00376785, -9.874e-05,
       -2.335e-05,  9.874e-05, 0.00664413;
*/
}

upenn_controller::~upenn_controller() {
  //Destructor
}

Eigen::Vector4d upenn_controller::calculate_control_fb(Eigen::Vector3d& pos_error, Eigen::Vector3d& vel_error, Eigen::Vector3d& w, Eigen::Vector3d& wd, Eigen::Vector3d& targetAcc_, double& mavYaw_, Eigen::Vector4d& mavAtt_, Eigen::Vector4d& q_des){

  const Eigen::Vector3d a_ref = targetAcc_;
  
  Eigen::Vector3d a_des;

  //Feedback acceleration calculated from position and velocity errors
  Eigen::Vector3d a_fb = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error; 
  
  //Clipping the feedback acceleration
  if(a_fb.norm() > max_fb_acc_) 
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb; 

  //Desired accleration calculated as sum of reference acceleration, feedback acceleration calculated from position
  //and velocity errors and the gravitational vector.
  a_des = a_fb + a_ref + g_;

  q_des = acc2quaternion(a_des, mavYaw_);

  //Calculate BodyRate and thrust
  return attcontroller(q_des, a_des, mavAtt_, w, wd, 1);
}


Eigen::Vector4d upenn_controller::attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att, Eigen::Vector3d& w, Eigen::Vector3d& wd, double tau_)
{

  Eigen::Vector4d control_output;
  Eigen::Vector3d zb, eR, ew;

  //Rotation matrix representing current attitude
  const Eigen::Matrix3d R     = quat2RotMatrix(curr_att);

  //Rotation matrix representing the desired attitude  
  const Eigen::Matrix3d R_des = quat2RotMatrix(ref_att);

  //Error in attitude, calculated in terms of rotation matrices
  eR = 0.5*vee_map(R_des.transpose()*R - R.transpose()*R_des);

  //Error in angular velocity
  ew = w - R.transpose()*R_des*wd;

  Eigen::Vector3d d = (Kr_.asDiagonal()*eR)/tau_ + Kw_.asDiagonal()*ew;

  //Desired angular velocity vector
  control_output(0) = d(0);
  control_output(1) = d(1);
  control_output(2) = d(2);

  //Axis normal to body as seen from ground frame
  zb = R.col(2);

  //Thrust command passed to the Quadrotor. Ideally, the magnitude of thrust should be the desired accleration.
  //But since the quadrotor can exert force only along its normal axis, the component of the desired acceleration
  //along the nody normal axis is determined.
  control_output(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb))); 

  return ratecmd;
}