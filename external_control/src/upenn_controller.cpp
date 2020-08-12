#include "quad_perch/upenn_controller.h"
#include <iostream>

upenn_controller::upenn_controller(const ros::NodeHandle& nh): nh_(nh){

  nh_.param<double>("max_acc", max_fb_acc_,5.0);

  nh_.param<double>("drag_dx", Dx_, 0.0);
  nh_.param<double>("drag_dy", Dy_, 0.0);
  nh_.param<double>("drag_dz", Dz_, 0.0);

  nh_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.040); // 1 / max acceleration
  nh_.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.0); // 1 / max acceleration
  nh_.param<double>("Kp_x", Kpos_x_, 15.0);
  nh_.param<double>("Kp_y", Kpos_y_, 15.0);
  nh_.param<double>("Kp_z", Kpos_z_, 15.0);
  //nh_.param<double>("Kr", Kr, 14.0);
  nh_.param<double>("Kr", Kr, 14.0);
  nh_.param<double>("Kw", Kw, 0.3);
  nh_.param<double>("Kv_x", Kvel_x_, 10.0);
  nh_.param<double>("Kv_y", Kvel_y_, 10.0);
  nh_.param<double>("Kv_z", Kvel_z_, 10.0);

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
  
//Drag Matrix
  D_ << Dx_, Dy_, Dz_;

  Ki_ << -0.0, -0.0, -0.0;

  Ai  << 0.0, 0.0, 0.0;

  J << 0.00406087, 4.157e-05 ,  2.335e-05,
       -4.157e-05, 0.00376785, -9.874e-05,
       -2.335e-05,  9.874e-05, 0.00664413;

}

upenn_controller::~upenn_controller() {
  //Destructor
}

Eigen::Vector4d upenn_controller::pos_control(Eigen::Vector3d& pos_error, Eigen::Vector3d& vel, Eigen::Vector3d& targetVel_, Eigen::Vector3d& w, Eigen::Vector3d& wd, Eigen::Vector3d& targetAcc_, double& mavYaw_, Eigen::Vector4d& mavAtt_, Eigen::Vector4d& q_des){

  const Eigen::Vector3d a_ref = targetAcc_;
  
  Eigen::Vector3d a_des;

  //pos_error(2) = 0;
  
  const Eigen::Vector4d q_ref = acc2quaternion(a_ref + g_, mavYaw_);
  const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);
  //Ai += Ki_.asDiagonal()*pos_error;
  const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * targetVel_; //Rotor drag

  Eigen::Vector3d vel_error = vel - targetVel_;

  //vel_error(2) = 0;

  Eigen::Vector3d a_fb = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error; //feedforward term for trajectory error
  if(a_fb.norm() > max_fb_acc_) a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb; //Clip acceleration if reference is too large

  a_des = a_fb + a_ref + g_ - a_rd;

  q_des = acc2quaternion(a_des, mavYaw_);

  return attcontroller(q_des, a_des, mavAtt_, w, wd, 1); //Calculate BodyRate
}

Eigen::Vector4d upenn_controller::hover_control(Eigen::Vector3d& pos_error, Eigen::Vector3d& vel_error, double& mavYaw_, Eigen::Vector4d& mavAtt_, Eigen::Vector4d& q_des){

  //std::cout<<"hover_control"<<"\n";

  const Eigen::Vector3d a_ref(0.0, 0.0, 0.0);
  
  Eigen::Vector3d a_des;

  //pos_error(2) = 0;
  
  const Eigen::Vector4d q_ref = acc2quaternion(a_ref + g_, mavYaw_);
  const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);
  //Ai += Ki_.asDiagonal()*pos_error;

  Eigen::Vector3d a_fb = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error; //feedforward term for trajectory error
  if(a_fb.norm() > max_fb_acc_) a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb; //Clip acceleration if reference is too large

  a_des = a_fb + a_ref + g_;

  q_des = acc2quaternion(a_des, mavYaw_);

  Eigen::Vector3d wd(0,0,0);

  Eigen::Vector3d w(0,0,0);

  return attcontroller(q_des, a_des, mavAtt_, w, wd, 1.0); //Calculate BodyRate
}

Eigen::Vector4d upenn_controller::vel_control(Eigen::Vector3d& vel_err, Eigen::Vector3d& w, Eigen::Vector4d& mavAtt_, Eigen::Vector4d& q_des){

  const Eigen::Vector3d a_ref(0,0,0);
  
  Eigen::Vector3d a_des;

  Eigen::Vector3d a_fb = Kvel_.asDiagonal() * vel_err; //feedforward term for trajectory error
  if(a_fb.norm() > max_fb_acc_) a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb; //Clip acceleration if reference is too large

  a_des = a_fb + a_ref + g_;

  q_des = acc2quaternion(a_des, 0);

  Eigen::Vector3d wd(0,0,0);

  return attcontroller(q_des, a_des, mavAtt_, w, wd, 3); //Calculate BodyRate
}

Eigen::Vector4d upenn_controller::flip_control(Eigen::Vector3d& pos_err, Eigen::Vector3d& vel_err, Eigen::Vector3d& acc, Eigen::Vector4d& mavAtt_, Eigen::Vector4d& q_des)
{

  Eigen::Vector3d a_ref = acc;

//Acceleration feedback term based on position and velocity error
 // Eigen::Vector3d a_fb = Kvel_.asDiagonal() * vel_err; 
  Eigen::Vector3d a_fb(0,0,0);
//Clip acceleration if reference is too large
  if(a_fb.norm() > max_fb_acc_) a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb; 

  const Eigen::Vector3d a_des = a_fb + a_ref + g_;

  q_des = acc2quaternion(a_des, 0);

  Eigen::Vector3d w(0,0,0), wd(0,0,0);

  std::cout<<"*********"<<"\n";

  return attcontroller(q_des, a_des, mavAtt_, w, wd, 0.8); 
}

Eigen::Vector4d upenn_controller::ang_control(double& roll, double z_err, Eigen::Vector3d& w, Eigen::Vector4d& mavAtt_, Eigen::Vector4d& q_des)
{
  Eigen::Matrix3d R;

  R << 1, 0                   , 0                    ,
       0, cos(roll*deg_to_rad), -sin(roll*deg_to_rad),
       0, sin(roll*deg_to_rad), cos(roll*deg_to_rad) ;

  //Eigen::Vector3d a_ref = R*(g_/cos(roll*deg_to_rad)) - g_;
  Eigen::Vector3d a_ref = R*g_ - g_;

  Eigen::Vector3d pos_error;

  pos_error << 0,0,0;
//Acceleration feedback term based on position and velocity error
  Eigen::Vector3d a_fb = Kpos_.asDiagonal() * pos_error; 
//Clip acceleration if reference is too large
  if(a_fb.norm() > max_fb_acc_) a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb; 

  const Eigen::Vector3d a_des = a_fb + a_ref + g_;

  q_des = acc2quaternion(a_des, 0);

  Eigen::Vector3d wd(0,0,0);

  return attcontroller(q_des, a_des, mavAtt_, w, wd, 1); //Calculate BodyRate
}

Eigen::Vector4d upenn_controller::acc2quaternion(const Eigen::Vector3d vector_acc, double yaw) 
{
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / ( yb_des.cross(zb_des) ).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0),
            xb_des(1), yb_des(1), zb_des(1),
            xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}


Eigen::Vector4d upenn_controller::attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att, Eigen::Vector3d& w, Eigen::Vector3d& wd, double tau_)
{

  Eigen::Vector4d ratecmd;
  Eigen::Vector4d qe, q_inv, inverse;
  Eigen::Matrix3d rotmat;
  Eigen::Vector3d zb, eR, ew;

  const Eigen::Matrix3d R     = quat2RotMatrix(curr_att);  
  const Eigen::Matrix3d R_des = quat2RotMatrix(ref_att);

  eR = 0.5*vee_map(R_des.transpose()*R - R.transpose()*R_des);
  ew = w - R.transpose()*R_des*wd;

  Ai += Ki_.asDiagonal()*eR;

  //for(int i=0;i<3;i++)
  //  Ai(i) = std::max(0.05,Ai(i));

  Eigen::Vector3d d = (Kr_.asDiagonal()*eR)/tau_ + Kw_.asDiagonal()*ew + Ai; //+ w.cross(J*w);

  ratecmd(0) = d(0);
  ratecmd(1) = d(1);
  ratecmd(2) = d(2);

  zb = R.col(2);

 //Thrust command passed to the Quadrotor
  ratecmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_)); 

  //std::cout<<ratecmd(3)<<"\n";
  //std::cout<<zb(0)<<" "<<ref_acc(0)<<"\n";
  //std::cout<<zb(1)<<" "<<ref_acc(1)<<"\n";
  //std::cout<<zb(2)<<" "<<ref_acc(2)<<"\n";

  return ratecmd;
}