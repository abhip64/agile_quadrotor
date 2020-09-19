////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
THIS CODE IS USED TO OBTAIN THE POSITION DATA OF THE ACKERMANN VEHICLE THAT IS PUBLISHED BY THE APRILTAG_ROS PACKAGE.
APRIL TAG 16H5 WITH TAG ID 007 IS FIXED ON TOP OF THE ACKERMANN VEHICLE. THE QUADROTOR HAS A CAMERA FIXED ON IT. WHEN
THE CAMERA IS IN RANGE OF THE APRIL TAG IMAGE IT IS ABLE TO ESIMATE THE POSITION OF THE TAG WITH RESPECT TO THE CAMERA
COORDINATE FRAME. THIS DATA CAN BE FUSED ALONG WITH THE POSITION DATA THAT IS PUBLISHED BY ACKERMANN VEHICLE
TO OBTAIN THE STATES OF THE VEHICLE WITH HIGHER CONFIDENCE. BOTH THE POSITION DATA CAN BE FUSED BY MAKING USE OF 
A KALMAN FILTER. BEFORE THE DATA FROM THE CAMERA CAN BE FUSED, IT HAS TO BE TRANSFORMED TO THE EARTH FIXED FRAME.
THUS THE CODE OBTAINS THE POSITION DATA FROM THE CAMERA, TRANSFORMS IT TO EARTH FRAME AND THEN REPUBLISHES IT 
SO THAT IT CAN BE USED BY THE KALMAN FILTER FOR STATE ESTIMATION.
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//HEADER FILES
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <Eigen/Dense>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <geometry_msgs/PoseStamped.h>

#include "quadrotor_sim/math_operations.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
geometry_msgs::Pose vehicle_pose;
geometry_msgs::PoseWithCovarianceStamped vehicle_stamped_pose, image_pose; 
ros::Time curr_time;

//Image ID of 7 corresponds to the required April Tag
int image_id = 0;

//Position of the apriltag in camera frame
Eigen::Vector3d image_pos; 
//Position of the apriltag in earth frame centered at the origin
Eigen::Vector3d image_pos_earth_frame;
//Current position of the quadrotor
Eigen::Vector3d mavPos_;
//Current attitude of the quadrotor
Eigen::Vector4d mavAtt_;

//This function transforms the position of the april tag measured in the camera frame to the world frame
//with center at origin.
void image_pos_transform()
{
  image_pos            << -image_pos(1) + 0.1, -image_pos(0), -image_pos(2);

  image_pos             = quat2RotMatrix(mavAtt_)*image_pos;

  image_pos_earth_frame = image_pos + mavPos_;
}

//Function to subscribe to the position of the April Tag in the image coordinate frame. This data is published by
//the apriltag_ros package. The april tag fixed to the top surface of the ackermann vehicle is Tag16H5 007.
void image_pos_sub(const apriltag_ros::AprilTagDetectionArray& msg){
  
  if(!(msg.detections.empty()))
  {
    image_id   = msg.detections[0].id[0];

    image_pose = msg.detections[0].pose;

    image_pos << msg.detections[0].pose.pose.pose.position.x, msg.detections[0].pose.pose.pose.position.y, msg.detections[0].pose.pose.pose.position.z;

    curr_time  = ros::Time::now();
  }
}

//Function to subscribe to the position and attitude of the quadrotor
void mavposeCallback(const geometry_msgs::PoseStamped& msg){

  mavPos_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

  mavAtt_(0) = msg.pose.orientation.w;
  mavAtt_(1) = msg.pose.orientation.x;
  mavAtt_(2) = msg.pose.orientation.y;
  mavAtt_(3) = msg.pose.orientation.z;

}

int main(int argc, char **argv)
{
  //Initialise ROS node with name "apriltag_pos_transform"
  ros::init(argc, argv, "apriltag_pos_transform");

  ros::NodeHandle node;

  curr_time                          = ros::Time::now();

  ros::Publisher vehicle_img_pos_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ugv_camera/pose", 1000);

  ros::Subscriber mavposeSub_        = node.subscribe("/mavros/local_position/pose", 50, mavposeCallback,ros::TransportHints().tcpNoDelay());

  ros::Subscriber imageposSub_       = node.subscribe("/tag_detections", 30, image_pos_sub,ros::TransportHints().tcpNoDelay());

  int frame_sequence_                = 0;

  //Execute at a loop rate of 30Hz
  ros::Rate loop_rate(30);

  while (ros::ok())
  {

    if(image_id == 7)
    {
      //Transform position data from camera frame to earth frame
      image_pos_transform();

      //Publish the position of the ackermann vehicle so that it can be used by
      //Kalman filter. This data is used along with the position data published by the 
      //ackermann vehicle to get a better estimate on the states of the ackermann vehicle
      vehicle_stamped_pose.header.stamp         = curr_time;

      vehicle_stamped_pose.header.seq           = ++ frame_sequence_;

      vehicle_stamped_pose.header.frame_id      = "odom";

      vehicle_stamped_pose.pose.pose.position.x = image_pos_earth_frame(0);
      vehicle_stamped_pose.pose.pose.position.y = image_pos_earth_frame(1);
      vehicle_stamped_pose.pose.pose.position.z = image_pos_earth_frame(2);

      vehicle_stamped_pose.pose.covariance      = image_pose.pose.covariance;

      vehicle_img_pos_pub.publish(vehicle_stamped_pose);
    }

    ros::spinOnce();

    loop_rate.sleep();
    
  }


  return 0;
}