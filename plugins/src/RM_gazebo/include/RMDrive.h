/*
 *Adapted from the gazebo_ros_skid_steer_drive.h
 *
 *Author: xxy
 */

#ifndef GAZEBO_ROS_SKID_STEER_DRIVE_H_
#define GAZEBO_ROS_SKID_STEER_DRIVE_H_

#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboRosRMDrive : public ModelPlugin {

    public:
	  GazeboRosRMDrive();
      ~GazeboRosRMDrive();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void publishOdometry(double step_time);
      void getWheelVelocities();

      physics::WorldPtr world;
      physics::ModelPtr parent;
      event::ConnectionPtr update_connection_;

      std::string left_front_joint_name_;
      std::string right_front_joint_name_;
      std::string left_rear_joint_name_;
      std::string right_rear_joint_name_;

      //Gimbal by xxy
      std::string gimbal_yaw_joint_name_;
      std::string gimbal_pitch_joint_name_;
      double gimbal_pos_[2];


      double wheel_separation_;
      double wheel_diameter_;
      double torque;
      double wheel_speed_[4];


      physics::JointPtr joints[4];
      physics::JointPtr gimbal_joints[2];

      common::PID gimbal_PID;
      common::PID chassis_PID;
      physics::JointControllerPtr pid_controller;

      // ROS STUFF
      ros::NodeHandle* rosnode_;
      ros::Publisher odometry_publisher_;
      ros::Subscriber cmd_vel_subscriber_;
      tf::TransformBroadcaster *transform_broadcaster_;
      nav_msgs::Odometry odom_;
      std::string tf_prefix_;
      bool broadcast_tf_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // DiffDrive stuff
      void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

      double x_;
      double y_;
      //double rot_;
      double pitch_;
      double rot_;
      bool alive_;

      // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;

  };

}


#endif /* GAZEBO_ROS_SKID_STEER_DRIVE_H_ */
