#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>
#include <thread>
#include <std_msgs/Float64.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

#include "legged_controllers/ROSJoyProcessor.hpp"
#include "legged_controllers/target.h"
namespace legged {
using namespace ocs2;

class TargetTrajectoriesPublisher final {
 public:
  using CmdToTargetTrajectories = std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;
  using CmdWithTerraingToTargetTrajectories = std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation, const scalar_t &angle)>;

  TargetTrajectoriesPublisher(::ros::NodeHandle& nh, const std::string& gaitFile, const std::string& topicPrefix,
                              CmdToTargetTrajectories goalToTargetTrajectories, CmdToTargetTrajectories cmdVelToTargetTrajectories,
                              CmdWithTerraingToTargetTrajectories cmdVelToTargetTrajectoriesWithTerrain)
      : goalToTargetTrajectories_(std::move(goalToTargetTrajectories)),
        cmdVelToTargetTrajectories_(std::move(cmdVelToTargetTrajectories)),
        cmdVelToTargetTrajectoriesWithTerrain_(std::move(cmdVelToTargetTrajectoriesWithTerrain)),
        tf2_(buffer_) {
    // Trajectories publisher
    targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nh, topicPrefix));

    // observation subscriber
    auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    };
    observationSub_ = nh.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

    // goal subscriber
    auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) {
        return;
      }
      geometry_msgs::PoseStamped pose = *msg;
      try {
        buffer_.transform(pose, pose, "odom", ros::Duration(0.2));
      } catch (tf2::TransformException& ex) {
        ROS_WARN("Failure %s\n", ex.what());
        return;
      }

      vector_t cmdGoal = vector_t::Zero(6);
      cmdGoal[0] = pose.pose.position.x;
      cmdGoal[1] = pose.pose.position.y;
      cmdGoal[2] = pose.pose.position.z;
      Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
      cmdGoal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
      cmdGoal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
      cmdGoal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

      const auto trajectories = goalToTargetTrajectories_(cmdGoal, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    // cmd_vel subscriber
    auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) {
        return;
      }

      vector_t cmdVel = vector_t::Zero(4);
      cmdVel[0] = msg->linear.x;
      cmdVel[1] = msg->linear.y;
      cmdVel[2] = msg->linear.z;
      cmdVel[3] = msg->angular.z;

      const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    auto terrainAngleCallback = [this](const std_msgs::Float64::ConstPtr &msg){
      if (latestObservation_.time == 0.0) {
        return;
      }
      terrain_angle = msg->data * (3.1415926 / 180);
      // std::cout << "terrain_angle " << terrain_angle << std::endl;
    };

    goalSub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalCallback);

    terrain_angle_sub_ = nh.subscribe<std_msgs::Float64>("/go1_debug/terrain_angle", 1, terrainAngleCallback);
    
    cmdVelSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);

    // start a slow loop to process rosJoy data
    rosJoy = new ROSJoyProcessor(nh, gaitFile, topicPrefix);
    ros::Time prev = ros::Time::now();
    ros::Time now = ros::Time::now();
    ros::Duration dt(0);
    joyThread_ = std::thread([&]() {
      while (ros::ok()) {
        if (latestObservation_.time == 0.0) {
          continue;
        }

        try {
          rosJoy->processJoy(0);

          vector_t cmdVel = vector_t::Zero(4);
          cmdVel[0] = rosJoy->joy_cmd_velx;
          cmdVel[1] = rosJoy->joy_cmd_vely;
          cmdVel[2] = rosJoy->joy_cmd_velz;
          cmdVel[3] = rosJoy->joy_cmd_yaw_rate;
          auto doTar = [this](const legged_controllers::target::ConstPtr& target_p){
            int x = target_p->x;
            int y = target_p->y;
            double dis = target_p->dis;
            double kp_vel = 5,kp_ang = 4,kd_vel = 1;
            kd_dis += (dis - 0.5);
            cmd_vel = (dis - 0.5) * kp_vel + kd_vel * sum_dis;
            if(cmd_vel > 4){
              cmd_vel = 4;
            }else if(cmd_vel < -4){
              cmd_vel = -4;
            }
          };
          tar_ = nh.subscribe<legged_controllers::target>("dis_msg",1,doTar);
          cmdVel[0] = cmd_vel;
          cmdVel[1] = 0;
          cmdVel[2] = 0;
          cmdVel[3] = 0;
          // const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, latestObservation_);
          const auto trajectories = cmdVelToTargetTrajectoriesWithTerrain_(cmdVel, latestObservation_, terrain_angle);
          targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
        } catch (const std::exception& e) {
          ROS_ERROR_STREAM("[ros JOY] Error : " << e.what());
        }
      }
    });
  }


 private:
  CmdToTargetTrajectories goalToTargetTrajectories_, cmdVelToTargetTrajectories_;
  CmdWithTerraingToTargetTrajectories cmdVelToTargetTrajectoriesWithTerrain_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;

  ::ros::Subscriber observationSub_, goalSub_, cmdVelSub_, terrain_angle_sub_,tar_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  ROSJoyProcessor* rosJoy;
  std::thread joyThread_;

  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
  double cmd_vel,sum_dis = 0;
  scalar_t terrain_angle;
};

}  // namespace legged
