#include "legged_unitree_hw/UnitreeHW.h"

namespace legged {
bool UnitreeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  robot_hw_nh.getParam("power_limit", powerLimit_);

  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);

  // for a1
#if ROBOT_TYPE == ROBOT_TYPE_A1
  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
#elif ROBOT_TYPE == ROBOT_TYPE_GO1
  // for go1
  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
  // udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007);
#endif

  udp_->InitCmdData(lowCmd_);

  std::string robot_type;
  root_nh.getParam("robot_type", robot_type);

#if ROBOT_TYPE == ROBOT_TYPE_A1
  safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::A1);
#elif ROBOT_TYPE == ROBOT_TYPE_ALIENGO
  safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Aliengo);

#elif ROBOT_TYPE == ROBOT_TYPE_GO1
  safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Go1);
#endif
  // else
  // {
  //   ROS_FATAL("Unknown robot type: %s", robot_type.c_str());
  //   return false;
  // }

  imu_pub = robot_hw_nh.advertise<sensor_msgs::Imu>("/unitree_hardware/imu", 100);
  joint_foot_pub = robot_hw_nh.advertise<sensor_msgs::JointState>("/unitree_hardware/joint_foot", 100);

  swap_joint_indices = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
  swap_foot_indices = {1, 0, 3, 2};

  // to record contact bias
  first_contact_force_read = false;

  return true;
}

void UnitreeHW::read(const ros::Time& currTime /*time*/, const ros::Duration& /*period*/) {
  udp_->Recv();
  udp_->GetRecv(lowState_);

  for (int i = 0; i < 12; ++i) {
    jointData_[i].pos_ = lowState_.motorState[i].q;
    jointData_[i].vel_ = lowState_.motorState[i].dq;
    jointData_[i].tau_ = lowState_.motorState[i].tauEst;
  }

  imuData_.ori_[0] = lowState_.imu.quaternion[1];
  imuData_.ori_[1] = lowState_.imu.quaternion[2];
  imuData_.ori_[2] = lowState_.imu.quaternion[3];
  imuData_.ori_[3] = lowState_.imu.quaternion[0];
  imuData_.angularVel_[0] = lowState_.imu.gyroscope[0];
  imuData_.angularVel_[1] = lowState_.imu.gyroscope[1];
  imuData_.angularVel_[2] = lowState_.imu.gyroscope[2];
  imuData_.linearAcc_[0] = lowState_.imu.accelerometer[0];
  imuData_.linearAcc_[1] = lowState_.imu.accelerometer[1];
  imuData_.linearAcc_[2] = lowState_.imu.accelerometer[2];

  // a temporary logic for Go1, may not work well if the robot stands up
  // initially. record the first contact force reading as contact force bias
  if (first_contact_force_read == false) {
    initTime = currTime;
    first_contact_force_read = true;
  } else {
    ros::Duration elapsedTime = (currTime - initTime);
    double timeSinceStart = elapsedTime.toSec();

    if (timeSinceStart < 0.5) {
      for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
        contactBias_[i] = lowState_.footForce[i];
        contactState_[i] = 0;
      }
    } else {
      for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
        // FR FL RR RL
        contactState_[i] = (lowState_.footForce[i] - contactBias_[i]) > contactThreshold_;
      }
    }
  }

  // Set feedforward and velocity cmd to zero to avoid for safety when not
  // controller setCommand
  std::vector<std::string> names = hybridJointInterface_.getNames();
  for (const auto& name : names) {
    HybridJointHandle handle = hybridJointInterface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(3.);
  }

  // publish ros topics
  ros::Time now = ros::Time::now();
  imu_msg.header.stamp = now;

  imu_msg.linear_acceleration.x = lowState_.imu.accelerometer[0];
  imu_msg.linear_acceleration.y = lowState_.imu.accelerometer[1];
  imu_msg.linear_acceleration.z = lowState_.imu.accelerometer[2];

  imu_msg.angular_velocity.x = lowState_.imu.gyroscope[0];
  imu_msg.angular_velocity.y = lowState_.imu.gyroscope[1];
  imu_msg.angular_velocity.z = lowState_.imu.gyroscope[2];

  imu_pub.publish(imu_msg);
  joint_foot_msg.header.stamp = now;

  joint_foot_msg.name = {"FL0", "FL1", "FL2", "FR0", "FR1",     "FR2",     "RL0",     "RL1",
                         "RL2", "RR0", "RR1", "RR2", "FL_foot", "FR_foot", "RL_foot", "RR_foot"};

  joint_foot_msg.position.resize(NUM_DOF + NUM_LEG);
  joint_foot_msg.velocity.resize(NUM_DOF + NUM_LEG);
  joint_foot_msg.effort.resize(NUM_DOF + NUM_LEG);

  for (int i = 0; i < NUM_DOF; i++) {
    int swap_i = swap_joint_indices[i];
    joint_foot_msg.position[i] = lowState_.motorState[swap_i].q;
    joint_foot_msg.velocity[i] = lowState_.motorState[swap_i].dq;
    joint_foot_msg.effort[i] = lowState_.motorState[swap_i].tauEst;
  }

  // read foot_force
  for (int i = 0; i < NUM_LEG; i++) {
    int swap_i = swap_foot_indices[i];  // 1 0 3 2 (index of footForce) (FL FR RL RR) SO
                                        // THE ORDER OF footForce is: FR FL RR RL
    joint_foot_msg.effort[NUM_DOF + i] = lowState_.footForce[swap_i] - contactBias_[swap_i];
  }
  joint_foot_pub.publish(joint_foot_msg);
}

void UnitreeHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  for (int i = 0; i < 12; ++i) {
    lowCmd_.motorCmd[i].q = static_cast<float>(jointData_[i].posDes_);
    lowCmd_.motorCmd[i].dq = static_cast<float>(jointData_[i].velDes_);
    lowCmd_.motorCmd[i].Kp = static_cast<float>(jointData_[i].kp_);
    lowCmd_.motorCmd[i].Kd = static_cast<float>(jointData_[i].kd_);
    lowCmd_.motorCmd[i].tau = static_cast<float>(jointData_[i].ff_);
  }
  safety_->PositionLimit(lowCmd_);
  safety_->PowerProtect(lowCmd_, lowState_, powerLimit_);
  udp_->SetSend(lowCmd_);
  udp_->Send();
}

bool UnitreeHW::setupJoints() {
  for (const auto& joint : urdfModel_->joints_) {
    int leg_index = 0;
    int joint_index = 0;
    if (joint.first.find("RF") != std::string::npos) {
      leg_index = UNITREE_LEGGED_SDK::FR_;
    } else if (joint.first.find("LF") != std::string::npos) {
      leg_index = UNITREE_LEGGED_SDK::FL_;
    } else if (joint.first.find("RH") != std::string::npos) {
      leg_index = UNITREE_LEGGED_SDK::RR_;
    } else if (joint.first.find("LH") != std::string::npos) {
      leg_index = UNITREE_LEGGED_SDK::RL_;
    } else {
      continue;
    }

    if (joint.first.find("HAA") != std::string::npos) {
      joint_index = 0;
    } else if (joint.first.find("HFE") != std::string::npos) {
      joint_index = 1;
    } else if (joint.first.find("KFE") != std::string::npos) {
      joint_index = 2;
    } else {
      continue;
    }

    int index = leg_index * 3 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].posDes_, &jointData_[index].velDes_,
                                                           &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

bool UnitreeHW::setupImu() {
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("unitree_imu", "unitree_imu", imuData_.ori_, imuData_.oriCov_,
                                                                         imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                         imuData_.linearAccCov_));
  imuData_.oriCov_[0] = 0.0012;
  imuData_.oriCov_[4] = 0.0012;
  imuData_.oriCov_[8] = 0.0012;

  imuData_.angularVelCov_[0] = 0.0004;
  imuData_.angularVelCov_[4] = 0.0004;
  imuData_.angularVelCov_[8] = 0.0004;

  return true;
}

bool UnitreeHW::setupContactSensor(ros::NodeHandle& nh) {
  nh.getParam("contact_threshold", contactThreshold_);
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
  }
  return true;
}

}  // namespace legged
