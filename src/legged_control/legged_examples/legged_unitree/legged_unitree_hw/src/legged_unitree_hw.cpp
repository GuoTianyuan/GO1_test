#include "legged_hw/LeggedHWLoop.h"
#include "legged_unitree_hw/UnitreeHW.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "legged_unitree_hw");
  ros::NodeHandle nh;
  ros::NodeHandle robotHwNh("~");

  // Run the hardware interface node
  // -------------------------------

  // We run the ROS loop in a separate thread as external calls, such
  // as service callbacks loading controllers, can block the (main) control loop

  ros::AsyncSpinner spinner(3);
  spinner.start();

  try {
    // Create the hardware interface specific to your robot
    std::shared_ptr<legged::UnitreeHW> unitreeHw = std::make_shared<legged::UnitreeHW>();
    // Initialize the hardware interface:
    // 1. retrieve configuration from rosparam
    // 2. initialize the hardware and interface it with ros_control
    unitreeHw->init(nh, robotHwNh);

    // Start the control loop
    legged::LeggedHWLoop controlLoop(nh, unitreeHw);

    // Wait until shutdown signal received
    ros::waitForShutdown();
  } catch (const ros::Exception& e) {
    ROS_FATAL_STREAM("Error in the hardware interface:\n"
                     << "\t" << e.what());
    return 1;
  }

  return 0;
}
