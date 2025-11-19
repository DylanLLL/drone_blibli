#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControlMinimal : public rclcpp::Node
{
public:
  OffboardControlMinimal() : Node("offboard_control_minimal")
  {
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    // Timer callback runs every 100ms
    timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControlMinimal::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Offboard Control Minimal started");
  }

  void arm();
  void disarm();

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

  uint64_t offboard_setpoint_counter_ = 0;

  void timer_callback();
  void publish_offboard_control_mode();
  void publish_trajectory_setpoint();
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

void OffboardControlMinimal::timer_callback()
{
  // Continuously publish trajectory setpoint (hovering at -2.5m)
  // This MUST be published before switching to offboard mode
  // publish_trajectory_setpoint();

  // Always publish offboard control heartbeat
  publish_offboard_control_mode();

  if (offboard_setpoint_counter_ == 10)
  {
    // After sending 10 setpoints, engage offboard mode
    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    RCLCPP_INFO(this->get_logger(), "Switching to OFFBOARD mode");
  }

  if (offboard_setpoint_counter_ == 11)
  {
    // Arm after offboard mode is engaged
    this->arm();
  }

  if (offboard_setpoint_counter_ < 12)
  {
    offboard_setpoint_counter_++;
  }
}

void OffboardControlMinimal::arm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
  RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void OffboardControlMinimal::disarm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
  RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void OffboardControlMinimal::publish_offboard_control_mode()
{
  OffboardControlMode msg{};
  msg.position = true;
  msg.velocity = true;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}

void OffboardControlMinimal::publish_trajectory_setpoint()
{
  TrajectorySetpoint msg{};
  msg.position = { 0.0, 0.0, -2.5 };  // NED frame: x=0, y=0, z=-2.5m (2.5m above ground)
  msg.yaw = 0.0;                      // radians
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControlMinimal::publish_vehicle_command(uint16_t command, float param1, float param2)
{
  VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(msg);
}

int main(int argc, char* argv[])
{
  std::cout << "Starting minimal offboard control node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControlMinimal>());

  rclcpp::shutdown();
  return 0;
}
