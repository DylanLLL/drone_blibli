
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

using namespace std::chrono_literals;

class OffboardWaypointControl : public rclcpp::Node
{
public:
  OffboardWaypointControl() : Node("offboard_waypoint_control"), waypoint_index_(0)
  {
    // Publishers
    offboard_control_mode_publisher_ =
        this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ =
        this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/in/vehicle_command", 10);

    // Timer: 20ms = 50Hz
    timer_ = this->create_wall_timer(20ms, std::bind(&OffboardWaypointControl::timer_callback, this));
    last_switch_time_ = this->now();

    // Define waypoints
    add_waypoint(0.0, 0.0, -2.0);
    add_waypoint(5.0, 0.0, -2.0);
    add_waypoint(5.0, 5.0, -2.0);
    add_waypoint(0.0, 5.0, -2.0);
  }

private:
  void add_waypoint(float x, float y, float z)
  {
    px4_msgs::msg::TrajectorySetpoint wp{};
    wp.position = { x, y, z };
    wp.yaw = 0.0;
    waypoints_.push_back(wp);
  }

  void timer_callback()
  {
    // Always publish offboard control mode
    px4_msgs::msg::OffboardControlMode offboard_mode{};
    offboard_mode.timestamp = now_us();
    offboard_mode.position = true;
    offboard_control_mode_publisher_->publish(offboard_mode);

    // Always publish current setpoint
    auto current_wp = waypoints_[waypoint_index_];
    current_wp.timestamp = now_us();
    trajectory_setpoint_publisher_->publish(current_wp);

    // Switch to Offboard and Arm once
    if (!offboard_started_ && (this->now() - start_time_).seconds() > 1.0)
    {
      RCLCPP_INFO(this->get_logger(), "Switching to Offboard mode and arming");
      publish_vehicle_command(176, 1.0);  // Set mode to Offboard
      publish_vehicle_command(400, 1.0);  // Arm
      offboard_started_ = true;
      last_switch_time_ = this->now();
    }

    // Change waypoint every 5 seconds
    if (offboard_started_ && (this->now() - last_switch_time_).seconds() > 5.0)
    {
      if (waypoint_index_ < waypoints_.size() - 1)
      {
        waypoint_index_++;
        last_switch_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Switching to waypoint %ld", waypoint_index_);
      }
    }
  }

  uint64_t now_us()
  {
    return this->get_clock()->now().nanoseconds() / 1000;
  }

  void publish_vehicle_command(uint16_t command, float param1)
  {
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = now_us();
    cmd.param1 = param1;
    cmd.command = command;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    vehicle_command_publisher_->publish(cmd);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

  std::vector<px4_msgs::msg::TrajectorySetpoint> waypoints_;
  size_t waypoint_index_;
  rclcpp::Time last_switch_time_;
  rclcpp::Time start_time_ = this->now();
  bool offboard_started_ = false;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardWaypointControl>());
  rclcpp::shutdown();
  return 0;
}