/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <stdint.h>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include "offboard_control_config.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>  // Required for std::ceil

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::this_thread;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
  OffboardControl() : Node("offboard_control")
  {
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    // Calculate y_total_steps_ after all parameters are initialized
    y_total_steps_ = static_cast<int>(std::ceil(std::abs(y_beginning_ - y_end_) / step_movement_));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_interval_ms_), [this]() {
      publish_offboard_control_mode();  // must be sent continuously
      timer_tick_count_++;

      switch (phase)
      {
        case FlightPhase::INIT:
          if (offboard_setpoint_counter_ <= 10)
          {
            offboard_setpoint_counter_++;
            publish_takeoff_setpoint();  // start sending setpoints for offboard mode
          }
          else
          {
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);  // Set to Offboard
            this->arm();
            RCLCPP_INFO(this->get_logger(), "Switched to OFFBOARD and ARMED.");
            phase = FlightPhase::INIT;
            timer_tick_count_ = 0;
          }
          break;

        case FlightPhase::TAKEOFF:
          publish_takeoff_setpoint();
          if (timer_tick_count_ >= hold_ticks_)
          {
            phase = FlightPhase::TAKEOFF;  // ASCEND_TO_LEVEL1;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Ascending to level 1.");
          }
          break;

        case FlightPhase::ASCEND_TO_LEVEL1: {
          z_total_steps_ = static_cast<int>(std::ceil(std::abs(z_level1 - (z_takeoff)) / step_movement_));
          float z = z_takeoff - step_movement_ * z_current_step_;
          if (z <= z_level1)
            z = z_level1;
          publish_position_setpoint(0.0, y_beginning_, z);
          z_current_step_++;
          if (z_current_step_ > z_total_steps_)
          {
            phase = FlightPhase::SCAN_LEVEL1;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Starting rack scan at Level 1");
          }
          break;
        }

        case FlightPhase::SCAN_LEVEL1: {
          float y = y_beginning_ + step_movement_ * y_current_step_;
          if (y >= y_end_)
            y = y_end_;
          publish_position_setpoint(0.0, y, z_level1);
          y_current_step_++;

          if (y_current_step_ > y_total_steps_)
          {
            phase = FlightPhase::ASCEND_TO_LEVEL2;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Ascending to level 2.");
          }
          break;
        }

        case FlightPhase::ASCEND_TO_LEVEL2: {
          z_total_steps_ = static_cast<int>(std::ceil(std::abs(z_level2 - (z_level1)) / step_movement_));
          float z = z_level1 - step_movement_ * z_current_step_;
          if (z <= z_level2)
            z = z_level2;
          publish_position_setpoint(0.0, y_end_, z);
          z_current_step_++;
          if (z_current_step_ > z_total_steps_)
          {
            phase = FlightPhase::SCAN_LEVEL2;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Starting rack scan at Level 2");
          }
          break;
        }

        case FlightPhase::SCAN_LEVEL2: {
          float y = y_end_ - step_movement_ * y_current_step_;
          if (y <= y_beginning_)
            y = y_beginning_;
          publish_position_setpoint(0.0, y, z_level2);
          y_current_step_++;

          if (y_current_step_ > y_total_steps_)
          {
            phase = FlightPhase::ASCEND_TO_LEVEL3;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Ascending to level 3.");
          }
          break;
        }

        case FlightPhase::ASCEND_TO_LEVEL3: {
          z_total_steps_ = static_cast<int>(std::ceil(std::abs(z_level3 - (z_level2)) / step_movement_));
          float z = z_level2 - step_movement_ * z_current_step_;
          if (z <= z_level3)
            z = z_level3;
          publish_position_setpoint(0.0, y_beginning_, z);
          z_current_step_++;
          if (z_current_step_ > z_total_steps_)
          {
            phase = FlightPhase::SCAN_LEVEL3;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Starting rack scan at Level 3");
          }
          break;
        }

        case FlightPhase::SCAN_LEVEL3: {
          float y = y_beginning_ + step_movement_ * y_current_step_;
          if (y >= y_end_)
            y = y_end_;
          publish_position_setpoint(0.0, y, z_level3);
          y_current_step_++;

          if (y_current_step_ > y_total_steps_)
          {
            phase = FlightPhase::ASCEND_TO_LEVEL4;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Ascending to level 4.");
          }
          break;
        }

        case FlightPhase::ASCEND_TO_LEVEL4: {
          z_total_steps_ = static_cast<int>(std::ceil(std::abs(z_level4 - (z_level3)) / step_movement_));
          float z = z_level3 - step_movement_ * z_current_step_;
          if (z <= z_level4)
            z = z_level4;
          publish_position_setpoint(0.0, y_end_, z);
          z_current_step_++;
          if (z_current_step_ > z_total_steps_)
          {
            phase = FlightPhase::SCAN_LEVEL4;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Starting rack scan at Level 4");
          }
          break;
        }

        case FlightPhase::SCAN_LEVEL4: {
          float y = y_end_ - step_movement_ * y_current_step_;
          if (y <= y_beginning_)
            y = y_beginning_;
          publish_position_setpoint(0.0, y, z_level4);
          y_current_step_++;

          if (y_current_step_ > y_total_steps_)
          {
            phase = FlightPhase::ASCEND_TO_LEVEL5;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Ascending to level 5.");
          }
          break;
        }

        case FlightPhase::ASCEND_TO_LEVEL5: {
          z_total_steps_ = static_cast<int>(std::ceil(std::abs(z_level5 - (z_level4)) / step_movement_));
          float z = z_level4 - step_movement_ * z_current_step_;
          if (z <= z_level5)
            z = z_level5;
          publish_position_setpoint(0.0, y_beginning_, z);
          z_current_step_++;
          if (z_current_step_ > z_total_steps_)
          {
            phase = FlightPhase::SCAN_LEVEL5;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Starting rack scan at Level 5");
          }
          break;
        }

        case FlightPhase::SCAN_LEVEL5: {
          float y = y_beginning_ + step_movement_ * y_current_step_;
          if (y >= y_end_)
            y = y_end_;
          publish_position_setpoint(0.0, y, z_level5);
          y_current_step_++;

          if (y_current_step_ > y_total_steps_)
          {
            phase = FlightPhase::ASCEND_TO_LEVEL6;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Ascending to level 6.");
          }
          break;
        }

        case FlightPhase::ASCEND_TO_LEVEL6: {
          z_total_steps_ = static_cast<int>(std::ceil(std::abs(z_level6 - (z_level5)) / step_movement_));
          float z = z_level5 - step_movement_ * z_current_step_;
          if (z <= z_level6)
            z = z_level6;
          publish_position_setpoint(0.0, y_end_, z);
          z_current_step_++;
          if (z_current_step_ > z_total_steps_)
          {
            phase = FlightPhase::SCAN_LEVEL6;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Starting rack scan at Level 6");
          }
          break;
        }

        case FlightPhase::SCAN_LEVEL6: {
          float y = y_end_ - step_movement_ * y_current_step_;
          if (y <= y_beginning_)
            y = y_beginning_;
          publish_position_setpoint(0.0, y, z_level6);
          y_current_step_++;

          if (y_current_step_ > y_total_steps_)
          {
            phase = FlightPhase::ASCEND_TO_LEVEL7;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Ascending to level 7.");
          }
          break;
        }

        case FlightPhase::ASCEND_TO_LEVEL7: {
          z_total_steps_ = static_cast<int>(std::ceil(std::abs(z_level7 - (z_level6)) / step_movement_));
          float z = z_level6 - step_movement_ * z_current_step_;
          if (z <= z_level7)
            z = z_level7;
          publish_position_setpoint(0.0, y_beginning_, z);
          z_current_step_++;
          if (z_current_step_ > z_total_steps_)
          {
            phase = FlightPhase::SCAN_LEVEL7;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Starting rack scan at Level 7");
          }
          break;
        }

        case FlightPhase::SCAN_LEVEL7: {
          float y = y_beginning_ + step_movement_ * y_current_step_;
          if (y >= y_end_)
            y = y_end_;
          publish_position_setpoint(0.0, y, z_level7);
          y_current_step_++;

          if (y_current_step_ > y_total_steps_)
          {
            phase = FlightPhase::ASCEND_TO_LEVEL8;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Ascending to level 8.");
          }
          break;
        }

        case FlightPhase::ASCEND_TO_LEVEL8: {
          z_total_steps_ = static_cast<int>(std::ceil(std::abs(z_level8 - (z_level7)) / step_movement_));
          float z = z_level7 - step_movement_ * z_current_step_;
          if (z <= z_level8)
            z = z_level8;
          publish_position_setpoint(0.0, y_end_, z);
          z_current_step_++;
          if (z_current_step_ > z_total_steps_)
          {
            phase = FlightPhase::SCAN_LEVEL8;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Starting rack scan at Level 8");
          }
          break;
        }

        case FlightPhase::SCAN_LEVEL8: {
          float y = y_end_ - step_movement_ * y_current_step_;
          if (y <= y_beginning_)
            y = y_beginning_;
          publish_position_setpoint(0.0, y, z_level8);
          y_current_step_++;

          if (y_current_step_ > y_total_steps_)
          {
            phase = FlightPhase::ASCEND_TO_LEVEL9;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Ascending to level 9.");
          }
          break;
        }

        case FlightPhase::ASCEND_TO_LEVEL9: {
          z_total_steps_ = static_cast<int>(std::ceil(std::abs(z_level9 - (z_level8)) / step_movement_));
          float z = z_level8 - step_movement_ * z_current_step_;
          if (z <= z_level9)
            z = z_level9;
          publish_position_setpoint(0.0, y_beginning_, z);
          z_current_step_++;
          if (z_current_step_ > z_total_steps_)
          {
            phase = FlightPhase::SCAN_LEVEL9;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Starting rack scan at Level 9");
          }
          break;
        }

        case FlightPhase::SCAN_LEVEL9: {
          float y = y_beginning_ + step_movement_ * y_current_step_;
          if (y >= y_end_)
            y = y_end_;
          publish_position_setpoint(0.0, y, z_level9);
          y_current_step_++;

          if (y_current_step_ > y_total_steps_)
          {
            phase = FlightPhase::ASCEND_TO_LEVEL10;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Ascending to level 10.");
          }
          break;
        }

        case FlightPhase::ASCEND_TO_LEVEL10: {
          z_total_steps_ = static_cast<int>(std::ceil(std::abs(z_level10 - (z_level9)) / step_movement_));
          float z = z_level9 - step_movement_ * z_current_step_;
          if (z <= z_level10)
            z = z_level10;
          publish_position_setpoint(0.0, y_end_, z);
          z_current_step_++;
          if (z_current_step_ > z_total_steps_)
          {
            phase = FlightPhase::SCAN_LEVEL10;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Starting rack scan at Level 10");
          }
          break;
        }

        case FlightPhase::SCAN_LEVEL10: {
          float y = y_end_ - step_movement_ * y_current_step_;
          if (y <= y_beginning_)
            y = y_beginning_;
          publish_position_setpoint(0.0, y, z_level10);
          y_current_step_++;

          if (y_current_step_ > y_total_steps_)
          {
            phase = FlightPhase::RETURN;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Returning to low height");
          }
          break;
        }

        case FlightPhase::RETURN: {
          z_total_steps_ = static_cast<int>(std::ceil(std::abs(z_takeoff - (z_level10)) / step_movement_));
          float z = z_level10 + step_movement_ * z_current_step_;
          if (z >= z_takeoff)
            z = z_takeoff;
          publish_position_setpoint(0.0, y_beginning_, z);
          z_current_step_++;
          if (z_current_step_ > z_total_steps_)
          {
            phase = FlightPhase::LAND;
            reset_step_variables();
            RCLCPP_INFO(this->get_logger(), "Landing");
          }
          break;
        }

        case FlightPhase::LAND:
          publish_landing_setpoint();
          if (timer_tick_count_ >= hold_ticks_)
          {
            this->disarm();
            phase = FlightPhase::DONE;
            RCLCPP_INFO(this->get_logger(), "Landing complete. DISARMED.");
          }
          break;

        case FlightPhase::DONE:
          // Stop timer if desired, or continue holding position
          timer_tick_count_ = 0;
          break;
      }
    });
  }

  void arm();
  void disarm();

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

  std::atomic<uint64_t> timestamp_;  //!< common synced timestamped

  uint64_t offboard_setpoint_counter_;  //!< counter for the number of setpoints sent

  void publish_offboard_control_mode();
  void publish_takeoff_setpoint();
  void publish_landing_setpoint();
  void publish_position_setpoint();
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

  enum class FlightPhase
  {
    INIT,
    TAKEOFF,
    TRY_X_PLUS_POS,
    ASCEND_TO_LEVEL1,
    SCAN_LEVEL1,
    ASCEND_TO_LEVEL2,
    SCAN_LEVEL2,
    ASCEND_TO_LEVEL3,
    SCAN_LEVEL3,
    ASCEND_TO_LEVEL4,
    SCAN_LEVEL4,
    ASCEND_TO_LEVEL5,
    SCAN_LEVEL5,
    ASCEND_TO_LEVEL6,
    SCAN_LEVEL6,
    ASCEND_TO_LEVEL7,
    SCAN_LEVEL7,
    ASCEND_TO_LEVEL8,
    SCAN_LEVEL8,
    ASCEND_TO_LEVEL9,
    SCAN_LEVEL9,
    ASCEND_TO_LEVEL10,
    SCAN_LEVEL10,
    RETURN,
    LAND,
    DONE
  };

  FlightPhase phase = FlightPhase::INIT;
  int timer_tick_count_ = 0;

  // Hold time at each waypoint (in timer ticks)
  const int hold_ticks_ = 20;  // e.g., 4 seconds if timer_interval_ms_ = 200ms

  const float scan_speed_ = 0.5;       // m/s
  const int timer_interval_ms_ = 200;  // ms
  const float step_movement_ = scan_speed_ * (timer_interval_ms_ / 1000.0f);
  int y_total_steps_;
  int y_current_step_ = 0;
  int z_total_steps_;
  int z_current_step_ = 0;

  const float y_beginning_ = 0.0;
  const float y_end_ = 10.0;
  const float z_takeoff = -0.5;
  const float z_level1 = -1.2;
  const float z_level2 = -2.4;
  const float z_level3 = -3.6;
  const float z_level4 = -4.8;
  const float z_level5 = -6.0;
  const float z_level6 = -7.2;
  const float z_level7 = -8.4;
  const float z_level8 = -9.6;
  const float z_level9 = -10.8;
  const float z_level10 = -12.0;

  void publish_position_setpoint(float x, float y, float z)
  {
    TrajectorySetpoint msg{};
    msg.position = { x, y, z };
    msg.yaw = 0.0;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
  }

  void reset_step_variables()
  {
    timer_tick_count_ = 0;
    z_total_steps_ = 0;
    z_current_step_ = 0;
    y_current_step_ = 0;
  }
};

void OffboardControl::publish_takeoff_setpoint()
{
  TrajectorySetpoint msg{};
  msg.position = { 0.0, 0.0, -3 };
  msg.yaw = 0.0;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_landing_setpoint()
{
  TrajectorySetpoint msg{};
  msg.position = { 0.0, 0.0, 0.0 };
  msg.yaw = 0.0;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::arm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
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

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
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
  std::cout << "Starting offboard control node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>());

  rclcpp::shutdown();
  return 0;
}
