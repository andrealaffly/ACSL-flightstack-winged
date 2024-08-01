/***********************************************************************************************************************
 * Copyright (c) 2024 Giri M. Kumar, Mattia Gramuglia, Andrea L'Afflitto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * File:        control.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 12, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class declaration for computing and publishing the control. Adds as a hub for all the control actions.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL_flightstack_X8.git
 **********************************************************************************************************************/

#ifndef CONTROL_HPP_
#define CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"   // ROS2 Client Library C++ header
#include "vehicle_class.hpp"
#include <px4_msgs/msg/actuator_motors.hpp>
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <bit>

#include "flight_bridge.hpp"
#include "flight_params.hpp"
#include "control_config.hpp"

/// ------------- USER DEFINED ------------- ///
// -> Pixhawk vehicle mode command message pacakge definitions
inline constexpr uint8_t TARGET_SYSTEM = 1;
inline constexpr uint8_t TARGET_COMPONENT = 1;
inline constexpr uint8_t SOURCE_SYSTEM = 1;
inline constexpr uint16_t SOURCE_COMPONENT = 1;
inline constexpr bool FROM_EXTERNAL_BOOL = true;
// -> Set Custom mode
inline constexpr float ENABLE_CUSTOM_MODE = 1;
// -> Custom Control mode definitions for pixhawk vehicle mode command
inline constexpr float CUSTOM_MAIN_MODE_MANUAL = 1;
inline constexpr float CUSTOM_MAIN_MODE_ALTCTL = 2;
inline constexpr float CUSTOM_MAIN_MODE_POSCTL = 3;
inline constexpr float CUSTOM_MAIN_MODE_AUTO = 4;
inline constexpr float CUSTOM_MAIN_MODE_ACRO = 5;
inline constexpr float CUSTOM_MAIN_MODE_OFFBOARD = 6;
inline constexpr float CUSTOM_MAIN_MODE_STABILIZED = 7;
inline constexpr float CUSTOM_MAIN_MODE_RATTITUDE = 8;
// -> ARM/DISARM defines
inline constexpr float ARM_ACSL_VEH = 1.0;
inline constexpr float DISARM_ACSL_VEH = 0.0;
// -> OFFBOARD_CONTROL_MODE define
inline constexpr bool OFFBOARD_POS_CTRL_BOOL = false;
inline constexpr bool OFFBOARD_VEL_CTRL_BOOL = false;
inline constexpr bool OFFBOARD_ACCEL_CTRL_BOOL = false;
inline constexpr bool OFFBOARD_ATT_CTRL_BOOL = false;
inline constexpr bool OFFBOARD_BODY_RATE_CTRL_BOOL = false;
inline constexpr bool OFFBOARD_THRST_TRQ_CTRL_BOOL = false;
inline constexpr bool OFFBOARD_DIRECT_ACT_CTRL_BOOL = true;
// -> Zero Thrust define for motors
inline constexpr float MOTOR_ZERO_THRUST = 0.0;

// -> Define for publishing data to motors
#define PUBLISH_ACTUATOR true


namespace _control_
{

class controlNode : public rclcpp::Node
{
public:
  /// rclcpp::Node needs a default constructor. 
  /// Therefore we implement passing of the vehicle states like so
  controlNode(vehicle_states* d, flight_params* p);

  virtual ~controlNode() = default;

private: 
  void control_timer_callback_();

  double get_rk4_timestep();

  float get_min_thrust();

  float get_zero_thrust();

  /// Common synced initial timestamp - synced with controlNode
  std::atomic<uint64_t> timestamp_initial_;   

  /// Current time
  std::atomic<double> time_current_; 

  /// Update function for the current time
  void updateCurrentTime();

  /// Timer for running the control node at a fixed rate
  rclcpp::TimerBase::SharedPtr control_timer_;

  /// Publisher definitions
  // -> Publisher for actuator motor commands
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr publisher_actuator_motors_;
  // -> Publisher for vehicle commands
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_vehicle_command_;
  // -> Publisher for switching to offboard control mode
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr publisher_offboard_control_mode_;

  uint offboard_flag_; // Flag for the arm/disarm logic

  /// Function that runs the control loop
  void run();

  /// Function that computes the control from the control algorithms and publishes them
  void compute_and_publish_control();

  /// Function to arm the vehicle
  void arm();

  /// Function to disarm the vehicle
  void disarm();

  /// Function to publish offboard control mode
  void publish_offboard_control_mode();

  /// Function to publish vehicle command 
  // -> switches to offboard mode and arms the vehicle
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

  /// Function to publish motor thrust commands
  void publish_actuator_motors(const float t1,
                               const float t2,
                               const float t3,
                               const float t4,
                               const float t5,
                               const float t6,
                               const float t7,
                               const float t8);

  /// Instance of the vehicle_ptr to point to the vehicle state object in
  /// flight_bridge.cpp
  vehicle_states* vehicle_ptr;

  /// Instance of the flight_params_ptr to point to the flight_parameter object in
  /// flight_main.cpp
  flight_params* flight_params_ptr;

  /// Function to debug vehicle states to the screen.
  void debug2screen();

  /// ------- CREATE CONTROLLER OBJECT -------- ///
  _picked_controller_ controller_;

};
}    // namespace _control_



#endif  // CONTROL_HPP_