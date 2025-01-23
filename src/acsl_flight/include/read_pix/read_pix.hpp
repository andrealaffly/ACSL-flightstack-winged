///@cond
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
///@endcond
/***********************************************************************************************************************
 * File:        read_pix.hpp \n 
 * Author:      Giri Mugundan Kumar \n 
 * Date:        April 12, 2024 \n 
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class declaration for reading pixhawk data.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef READ_PIX_HPP_
#define READ_PIX_HPP_

/**
 * @file read_pix.hpp
 * @brief Class decleartion for reading pixhawk data
 */

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>
#include <functional>
#include <atomic>

#include <Eigen/Dense>                        // For quaternion operation

#include "rclcpp/rclcpp.hpp"                  // ROS2 Client Library C++ header
#include "std_msgs/msg/int32.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "vehicle_class.hpp"
#include "flight_params.hpp"

using namespace Eigen;

namespace _read_pix_
{

/***********************************************************************************************/
/*                                  READ ODOMETRY CLASS                                        */
/***********************************************************************************************/

/**
 * @class read_odometryNode
 * @brief READ ODOMETRY CLASS
 */
class read_odometryNode : public rclcpp::Node
{
public:
  /// rclcpp::Node needs a default constructor. 
  /// Therefore we implement passing of the vehicle states like so
  read_odometryNode(vehicle_states* d, flight_params* p);

  virtual ~read_odometryNode() = default;

private:
  /// Odometry Subcriber pointer
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription_;

  /// Odometry callback function to process and save data to vehicle obejct in flight_bridge.cpp
  /**
   * @brief Odometry callback function to process and save data to vehicle obejct in flight_bridge.cpp
   * @param msg
   */
  void odometry_callback_(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);

  /// Instance of the vehicle_ptr to point to the vehicle state object in
  /// flight_bridge.cpp
  vehicle_states* vehicle_ptr;

  /// Instance of the flight_params_ptr to point to the flight_parameter object in
  /// flight_main.cpp
  flight_params* flight_params_ptr;

  // Prototype function for calculating the euler angles from quaternions
  std::array<double, 3> quaternionToEulerAnglesRPY(double q0, double q1, double q2, double q3);

  // Private variable for initial vehicle state.
  /**
   * @struct init_state
   * @brief Private variable for initial vehicle state.
   */
  struct init_state
  {
    bool init = false;
    
    // Member variables
    double x = 0.0;                     // position along x-axis in I - NED earth-fixed frame [m]
    double y = 0.0;                     // position along y-axis in I - NED earth-fixed frame [m]
    double z = 0.0;                     // position along z-axis in I - NED earth-fixed frame [m]
    std::array<double, 3> EulerRPY;     // Euler Roll Pitch Yaw for computation
    double roll = 0.0;                  // Euler Roll  [rad]
    double pitch = 0.0;                 // Euler Pitch [rad]
    double yaw = 0.0;                   // Euler Yaw   [rad]
    Quaterniond yaw_quaternion;         // Quaternion offset in yaw in I - NED earth-fixed frame [-]
  } initvals; 

  // Raw quaternion data from the  for computation
  Quaterniond quaternion_pix;

  // Zeroed in yaw quaternion to write to pixhawk
  Quaterniond zeroed_quaternion;

  // Euler Roll Pitch Yaw for computation
  std::array<double, 3> EulerRPY;

};

}  // namespace read_pix

#endif  // READ_PIX_HPP_