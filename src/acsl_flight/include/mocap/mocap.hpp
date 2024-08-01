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

 /**********************************************************************************************************************  
 * Part of the code in this file leverages the following material.
 *
 * Referenced:  https://github.com/ros-drivers/transport_drivers/tree/main
 *              Copyright 2021 LeoDrive.
 *            
 *              Licensed under the Apache License, Version 2.0 (the "License");
 *              you may not use this file except in compliance with the License.
 *              You may obtain a copy of the License at
 *               
 *                  http://www.apache.org/licenses/LICENSE-2.0
 *              
 *              Unless required by applicable law or agreed to in writing, software
 *              distributed under the License is distributed on an "AS IS" BASIS,
 *              WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *              See the License for the specific language governing permissions and
 *              limitations under the License.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * File:        mocap.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 20, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class declaration for UDP socket as a lifecycle node.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL_flightstack_X8.git
 **********************************************************************************************************************/

#ifndef MOCAP_HPP_
#define MOCAP_HPP_

#include "udp_driver.hpp"
#include "control_config.hpp"                     // Include this for creating the logging file.

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <bit>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "vehicle_class.hpp"
#include "flight_log.hpp"

#define TESTING false

// -> IP of the Odroid M1s
inline constexpr const char* GROUND_STATION_IP = "127.0.0.1"; // For testing 
inline constexpr const char* ODROID_M1S_IP = "192.168.12.1";
// -> PORT of the Odroid M1s
inline constexpr uint16_t ODROID_M1S_PORT = 52000;

// Block of code that decides if we are testing using the ground station or running the mocap on the ODroid.
#if TESTING
  // For testing with internal port on groundstation
  inline constexpr const char* IP_IN_USE = GROUND_STATION_IP;
#else
  // For implementation with VICON and ODroid M1s
  inline constexpr const char* IP_IN_USE = ODROID_M1S_IP;
#endif

/// ------------- USER DEFINED ------------- ///
// -> Pixhawk vehicle odometry pose frame message package definitions
inline constexpr uint8_t POSE_FRAME_UNKNOWN = 0; // Pose frame unkown
inline constexpr uint8_t POSE_FRAME_NED     = 1; // NED earth-fixed frame
inline constexpr uint8_t POSE_FRAME_FRD     = 2; // FRD world-fixed frame, arbitrary heading reference
// -> Pixhawk vehicle odometry velocity framemessage pacakge definitions
inline constexpr uint8_t VELOCITY_FRAME_UNKNOWN = 0;  // Velocity frame unkown
inline constexpr uint8_t VELOCITY_FRAME_NED      = 1; // NED earth-fixed frame
inline constexpr uint8_t VELOCITY_FRAME_FRD      = 2; // FRD world-fixed frame, arbitrary heading reference
inline constexpr uint8_t VELOCITY_FRAME_BODY_FRD = 3; // FRD body-fixed frame

namespace lc = rclcpp_lifecycle;
namespace fl = _flight_log_;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace _drivers_
{
namespace _udp_driver_
{

// Struct for mocap states that come over the udp.
struct mocap_states
{
  double control_time;
  double x; 
  double y;
  double z;
  double q0;
  double q1;
  double q2;
  double q3;
  double vx;
  double vy;
  double vz;
  double rollspeed;
  double pitchspeed;
  double yawspeed;
};

/// \brief UdpReceiverNode class which can receive UDP datagrams
class UdpReceiverNode final
  : public lc::LifecycleNode, public fl::blackbox
{
public:
    /// \brief Constructor which accepts IoContext
    /// \param[in] ctx A shared IoContext
    /// \param[in] pointer to vehicle states in flight_bridge
    UdpReceiverNode(const IoContext & ctx, vehicle_states* d);

    /// \brief Destructor - required to manage owned IoContext
    ~UdpReceiverNode();

    /// \brief Callback from transition to "configuring" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_configure(const lc::State & state) override;

    /// \brief Callback from transition to "activating" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_activate(const lc::State & state) override;

    /// \brief Callback from transition to "deactivating" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_deactivate(const lc::State & state) override;

    /// \brief Callback from transition to "unconfigured" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_cleanup(const lc::State & state) override;

    /// \brief Callback from transition to "shutdown" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_shutdown(const lc::State & state) override;

    /// \brief Callback for receiving a UDP datagram
    void receiver_callback(const std::vector<uint8_t> & buffer);

private: 
    /// \brief Get the parameters for the ip and port to ping
    void get_params();

    /// Pointer to the asio context owned by this node for async communication
    std::unique_ptr<IoContext> m_owned_ctx{};
    
    /// String for the ip of the odrioid
    std::string m_ip{};
        
    /// String for the port of the odroid
    uint16_t m_port{};
    
    /// Pointer for the udp driver which wraps the udp socket
    std::unique_ptr<UdpDriver> m_udp_driver;
    
    /// Publisher for publishing the mocap message
    lc::LifecyclePublisher<px4_msgs::msg::VehicleOdometry>::SharedPtr mocap_publisher_;

    /// Instance of the vehicle_ptr to point to the vehicle state object in
    /// flight_bridge.cpp
    vehicle_states* vehicle_ptr;

    /// Declare variables to store parsed data and declare it as interal members "_im"
    mocap_states mc_im;

    /// \brief Debugger function to output the mocap data.
    void debugMocapData2screen();

    /// \brief Implementing virtual functions from blackbox
    void logInitHeaders();
    bool logInitLogging();
    void logLogData();


};  

}   // namespace _udp_driver_
}   // namesapce _drivers_

#endif  // MOCAP_HPP_