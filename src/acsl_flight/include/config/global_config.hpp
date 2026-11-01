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
 * File:        global_config.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        June 20, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Global build configuration: UDP networking, the SIMULATION LOCKSTEP switch, and the compile-time
 *              feature flags (_config_param_).
 *              >> LEAF HEADER: must NOT include any project header. << It is included by low-level headers
 *              (piecewise_polynomial_trajectory.hpp, control_algorithm_base.hpp) that the controller headers include,
 *              so including a controller from here would create an include cycle. Platform and controller
 *              selection lives in control_config.hpp.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef GLOBAL_CONFIG_HPP_
#define GLOBAL_CONFIG_HPP_

#include <cstdint>   // For uint16_t
#include <string>    // For const char*

/*********************************************************************************************************************
  UDP Parameter selection
**********************************************************************************************************************
*/
// IPs for UDP communication
inline constexpr const char* GROUND_STATION_IP = "127.0.0.1";   // <-- For testing only
inline constexpr const char* ODROID_M1S_IP = "192.168.12.1";    // <-- For real flights / testing 

// Ports for UDP communication 
inline constexpr uint16_t MOCAP_PORT = 52000;   // <-- Make sure that this matches the port in your mocap software
inline constexpr uint16_t WEATHER_PORT = 53000; // <-- Confirm this matches the port in the yaml file in accessories

// BOOLEAN FOR SPOOFING IP ==== TRUE -> GROUND_STATION_IP (testing code) || FALSE -> ODROID_M1S_IP (flights)
#define TESTING_UDP_COMMUNICATION false

/*********************************************************************************************************************
  SIMULATION LOCKSTEP selection
**********************************************************************************************************************
*/
// 0 -> normal flight build (default). Trajectory from piecewise_polynomial_trajectory (the trajectory file),
//      flight_bridge launches the usual odometry / control / mocap nodes talking to PX4.
// 1 -> simulation lockstep build. The simulator (acsl-chrono-simulator) owns the trajectory, physics and clock:
//      flight_bridge launches only the lockstep node, the trajectory class is swapped for lockstep_trajectory
//      (reference injected by the simulator every step), and only the controller runs on this computer.
//
// A #define (not an inline constexpr bool like the flags below) because it is used in preprocessor #if blocks to
// choose class declarations and which nodes/headers are compiled.
// piecewise_polynomial_trajectory.hpp includes this header, so every file that uses the trajectory class sees this
// switch. That rules out silently compiling the wrong trajectory.
// Rebuild (colcon) after changing it. utils/start_lockstep.sh refuses to start unless it is 1.
// NEVER fly a binary built with 1: it has no trajectory of its own.
#define SIMULATION_LOCKSTEP 0

/*********************************************************************************************************************
  FEATURE FLAGS (compile-time)
**********************************************************************************************************************
*/
// Kept here (not in control_config.hpp) because the controllers reach them through control_algorithm_base.hpp,
// which must not include control_config.hpp (include cycle, see the description above).
namespace _config_param_
{
  // Flag for enable/disable publishing data to motors
  inline constexpr bool PUBLISH_ACTUATOR  = true;

  // Flag to enable/disable the MRAC Projection Operator
  inline constexpr bool USE_PROJECTION_OPERATOR = true;

  // Flag to enable/disable using a sine wave function for 
  // the inner loop desired orientation.
  inline constexpr bool USE_SINEWAVE_INL_CMD = false;

}

/*********************************************************************************************************************
  HELPER BLOCK OF CODE - MODIFY ONLY IF THERE ARE CHANGES TO UDP NETWORKING
**********************************************************************************************************************
*/
/// ------- INITIALIZING THE UDP COMMUNICATION -------- ///
// Block of code that decides if we are testing using the ground station or running the weather capture on the ODroid.
#if TESTING_UDP_COMMUNICATION
  // For testing with internal port on groundstation
  inline constexpr const char* IP_IN_USE = GROUND_STATION_IP;
#else
  // For implementation with VICON and ODroid M1s
  inline constexpr const char* IP_IN_USE = ODROID_M1S_IP;
#endif


#endif  // GLOBAL_CONFIG_HPP_
