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
 * File:        control_config.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        January 20, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for configuring which platform and controller to pick during compilation.
 *              Used by control.hpp (and through it flight_bridge).
 *              >> Do NOT include this from control_algorithm_base.hpp, the trajectory headers or any controller
 *              header: it includes the controller headers, so that would create an include cycle. <<
 *              Global build settings (UDP, SIMULATION_LOCKSTEP, feature flags) are in global_config.hpp.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef CONTROL_CONFIG_HPP_
#define CONTROL_CONFIG_HPP_

/*********************************************************************************************************************
  PLATFORM selection
**********************************************************************************************************************
*/
// Define named constants for controller types
#define __QRBP__ 1
#define __QUADM__ 2

/*********************************************************************************************************************
  CONTROLLER selection
**********************************************************************************************************************
*/
// Define named constants for controller types
#define __PID__            1
#define __MRAC_PID__       2
#define __PID_OMEGA__      3
#define __MRAC_OMEGA__     4
#define __MRAC_OBSERVER__  6
#define __MRAC_GEOMETRIC__ 7

// SELECT here the PLATFORM you are using ---------------------------------------------------------------------------
#define SELECTED_PLATFORM __QUADM__

// SELECT here the CONTROLLER you want to run -----------------------------------------------------------------------
#define SELECTED_CONTROLLER __MRAC_GEOMETRIC__

/*********************************************************************************************************************
  HELPER BLOCK OF CODE - MODIFY ONLY IF THERE ARE CHANGES TO CONTROLLERS
**********************************************************************************************************************
*/
/// ------- INITIALIZING THE CONTROLLER -------- ///
#if SELECTED_PLATFORM == __QRBP__

    // Define the platform of choice for sending to the mocap file.
    // This directory name should be used in the logger for all your controllers! - DO NOT FORGET
    // SIMULATION LOCKSTEP: must also match the platform key in the simulator's sim-config.yaml (handshake check).
    constexpr const char* platform_of_choice = "qrbp"; 

    // Define ControlType based on SELECTED_CONTROLLER using type aliasing
    #if SELECTED_CONTROLLER == __PID__ 

        #include "PID.hpp"
        // Remember the class name you used in control_algorithms
        using _picked_controller_ = _qrbp_::_pid_::pid;    
        // Lockstep handshake name, must match the controller key in the simulator's sim-config.yaml
        constexpr const char* controller_of_choice = "pid";
        
    #elif  SELECTED_CONTROLLER == __MRAC_PID__ 

        #include "MRAC_PID.hpp"
        // Remember the class name you used in control_algorithms
        using _picked_controller_ = _qrbp_::_mrac_pid_::mrac_pid;    
        // Lockstep handshake name, must match the controller key in the simulator's sim-config.yaml
        constexpr const char* controller_of_choice = "mrac_pid";

    #elif SELECTED_CONTROLLER == __PID_OMEGA__

        #include "PID_OMEGA.hpp"
        // Remember the class name you used in control_algorithms
        using _picked_controller_ = _qrbp_::_pid_omega_::pid_omega;    
        // Lockstep handshake name, must match the controller key in the simulator's sim-config.yaml
        constexpr const char* controller_of_choice = "pid_omega";

    #elif SELECTED_CONTROLLER == __MRAC_OMEGA__

        #include "MRAC_OMEGA.hpp"
        // Remember the class name you used in control_algorithms
        using _picked_controller_ = _qrbp_::_mrac_omega_::mrac_omega;
        // Lockstep handshake name, must match the controller key in the simulator's sim-config.yaml
        constexpr const char* controller_of_choice = "mrac_omega";

    #elif SELECTED_CONTROLLER == __MRAC_OBSERVER__
        #include "MRAC_OBSERVER.hpp"
        // Remember the class name you used in control_algorithms
        using _picked_controller_ = _qrbp_::_mrac_observer_::mrac_observer;
        // Lockstep handshake name, must match the controller key in the simulator's sim-config.yaml
        constexpr const char* controller_of_choice = "mrac_observer";

    #else 

        #error "ERROR: Unsupported controller type selected"

    #endif

#elif SELECTED_PLATFORM == __QUADM__

    // Define the platform of choice for sending to the mocap file.
    // This directory name should be used in the logger for all your controllers! - DO NOT FORGET
    // SIMULATION LOCKSTEP: must also match the platform key in the simulator's sim-config.yaml (handshake check).
    constexpr const char* platform_of_choice = "quadm";

    // Define ControlType based on SELECTED_CONTROLLER using type aliasing
    #if SELECTED_CONTROLLER == __PID__ 

        #include "PID_TD.hpp"
        // Remember the class name you used in control_algorithms
        using _picked_controller_ = _quadm_::_pid_::pid;    
        // Lockstep handshake name, must match the controller key in the simulator's sim-config.yaml
        constexpr const char* controller_of_choice = "pid";

    #elif SELECTED_CONTROLLER == __MRAC_GEOMETRIC__

        #include "MRAC_GEOMETRIC_TD.hpp"
        // Remember the class name you used in control_algoritms
        using _picked_controller_ = _quadm_::_mrac_geometric_::mrac_geometric;
        // Lockstep handshake name, must match the controller key in the simulator's sim-config.yaml
        constexpr const char* controller_of_choice = "mrac_geometric";

    #else 

        #error "ERROR: Unsupported controller type selected"

    #endif

#else 

    #error "ERROR: Platform not defined"

#endif

#endif  // CONTROL_CONFIG_HPP_
