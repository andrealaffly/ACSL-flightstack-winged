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
 * Date:        June 20, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Just a header file for configuring which platform and 
 *              controller to pick during compilation.
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
#define __ROSTESTDRONE__ 2

/*********************************************************************************************************************
  CONTROLLER selection
**********************************************************************************************************************
*/
// Define named constants for controller types
#define __PID__ 1
#define __MRAC_PID__ 2
#define __PID_OMEGA__ 3
#define __MRAC_OMEGA__ 4

// SELECT here the PLATFORM you are using ---------------------------------------------------------------------------
#define SELECTED_PLATFORM __QRBP__

// SELECT here the CONTROLLER you want to run -----------------------------------------------------------------------
#define SELECTED_CONTROLLER __PID_OMEGA__


/// ------- INITIALIZING THE CONTROLLER -------- ///
#if SELECTED_PLATFORM == __QRBP__

    // Define the platform of choice for sending to the mocap file.
    // This directory name should be used in the logger for all your controllers! - DO NOT FORGET
    constexpr const char* platform_of_choice = "qrbp"; 

    // Define ControlType based on SELECTED_CONTROLLER using type aliasing
    #if SELECTED_CONTROLLER == __PID__ 

        #include "PID.hpp"
        // Remember the class name you used in control_algorithms
        using _picked_controller_ = _qrbp_::_pid_::pid;    
        

    #elif  SELECTED_CONTROLLER == __MRAC_PID__ 

        #include "MRAC_PID.hpp"
        // Remember the class name you used in control_algorithms
        using _picked_controller_ = _qrbp_::_mrac_pid_::mrac_pid;    

    #elif SELECTED_CONTROLLER == __PID_OMEGA__

        #include "PID_OMEGA.hpp"
        // Remember the class name you used in control_algorithms
        using _picked_controller_ = _qrbp_::_pid_omega_::pid_omega;    

    #elif SELECTED_CONTROLLER == __MRAC_OMEGA__

        #include "MRAC_OMEGA.hpp"
        // Remember the class name you used in control_algorithms
        using _picked_controller_ = _qrbp_::_mrac_omega_::mrac_omega;

    #else 

        #error "ERROR: Unsupported controller type selected"

    #endif

#elif SELECTED_PLATFORM == __ROSTESTDRONE__

    // Define the platform of choice for sending to the mocap file.
    // This directory name should be used in the logger for all your controllers! - DO NOT FORGET
    constexpr const char* platform_of_choice = "rostestdrone";

    // Define ControlType based on SELECTED_CONTROLLER using type aliasing
    #if SELECTED_CONTROLLER == __PID__ 

        #include "PID_TD.hpp"
        // Remember the class name you used in control_algorithms
        using _picked_controller_ = _rostestdrone_::_pid_::pid;    

    #else 

        #error "ERROR: Unsupported controller type selected"

    #endif

#else 

    #error "ERROR: Platform not defined"

#endif


#endif  // CONTROL_CONFIG_HPP_