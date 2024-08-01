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
 * File:        flight_params.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 30, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Define file for the main flight parameters that will be
 *              used for the entire controller.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL_flightstack_X8.git
 **********************************************************************************************************************/

#include "flight_params.hpp"

namespace _flight_params_
{
    // Define the parameter reader function
    flight_params FlightConfigReader::readFlightConfig(const std::string& jsonFile)
    {
        std::ifstream file(jsonFile);
        json j;
        file >> j;

        flight_params read;
        read.arm_time = j["FlightMainParams"]["arm_time"];
        read.min_thrust_after_arm = j["FlightMainParams"]["min_thrust_after_arm"];
        read.start_time = j["FlightMainParams"]["start_time"];
        read.user_defined_trajectory_file = j["FlightMainParams"]["user_defined_trajectory_file"].get<std::string>();
        read.hover_time = j["FlightMainParams"]["hover_after_trajectory_time_seconds"];
        read.mocap = j["FlightMainParams"]["mocap"];
        read.controller_rate = j["FlightMainParams"]["controller_rate"];

        // Read in the trajectory info from the trajectory file.
        read.traj_params = readTrajectoryConfig(read.user_defined_trajectory_file);

        // Set the landing start time
        // This will be sum of the start_time, the end of the flight_trajectory + the time needed to hover.
        read.landing_start_time = read.start_time + read.traj_params.waypoint_times_.back() + read.hover_time;

        // Set the landing end time
        // This will be the landing_start_time + 4.0 seconds.
        read.landing_end_time = read.landing_start_time + 4.0;

        // Define the end time 
        // Disarm after the flight ends. this will be a second after landing.
        read.end_time = read.landing_end_time + 1.0;

        return read;
    }

    // Define the user defined Piecewise Polynomial trajectory reader function
    PiecewisePolynomialTrajectoryInfo FlightConfigReader::readTrajectoryConfig(const std::string& fileName)
    {
         // Define the path where the user-defined trajectory JSON files are located
        const std::string path = "./src/acsl_flight/params/user_defined_trajectory/";

        // Concatenate the path with the file name
        std::string jsonFile = path + fileName;

        // Create objecto to return
        PiecewisePolynomialTrajectoryInfo piecewise_polynomial_trajectory_info_;

        std::ifstream file(jsonFile);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file: " + jsonFile);
        }

        nlohmann::json j;
        file >> j;
        
        piecewise_polynomial_trajectory_info_.waypoint_times_ = 
            j["waypoint_times"].get<std::vector<double>>();
        
        piecewise_polynomial_trajectory_info_.piecewise_polynomial_coefficients_ = 
            j["piecewise_polynomial_coefficients"].get<std::vector<std::vector<double>>>();
        
        piecewise_polynomial_trajectory_info_.num_waypts = 
            piecewise_polynomial_trajectory_info_.waypoint_times_.size();

        return piecewise_polynomial_trajectory_info_;
    }
}