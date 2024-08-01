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
 * File:        flight_main.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 16, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Main function to initiate ROS2.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL_flightstack_X8.git
 **********************************************************************************************************************/

#include "flight_main.hpp"

using namespace _flight_bridge_;


int main(int argc, char * argv[])
{

    std::cout << "===================== STARTING ACSL FLIGHTSTACK ===================== \n" << std::endl;

    // Parse the flight parameter settings for the flight options.
    // OPTIONS: 
    // 1. Arm Time
    // 2. Minimum Thrust After Arm
    // 3. Flight Start time
    // 4. Flight End time
    // 5. Type of State Estimation
    // 6. Type of Controller used in flight
    // 7. Controller Period
    
    // Intialzie the reader object for reading the flight parameters
    FlightConfigReader reader;  
    // Read in the flight parameters
    flight_params run_params = reader.readFlightConfig("./src/acsl_flight/params/flight_params/flight_main_params.json");

    std::cout << "Flying Under Parameters: " << std::endl;
    std::cout << "Arm Time: " << run_params.arm_time << std::endl;
    std::cout << "Start Time: " << run_params.start_time << std::endl;
    std::cout << "End Time: " << run_params.end_time << std::endl;
    std::cout << "Mocap: " << run_params.mocap << std::endl;
    std::cout << "controller_period: " << run_params.controller_rate << "ms" << std::endl;
    std::cout << "\n" << std::endl;

    // Initialize ROS2
    std::cout << "Initializing ROS2" << std::endl;
    rclcpp::init(argc, argv);    

    // Initialize flight_bridge
    flight_bridge flight(&run_params);

    // Intialize the nodes
    flight.init();
   
    rclcpp::shutdown();
}