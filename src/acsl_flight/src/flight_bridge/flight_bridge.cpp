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
 * Referenced:  https://github.com/ros2/examples/tree/galactic/rclcpp/executors/cbg_executor
 *              Copyright (c) 2020 Robert Bosch GmbH
 *
 *              Licensed under the Apache License, Version 2.0 (the "License");
 *              you may not use this file except in compliance with the License.
 *              You may obtain a copy of the License at
 *
 *                        http://www.apache.org/licenses/LICENSE-2.0
 *
 *              Unless required by applicable law or agreed to in writing, software
 *              distributed under the License is distributed on an "AS IS" BASIS,
 *              WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *              See the License for the specific language governing permissions and
 *              limitations under the License.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * File:        flight_bridge.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 12, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Utilities for multithreading.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL_flightstack_X8.git
 **********************************************************************************************************************/

#include "flight_bridge.hpp"

namespace _flight_bridge_{

flight_bridge::flight_bridge(flight_params* p) : flight_params_ptr(p)
{
    std::cout << "Starting Flight Bridge" << std::endl;
}

void flight_bridge::init()
{

    /// ---------------------------------- MOCAP EKF VISION FUSION MODE ---------------------------------------- ///
    if (flight_params_ptr->mocap)
    {
        // Boolean for the mocap node
        bool receiver_started{false};

        // Create Executors -------------------------------------------------------------------- >>
        // -> High priority odometry callback executor.
        rclcpp::executors::StaticSingleThreadedExecutor high_prio_odometry_executor;
        // -> High priority control publisher executor.
        rclcpp::executors::StaticSingleThreadedExecutor high_prio_control_executor;
        // -> High priority mocap callback executor.
        rclcpp::executors::SingleThreadedExecutor high_prio_mocap_executor;

        
        // Create the node instances ----------------------------------------------------------- >>
        // create read_odometry node and pass the address to the vehicle object in flight_bridge.hpp and the pointer
        // to the flight_params in flight_bridge.hpp which points to the run_params object in flight_main.cpp
        auto read_odometry_node = std::make_shared<_read_pix_::read_odometryNode>(&vehicle, flight_params_ptr);
        // create control  node and pass the address to the vehicle object in flight_bridge.hpp and the pointer
        // to the flight_params in flight_bridge.hpp which points to the run_params object in flight_main.cpp
        auto control_node = std::make_shared<_control_::controlNode>(&vehicle, flight_params_ptr);
        
        // Create IO context thread on cpu_three and then pass it to the UdpReceiverNode
        // Create a single IoContext and pin it on cpu 3.
        _drivers_::_common_::IoContext ctx{1,CPU_THREE}; 
        auto mocap_node = std::make_shared<_drivers_::_udp_driver_::UdpReceiverNode>(ctx, &vehicle);
                
        // Add the nodes instances to the executor --------------------------------------------- >>
        // Add the odometry node to the odometry executor
        high_prio_odometry_executor.add_node(read_odometry_node);
        // Add the control node to the control executor
        high_prio_control_executor.add_node(control_node);
        // Add the udp receiver node tot the mocap executor
        high_prio_mocap_executor.add_node(mocap_node->get_node_base_interface());


        // Check for Lifecycle Node Starts ----------------------------------------------------- >>
        if (mocap_node->configure().id() == State::PRIMARY_STATE_INACTIVE) {
            if (mocap_node->activate().id() == State::PRIMARY_STATE_ACTIVE) {
            receiver_started = true;
            } else {
            throw std::runtime_error{"Failed to activate UDP receiver."};
            }
        } else {
            throw std::runtime_error{"Failed to configure UDP receiver."};
        }
                
        // Create a thread for each of the executors ------------------------------------------- >>
        auto high_prio_odometry_executor_thread = std::thread(
            [&]() {
                high_prio_odometry_executor.spin();
            });
        auto high_prio_control_executor_thread = std::thread(
            [&]() {
                high_prio_control_executor.spin();
            });
        auto high_prio_mocap_executor_thread = std::thread(
            [&]() {
                // Spin the thread only if Lifecycle Node is configured
                if (receiver_started) { high_prio_mocap_executor.spin(); }
            });

        // Pin the threads to each CPU core ---------------------------------------------------- >>
        // Attempt to create odometry read thread
        ret = configure_thread(high_prio_odometry_executor_thread, ThreadPriority::HIGH, CPU_ZERO);
        if (!ret) {
            std::cout << "Odometry executor thread failed to create" << std::endl;
        }
        else {
            nanoseconds high_prio_odometry_executor_thread_begin_time = 
                get_thread_time(high_prio_odometry_executor_thread);
            
            std::cout << "Odometry executor thread created at:" << 
                high_prio_odometry_executor_thread_begin_time.count() << "ns" << std::endl;
        }
        // Attempt to create control publisher thread
        
        ret = configure_thread(high_prio_control_executor_thread, ThreadPriority::HIGH, CPU_ONE);
        if (!ret) {
            std::cout << "Control executor thread failed to create" << std::endl;
        }
        else {
            nanoseconds high_prio_control_executor_thread_begin_time = 
                get_thread_time(high_prio_control_executor_thread);

            std::cout << "Control executor thread created at:" << 
                high_prio_control_executor_thread_begin_time.count() << "ns" << std::endl;
        }
        
        // Attempt to create mocap publisher thread
        ret = configure_thread(high_prio_mocap_executor_thread, ThreadPriority::HIGH, CPU_TWO);
        if (!ret) {
            std::cout << "Mocap executor thread failed to create" << std::endl;
        }
        else {
            nanoseconds high_prio_mocap_executor_thread_begin_time = 
                get_thread_time(high_prio_odometry_executor_thread);
            
            std::cout << "Mocap executor thread created at:" << 
                high_prio_mocap_executor_thread_begin_time.count() << "ns" << std::endl;
        }
    

        // Join the threads.
        high_prio_odometry_executor_thread.join();
        high_prio_control_executor_thread.join();
        high_prio_mocap_executor_thread.join();
    }
    /// ---------------------------------- GPS RTK EKF  FUSION MODE -------------------------------------------- ///
    else
    {
        // Create Executors -------------------------------------------------------------------- >>
        // -> High priority odometry callback executor.
        rclcpp::executors::StaticSingleThreadedExecutor high_prio_odometry_executor;
        // -> High priority control publisher executor.
        rclcpp::executors::StaticSingleThreadedExecutor high_prio_control_executor;

        
        // Create the node instances ----------------------------------------------------------- >>
        // create read_odometry node and pass the address to the vehicle object in flight_bridge.hpp and the pointer
        // to the flight_params in flight_bridge.hpp which points to the run_params object in flight_main.cpp
        auto read_odometry_node = std::make_shared<_read_pix_::read_odometryNode>(&vehicle, flight_params_ptr);
        // create control  node and pass the address to the vehicle object in flight_bridge.hpp and the pointer
        // to the flight_params in flight_bridge.hpp which points to the run_params object in flight_main.cpp
        auto control_node = std::make_shared<_control_::controlNode>(&vehicle, flight_params_ptr);
                
        // Add the nodes instances to the executor --------------------------------------------- >>
        // Add the odometry node to the odometry executor
        high_prio_odometry_executor.add_node(read_odometry_node);
        // Add the control node to the control executor
        high_prio_control_executor.add_node(control_node);
                
        // Create a thread for each of the executors ------------------------------------------- >>
        auto high_prio_odometry_executor_thread = std::thread(
            [&]() {
                high_prio_odometry_executor.spin();
            });
        auto high_prio_control_executor_thread = std::thread(
            [&]() {
                high_prio_control_executor.spin();
            });

        // Pin the threads to each CPU core ---------------------------------------------------- >>
        // Attempt to create odometry read thread
        ret = configure_thread(high_prio_odometry_executor_thread, ThreadPriority::HIGH, CPU_ZERO);
        if (!ret) {
            std::cout << "Odometry executor thread failed to create" << std::endl;
        }
        else {
            nanoseconds high_prio_odometry_executor_thread_begin_time = 
                get_thread_time(high_prio_odometry_executor_thread);
            
            std::cout << "Odometry executor thread created at:" << 
                high_prio_odometry_executor_thread_begin_time.count() << "ns" << std::endl;
        }
        // Attempt to create control publisher thread
        
        ret = configure_thread(high_prio_control_executor_thread, ThreadPriority::HIGH, CPU_ONE);
        if (!ret) {
            std::cout << "Control executor thread failed to create" << std::endl;
        }
        else {
            nanoseconds high_prio_control_executor_thread_begin_time = 
                get_thread_time(high_prio_control_executor_thread);

            std::cout << "Control executor thread created at:" << 
                high_prio_control_executor_thread_begin_time.count() << "ns" << std::endl;
        }        

        // Join the threads.
        high_prio_odometry_executor_thread.join();
        high_prio_control_executor_thread.join();
    }
}

} // namespace _flight_bridge_