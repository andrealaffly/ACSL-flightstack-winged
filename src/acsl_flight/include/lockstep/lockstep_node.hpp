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
 * File:        lockstep_node.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        October 31, 2026
 * For info:    Andrea L'Afflitto
 *              a.lafflitto@vt.edu
 *
 * Description: Simulation lockstep node (used only when SIMULATION_LOCKSTEP is 1 in global_config.hpp).
 *              The simulator (acsl-chrono-simulator) is the client and this node is the server:
 *              - /acsl_lockstep/handshake : once. Checks protocol version, platform, controller; stores the step size.
 *              - /acsl_lockstep/step      : every simulator step. Takes time + state + reference, runs the SAME
 *                                           controller calls as controlNode (update -> run -> get_t1..t8) and
 *                                           returns the 8 normalized motor thrusts.
 *              Start order does not matter: this node waits passively; the simulator does not step its physics
 *              until the handshake is accepted, so simulation time starts at 0 on the first Step.
 *              One simulation run per flightstack run: a new handshake is refused once stepping has started.
 *              Never publishes to /fmu/in/* : no PX4, no arming, no wall timer. The simulator owns the clock.
 *
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef LOCKSTEP_NODE_HPP_
#define LOCKSTEP_NODE_HPP_

#include <memory>
#include <string>
#include <cstdint>
#include <chrono>

#include "rclcpp/rclcpp.hpp"                            // ROS2 Client Library C++ header
#include "acsl_lockstep_interfaces/srv/handshake.hpp"   // Lockstep handshake service
#include "acsl_lockstep_interfaces/srv/step.hpp"        // Lockstep step service
#include "acsl_lockstep_interfaces/srv/finish.hpp"      // Lockstep finish service

#include "flight_params.hpp"                            // Include for the flight params (controller constructor)
#include "global_helpers.hpp"                           // Include for the flightstack print functions
#include "global_config.hpp"                            // Include for SIMULATION_LOCKSTEP
#include "control_config.hpp"                           // Include for _picked_controller_, platform_of_choice, controller_of_choice
#include "lockstep_trajectory.hpp"                      // Include for lockstep_reference_store

namespace _lockstep_
{

    class lockstepNode : public rclcpp::Node
    {
        public:
            // Same controller construction as controlNode 
            lockstepNode(flight_params* p, const std::string & global_log_dir);

            virtual ~lockstepNode() = default;

        private:
            using Handshake = acsl_lockstep_interfaces::srv::Handshake;
            using Step = acsl_lockstep_interfaces::srv::Step;
            using Finish = acsl_lockstep_interfaces::srv::Finish;

            // Service callbacks (signature validated for ROS2 Foxy -- Jazzy as of 10/2026)
            void handshake_callback(const std::shared_ptr<Handshake::Request> request,
                                    std::shared_ptr<Handshake::Response> response);
             
            void step_callback(const std::shared_ptr<Step::Request> request,
                               std::shared_ptr<Step::Response> response);

            void finish_callback(const std::shared_ptr<Finish::Request> request,
                                 std::shared_ptr<Finish::Response> response);

            // Services
            rclcpp::Service<Handshake>::SharedPtr handshake_service_;
            rclcpp::Service<Step>::SharedPtr step_service_;
            rclcpp::Service<Finish>::SharedPtr finish_service_;

            // Lockstep state
            bool handshake_done_ = false;  // True after a good handshake.
            double step_size_ = 0.0;        // RK4 step [s] from the simulator, used in controller_.run()
            uint64_t steps_done_ = 0;       // number of Steps computed (a new handshake is refused once this is > 0)
            uint64_t expected_seq_ = 0;     // next seq expected from the simulator (warning only on mismatch)

            // Rejected handshake: exit only after the response has been sent, so the simulator receives the reason
            rclcpp::TimerBase::SharedPtr exit_timer_;   // one-shot, armed on rejection (shutdown only, never drives the controller)
            std::string reject_message_;                // reason printed by the deferred FLIGHTSTACK_ERROR

            // End of simulation: shut down cleanly only after the Finish response has been sent
            rclcpp::TimerBase::SharedPtr finish_timer_;  // one-shot, armed by finish_callback (shutdown only)

            /// ------- CREATE CONTROLLER OBJECT -------- ///
            _picked_controller_ controller_;

    };

}   //  namespace _lockstep_

#endif // LOCKSTEP_NODE_HPP_