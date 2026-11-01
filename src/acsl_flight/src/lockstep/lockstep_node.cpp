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
 * File:        lockstep_node.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        October 31, 2026
 * For info:    Andrea L'Afflitto
 *              a.lafflitto@vt.edu
 *
 * Description: Implementation of the simulation lockstep node (see lockstep_node.hpp). Always compiled; only
 *              started by flight_bridge when SIMULATION_LOCKSTEP is 1.
 *
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#include "lockstep_node.hpp"

#include <cmath>  // For std::isfinite

using namespace _flightstack_;

namespace _lockstep_
{

/***********************************************************************************************/
/*                                  NODE CONSTRUCTOR                                           */
/***********************************************************************************************/
lockstepNode::lockstepNode(flight_params* p, const std::string & global_log_dir)
: rclcpp::Node("lockstep_node"), controller_(p, global_log_dir)
{
    using std::placeholders::_1;
    using std::placeholders::_2;

    // No QoS argument on purpose: the only create_service form common to Jazzy and Galactic.
    handshake_service_ = this->create_service<Handshake>(
        "/acsl_lockstep/handshake",
        std::bind(&lockstepNode::handshake_callback, this, _1, _2));

    step_service_ = this->create_service<Step>(
        "/acsl_lockstep/step",
        std::bind(&lockstepNode::step_callback, this, _1, _2));

    finish_service_ = this->create_service<Finish>(
        "/acsl_lockstep/finish",
        std::bind(&lockstepNode::finish_callback, this, _1, _2));        

    FLIGHTSTACK_INFO("HIL/SIL Lockstep node ready. Platform:", platform_of_choice);
    FLIGHTSTACK_INFO("HIL/SIL Lockstep node ready. Controller:", controller_of_choice);
    FLIGHTSTACK_INFO("Lockstep protocol version:", Handshake::Request::PROTOCOL_VERSION);
}

/***********************************************************************************************/
/*                                  HANDSHAKE                                                  */
/***********************************************************************************************/
void lockstepNode::handshake_callback(const std::shared_ptr<Handshake::Request> request,
                                        std::shared_ptr<Handshake::Response> response)
{
    // Always report what this build is, so the simulator can print a useful mismatch message
    response->protocol_version = Handshake::Request::PROTOCOL_VERSION;
    response->platform         = platform_of_choice;
    response->controller       = controller_of_choice;
    response->accepted         = false;

    // Checks, in order. 
    // Check for stale data in the flightstack: the simulator should never send a handshake after the first Step.
    if (steps_done_ > 0) {
        response->message = "Lockstep already running. Restart the flightstack (start_lockstep.sh) for a new simulation.";
    } 
    // Protocol version mismatch: the simulator and flightstack must use the same version of acsl_lockstep_interfaces.
    else if (request->protocol_version != Handshake::Request::PROTOCOL_VERSION) {
        response->message = "Protocol version mismatch: simulator " + std::to_string(request->protocol_version) +
                            ", flightstack " + std::to_string(Handshake::Request::PROTOCOL_VERSION) +
                            ". Keep acsl_lockstep_interfaces identical in both repos.";
    }
    // Checks for platform and controller mismatches, which are defined in 
    // control_config.hpp. The simulator's sim-config.yaml must match these.
    else if (request->platform != platform_of_choice) {
        response->message = "Platform mismatch: simulator '" + request->platform +
                            "', flightstack '" + platform_of_choice + "' (control_config.hpp).";
    }
    else if (request->controller != controller_of_choice) {
        response->message = "Controller mismatch: simulator '" + request->controller +
                            "', flightstack '" + controller_of_choice + "' (control_config.hpp).";
    }
    // Step size check.
    else if (!std::isfinite(request->step_size) || request->step_size <= 0.0) {
        response->message = "Invalid step size: " + std::to_string(request->step_size) + " s.";
    } 
    // Good to go.
    else {
        // Accepted. A repeated identical handshake before the first Step is fine.
        handshake_done_    = true;
        step_size_         = request->step_size;
        expected_seq_      = 0;
        response->accepted = true;
    }

    // Accept or reject message.
    if (response->accepted) {
        FLIGHTSTACK_INFO("HIL/SIL Lockstep handshake ACCEPTED. RK4 step size [s]:", step_size_);
    } else if (!exit_timer_) {
        // Do NOT exit here: FLIGHTSTACK_ERROR calls std::exit, and the response is only sent after this callback
        // returns. Arm a one-shot timer instead; it fires on this same executor after the reply has gone out.
        reject_message_ = response->message;
        exit_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
            FLIGHTSTACK_ERROR("HIL/SIL Lockstep handshake REJECTED:", reject_message_);
        });
    }
}

/***********************************************************************************************/
/*                                  STEP                                                       */
/***********************************************************************************************/
void lockstepNode::step_callback(const std::shared_ptr<Step::Request> request,
                                     std::shared_ptr<Step::Response> response)
{
    // Match sequencing, set default response values (ok = false, motors = 0.0).
    response->seq = request->seq;
    response->ok  = false;
    response->motors.fill(0.0);

    // No controller output before an accepted handshake
    if (!handshake_done_) {
        FLIGHTSTACK_WARNING("Lockstep step before an accepted handshake. Ignored, seq:", request->seq);
        return;
    }

    // Lost or repeated steps show up as a seq jump (warning only, the step is still computed)
    if (request->seq != expected_seq_) {
        FLIGHTSTACK_WARNING("Lockstep seq jump. Expected " + std::to_string(expected_seq_) + ", got",
                            request->seq);
    }
    expected_seq_ = request->seq + 1;

    // 1) Inject this step's reference. The controller reads it through
    //    ud.updateUserDefinedTrajectory() -> lockstep_trajectory (same thread, same callback).
    auto & ref = _piecewise_polynomial_trajectory_::lockstep_reference_store;
    // Unpack the arrays
    for (int i = 0; i < 3; ++i) {
        ref.position(i)             = request->ref_position[i];
        ref.velocity(i)             = request->ref_velocity[i];
        ref.acceleration(i)         = request->ref_acceleration[i];
        ref.angle(i)                = request->ref_angle[i];
        ref.angular_rate(i)         = request->ref_angular_rate[i];
        ref.angular_rate_dot(i)     = request->ref_angular_rate_dot[i];
        ref.angular_velocity(i)     = request->ref_angular_velocity[i];
        ref.angular_acceleration(i) = request->ref_angular_acceleration[i];
    }

    // 2) Same calls as controlNode::compute_and_publish_control(), with simulator time/state and step size
    controller_.update(request->time,
                        request->x,  request->y,  request->z,
                        request->vx, request->vy, request->vz,
                        request->q0, request->q1, request->q2, request->q3,
                        request->roll, request->pitch, request->yaw,
                        request->p,  request->q,  request->r);

    // Compute control with the simulator's step size. Obtained during handshake.
    controller_.run(step_size_);

    // 3) Motor outputs (normalized thrusts, same getters controlNode publishes to PX4)
    response->motors[0] = controller_.get_t1();
    response->motors[1] = controller_.get_t2();
    response->motors[2] = controller_.get_t3();
    response->motors[3] = controller_.get_t4();
    response->motors[4] = controller_.get_t5();
    response->motors[5] = controller_.get_t6();
    response->motors[6] = controller_.get_t7();
    response->motors[7] = controller_.get_t8();

    // NaN / inf guard: report it, the simulator decides what to do (it stops on ok == false)
    bool finite = true;
    for (const double m : response->motors) { finite = finite && std::isfinite(m); }
    if (!finite) {
        FLIGHTSTACK_WARNING("Lockstep controller output is not finite at t [s]:", request->time);
    }

    response->ok = finite;
    ++steps_done_;

    // Same throttled time print as controlNode
    FLIGHTSTACK_INFO_STREAM_NO_TAG("t:", request->time, 1000);
}

/***********************************************************************************************/
/*                                  FINISH                                                     */
/***********************************************************************************************/
void lockstepNode::finish_callback(const std::shared_ptr<Finish::Request> request,
                                   std::shared_ptr<Finish::Response> response)
{
    response->steps_done = steps_done_;
    response->ok         = handshake_done_ && (steps_done_ == request->steps_sent);

    FLIGHTSTACK_INFO("HIL/SIL Simulation finished at t [s]:", request->time);
    FLIGHTSTACK_INFO("HIL/SIL Steps computed:", steps_done_);
    if (!response->ok) {
        FLIGHTSTACK_WARNING("HIL/SIL Step count mismatch. Steps sent by the simulator:", request->steps_sent);
    }

    // Shut down only after this reply has gone out (same pattern as a rejected handshake), but cleanly:
    // rclcpp::shutdown() makes spin() return, so the thread joins and the controller logs close normally (exit code 0)
    if (!finish_timer_) {
        finish_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), []() {
            FLIGHTSTACK_INFO("HIL/SIL Lockstep done. Shutting down.");
            rclcpp::shutdown();
        });
    }
}

}   // namespace _lockstep_