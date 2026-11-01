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
 * File:        lockstep_trajectory.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        October 30, 2026
 * For info:    Andrea L'Afflitto
 *              a.lafflitto@vt.edu
 *
 * Description: Trajectory class for SIMULATION LOCKSTEP builds (#define SIMULATION_LOCKSTEP 1 in global_config.hpp).
 *              In lockstep, the simulator (acsl-chrono-simulator) owns the trajectory. Every step, the lockstep node
 *              writes the simulator's reference into lockstep_reference_store, then calls the controller. When the
 *              controller calls ud.updateUserDefinedTrajectory(time), this class ignores the time and copies the
 *              stored reference into the trajectory_base fields, so the controller's getters return the simulator's
 *              reference. No trajectory file is read, and no controller code changes.
 *              In lockstep builds, piecewise_polynomial_trajectory.hpp aliases piecewise_polynomial_trajectory to
 *              this class.
 *
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef LOCKSTEP_TRAJECTORY_HPP_
#define LOCKSTEP_TRAJECTORY_HPP_


#include <Eigen/Dense>                          // Include for Eigen vectors
#include "user_defined_trajectory_base.hpp"     // Include for the base trajectory class
#include "flight_params.hpp"                    // Include for the flight params (same constructor as the original class)

// Same namespace as piecewise_polynomial_trajectory: every controller does
// "using namespace _piecewise_polynomial_trajectory_;", and the lockstep alias lives here too.
namespace _piecewise_polynomial_trajectory_{

// Reference received from the simulator every step. One field per trajectory_base field (1:1), in the same
// frames and units as the original trajectory class produces them.
struct lockstep_reference
{
    Eigen::Vector3d position             = Eigen::Vector3d::Zero();   // ud_pos
    Eigen::Vector3d velocity             = Eigen::Vector3d::Zero();   // ud_vel
    Eigen::Vector3d acceleration         = Eigen::Vector3d::Zero();   // ud_acc
    Eigen::Vector3d angle                = Eigen::Vector3d::Zero();   // ud_angle               (roll, pitch, yaw)
    Eigen::Vector3d angular_rate         = Eigen::Vector3d::Zero();   // ud_angular_rate        (roll, pitch, yaw rates)
    Eigen::Vector3d angular_rate_dot     = Eigen::Vector3d::Zero();   // ud_angular_rate_dot
    Eigen::Vector3d angular_velocity     = Eigen::Vector3d::Zero();   // ud_angular_vel         (omega x, y, z)
    Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero();   // ud_angular_acc         (alpha x, y, z)
};

inline lockstep_reference lockstep_reference_store;

// Inhertis from the base trajectory class, but ignores time and copies the stored reference into the base fields.
class lockstep_trajectory : public _user_defined_trajectory_base_::trajectory_base
{
    public:
        // Same signature as piecewise_polynomial_trajecotry, so the controllers' "ud(p)" compiles unchanged.
        // Loads nothing: the trajectory comes from the simulator every step.
        explicit lockstep_trajectory(flight_params* p);

        // Called by the controllers with the controller time. The time is ignored: the rereference for this
        // step has alredy been written to lockstep_reference_store by the lockstep node. 
        void updateUserDefinedTrajectory(double controller_time);

    private:
        // Kept for partiy with piecewise_polynomial_trajectory, but unused (can be used later)
        flight_params* flight_params_ptr;

};

} // namespace _piecewise_polynomial_trajectory_


#endif  // LOCKSTEP_TRAJECTORY_HPP_