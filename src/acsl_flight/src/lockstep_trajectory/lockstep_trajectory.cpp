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
 * File:        lockstep_trajectory.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        October 30, 2026
 * For info:    Andrea L'Afflitto
 *              a.lafflitto@vt.edu
 *
 * Description: Implementation of lockstep_trajectory (see lockstep_trajectory.hpp). Always compiled; only used when
 *              SIMULATION_LOCKSTEP is 1.
 *
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#include "lockstep_trajectory.hpp"  // Include for the lockstep trajectory class

namespace _piecewise_polynomial_trajectory_{

// Constructor: store the flight params pointer only. Load nothing.
lockstep_trajectory::lockstep_trajectory(flight_params* p) : flight_params_ptr(p) {}

// Called by the controller every step. The controller time is ignored on purose: the lockstep node 
// has already written this step's refernce into lockstep_refernce_store before calling the controller.
// Copies every field into trajectory_base with its own setters, so the controller's gettters 
// return the simulator's reference.
void lockstep_trajectory::updateUserDefinedTrajectory([[maybe_unused]] double controller_time)
{
    const lockstep_reference& ref = lockstep_reference_store;  // Copy the reference from the store

    setUserDefinedPosition(ref.position);
    setUserDefinedVelocity(ref.velocity);
    setUserDefinedAcceleration(ref.acceleration);
    setUserDefinedAngle(ref.angle);
    setUserDefinedAngularRate(ref.angular_rate);
    setUserDefinedAngularRateDot(ref.angular_rate_dot);
    setUserDefinedAngularVelocity(ref.angular_velocity);
    setUserDefinedAngularAcceleration(ref.angular_acceleration);
}

}   // namespace _piecewise_polynomial_trajectory_