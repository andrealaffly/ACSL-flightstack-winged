/***********************************************************************************************************************
 * Copyright (c) 2025 Giri M. Kumar, Mattia Gramuglia, Andrea L'Afflitto. All rights reserved.
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
 * File:        NMPC_HYBRID_TD.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        November 9, 2026
 * For info:    Andrea L'Afflitto
 *              a.lafflitto@vt.edu
 *
 * Description: Interconnected nonlinear MPC and hybrid MRAC for the QUADM. Equation numbers refer to
 *              G. M. Kumar, X. E. Casanova Pabon, A. L'Afflitto, "Interconnected Nonlinear MPC and Hybrid MRAC with
 *              Applications to Multi-Rotor UAVs".
 *
 *              Every control tick (run):
 *                  1. Hybrid reset of the outer reference model if ||p - p_ref|| >= threshold (Eq. (19))
 *                  2. Outer NMPC (every dt_outer): plan the force from the measured [p; v], hold the first input as
 *                     mu_baseline (Eq. (44)) and build the reference command r from the plan (Eqs. (52), (71))
 *                  3. Outer MRAC: mu = mu_baseline + mu_adaptive (Eqs. (28)-(30), (49), (66)-(72))
 *                  4. Command generator: mu -> filter -> thrust, R_d, Omega_d (Eqs. (60)-(63))
 *                  5. Inner NMPC (every dt_inner): plan the torque from the measured [R; omega] to track the preview
 *                     of R_d built from the outer plan + the held adaptive force, hold the first input
 *                  6. Inner MRAC on omega: tau = tau_baseline + tau_adaptive (Eqs. (73)-(80))
 *                  7. Mixer -> motor thrusts -> normalized commands, then integrate the reference models and gains
 *
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

/*               _           _      _                      
 $$$$$$\  $$\   $$\  $$$$$$\  $$$$$$$\  $$\      $$\ 
$$  __$$\ $$ |  $$ |$$  __$$\ $$  __$$\ $$$\    $$$ |
$$ /  $$ |$$ |  $$ |$$ /  $$ |$$ |  $$ |$$$$\  $$$$ |
$$ |  $$ |$$ |  $$ |$$$$$$$$ |$$ |  $$ |$$\$$\$$ $$ |
$$ |  $$ |$$ |  $$ |$$  __$$ |$$ |  $$ |$$ \$$$  $$ |
$$ $$\$$ |$$ |  $$ |$$ |  $$ |$$ |  $$ |$$ |\$  /$$ |
\$$$$$$ / \$$$$$$  |$$ |  $$ |$$$$$$$  |$$ | \_/ $$ |
 \___$$$\  \______/ \__|  \__|\_______/ \__|     \__|
     \___|                                           
*/

#include "NMPC_HYBRID_TD.hpp"

namespace _quadm_{
namespace _nmpc_hybrid_{

// -------------------------------------------------------------------------
// Constructor Implementation:
//   - Calls the base (controller_base) constructor, builds the trajectory
//     and the logger (the logger creates the log file and writes the headers)
// -------------------------------------------------------------------------
nmpc_hybrid::nmpc_hybrid(flight_params* p, const std::string & controller_log_dir_) :
                         controller_base(p), ud(p),
                         logger(&cim, &csm, &onm, &inm, &control_input, controller_log_dir_)
{
    // Reading in the parameters (path is relative to the workspace folder the flightstack runs from)
    read_params("./src/acsl_flight/params/control_algorithms/quadm/NMPC_HYBRID/gains_NMPC_HYBRID.json");

    // Initial Conditions
    init();
}

// Destructor
nmpc_hybrid::~nmpc_hybrid() {
}

// -------------------------------------------------------------------------
// read_params Implementation:
// - Takes the hardcoded path for the gains and parameters and reads it in
// -------------------------------------------------------------------------
void nmpc_hybrid::read_params(const std::string& jsonFile)
{
    // Open and parse the gains file
    std::ifstream file(jsonFile);
    nlohmann::json j;
    file >> j;

    // Outer (translational) NMPC ---------------------------------------------------------------------------------------
    // Q/Qf are ordered as the error state [dp, dv] and R as the force [mu_x, mu_y, mu_z]
    cip.dt_outer = j["OUTER_NMPC"]["dt"];
    cip.max_iterations_outer = j["OUTER_NMPC"]["max_iterations"];
    cip.line_search_steps_outer = j["OUTER_NMPC"]["line_search_steps"];
    cip.regularization_outer = j["OUTER_NMPC"]["regularization"];
    cip.thrust_min = j["OUTER_NMPC"]["thrust_min"];
    cip.thrust_max = j["OUTER_NMPC"]["thrust_max"];
    const double tilt_max_deg = j["OUTER_NMPC"]["tilt_max_deg"];
    cip.cos_tilt_max = std::cos(tilt_max_deg * DEG2RAD);
    cip.terminal_cost_from_lqr_outer = j["OUTER_NMPC"]["terminal_cost_from_lqr"];
    Eigen::VectorXd q_outer = jsonToScaledMatrixXd(j["OUTER_NMPC"]["Q_diagonal"], 1, NX).transpose();
    Eigen::VectorXd qf_outer = jsonToScaledMatrixXd(j["OUTER_NMPC"]["Qf_diagonal"], 1, NX).transpose();
    Eigen::VectorXd r_outer = jsonToScaledMatrixXd(j["OUTER_NMPC"]["R_diagonal"], 1, NU).transpose();
    cip.Q_outer = q_outer.asDiagonal();
    cip.Qf_outer = qf_outer.asDiagonal();
    cip.R_outer = r_outer.asDiagonal();

    // Inner (attitude) NMPC --------------------------------------------------------------------------------------------
    // Q/Qf are ordered as the residual [eta, e_omega] (Eq. (84)) and R as the torque [tau_x, tau_y, tau_z]
    cip.dt_inner = j["INNER_NMPC"]["dt"];
    cip.max_iterations_inner = j["INNER_NMPC"]["max_iterations"];
    cip.line_search_steps_inner = j["INNER_NMPC"]["line_search_steps"];
    cip.regularization_inner = j["INNER_NMPC"]["regularization"];
    cip.terminal_cost_from_lqr_inner = j["INNER_NMPC"]["terminal_cost_from_lqr"];
    Eigen::VectorXd q_inner = jsonToScaledMatrixXd(j["INNER_NMPC"]["Q_diagonal"], 1, NX).transpose();
    Eigen::VectorXd qf_inner = jsonToScaledMatrixXd(j["INNER_NMPC"]["Qf_diagonal"], 1, NX).transpose();
    Eigen::VectorXd r_inner = jsonToScaledMatrixXd(j["INNER_NMPC"]["R_diagonal"], 1, NU).transpose();
    cip.Q_inner = q_inner.asDiagonal();
    cip.Qf_inner = qf_inner.asDiagonal();
    cip.R_inner = r_inner.asDiagonal();

    // Disturbance estimate ---------------------------------------------------------------------------------------------
    cip.use_nmpc_disturbance_estimate = j["DISTURBANCE"]["use_nmpc_disturbance_estimate"];
    cip.Ki_disturbance = jsonToScaledMatrixXd(j["DISTURBANCE"]["KI_translational"], 3, 3);
    cip.d_hat_max = jsonToScaledMatrixXd(j["DISTURBANCE"]["d_hat_max"], 1, 3).transpose();

    // Translational (outer) MRAC ---------------------------------------------------------------------------------------
    cip.Kp_refmod_tran = jsonToScaledMatrixXd(j["TRANSLATIONAL"]["KP_refmod_translational"], 3, 3);
    cip.Kv_refmod_tran = jsonToScaledMatrixXd(j["TRANSLATIONAL"]["KV_refmod_translational"], 3, 3);
    cip.Gamma_x_tran = jsonToScaledMatrixXd(j["TRANSLATIONAL"]["Gamma_x_translational"], 6, 6);
    cip.Gamma_r_tran = jsonToScaledMatrixXd(j["TRANSLATIONAL"]["Gamma_r_translational"], 3, 3);
    cip.Gamma_Theta_tran = jsonToScaledMatrixXd(j["TRANSLATIONAL"]["Gamma_Theta_translational"], NTHETA_TRAN, NTHETA_TRAN);
    cip.Q_tran = jsonToScaledMatrixXd(j["TRANSLATIONAL"]["Q_translational"], 6, 6);
    cip.wind_estimate = jsonToScaledMatrixXd(j["TRANSLATIONAL"]["wind_estimate"], 1, 3).transpose();

    // Rotational (inner) MRAC ------------------------------------------------------------------------------------------
    cip.K_omega_ref = jsonToScaledMatrixXd(j["ROTATIONAL"]["K_omega_ref"], 3, 3);
    cip.k_R = j["ROTATIONAL"]["k_R"];
    cip.Gamma_x_rot = jsonToScaledMatrixXd(j["ROTATIONAL"]["Gamma_x_rotational"], 3, 3);
    cip.Gamma_r_rot = jsonToScaledMatrixXd(j["ROTATIONAL"]["Gamma_r_rotational"], 3, 3);
    cip.Gamma_Theta_rot = jsonToScaledMatrixXd(j["ROTATIONAL"]["Gamma_Theta_rotational"], NTHETA_ROT, NTHETA_ROT);
    cip.Q_rot = jsonToScaledMatrixXd(j["ROTATIONAL"]["Q_rotational"], 3, 3);

    // Command generator ------------------------------------------------------------------------------------------------
    cip.A_filter_mu = jsonToScaledMatrixXd(j["FILTERS"]["A_filter_mu"], 2, 2);
    cip.B_filter_mu = jsonToScaledMatrixXd(j["FILTERS"]["B_filter_mu"], 2, 1);
    cip.C_filter_mu = jsonToScaledMatrixXd(j["FILTERS"]["C_filter_mu"], 1, 2);
    cip.C_filter_mu_lowpass = jsonToScaledMatrixXd(j["FILTERS"]["C_filter_mu_lowpass"], 1, 2);

    // Hybrid -----------------------------------------------------------------------------------------------------------
    cip.use_hybrid = j["HYBRID"]["use_hybrid"];
    cip.hybrid_error_norm_threshold = j["HYBRID"]["error_norm_threshold"];
    cip.hybrid_rho = j["HYBRID"]["rho"];
    cip.hybrid_dwell_time = j["HYBRID"]["dwell_time"];

    // Robustification Parameters ---------------------------------------------------------------------------------------
    cip.use_dead_zone_emod = j["ROBUSTIFICATION"]["use_dead_zone_emod"];
    cip.dead_zone_delta_translational = j["ROBUSTIFICATION"]["dead_zone_delta_translational"];
    cip.dead_zone_e0_translational = j["ROBUSTIFICATION"]["dead_zone_e0_translational"];
    cip.sigma_x_translational = j["ROBUSTIFICATION"]["sigma_x_translational"];
    cip.sigma_r_translational = j["ROBUSTIFICATION"]["sigma_r_translational"];
    cip.sigma_Theta_translational = j["ROBUSTIFICATION"]["sigma_Theta_translational"];
    cip.projection_x_max_x_translational = j["ROBUSTIFICATION"]["projection_x_max_x_translational"];
    cip.projection_epsilon_x_translational = j["ROBUSTIFICATION"]["projection_epsilon_x_translational"];
    cip.projection_x_max_r_translational = j["ROBUSTIFICATION"]["projection_x_max_r_translational"];
    cip.projection_epsilon_r_translational = j["ROBUSTIFICATION"]["projection_epsilon_r_translational"];
    cip.projection_x_max_Theta_translational = j["ROBUSTIFICATION"]["projection_x_max_Theta_translational"];
    cip.projection_epsilon_Theta_translational = j["ROBUSTIFICATION"]["projection_epsilon_Theta_translational"];

    cip.dead_zone_delta_rotational = j["ROBUSTIFICATION"]["dead_zone_delta_rotational"];
    cip.dead_zone_e0_rotational = j["ROBUSTIFICATION"]["dead_zone_e0_rotational"];
    cip.sigma_x_rotational = j["ROBUSTIFICATION"]["sigma_x_rotational"];
    cip.sigma_r_rotational = j["ROBUSTIFICATION"]["sigma_r_rotational"];
    cip.sigma_Theta_rotational = j["ROBUSTIFICATION"]["sigma_Theta_rotational"];
    cip.projection_x_max_x_rotational = j["ROBUSTIFICATION"]["projection_x_max_x_rotational"];
    cip.projection_epsilon_x_rotational = j["ROBUSTIFICATION"]["projection_epsilon_x_rotational"];
    cip.projection_x_max_r_rotational = j["ROBUSTIFICATION"]["projection_x_max_r_rotational"];
    cip.projection_epsilon_r_rotational = j["ROBUSTIFICATION"]["projection_epsilon_r_rotational"];
    cip.projection_x_max_Theta_rotational = j["ROBUSTIFICATION"]["projection_x_max_Theta_rotational"];
    cip.projection_epsilon_Theta_rotational = j["ROBUSTIFICATION"]["projection_epsilon_Theta_rotational"];
}

// Implementing virutal functions from controller_base
void nmpc_hybrid::init(){
    // Set the inital conditions (zero adaptive gains, the reference models are set at the first update)
    y.fill(0.0);
    dy.fill(0.0);

    // Physical parameters from the quadm parameter file
    // NOTE: mixer_matrix_quadm maps [T; tau] to the motor thrusts, so its inverse maps the thrusts back to [T; tau]
    cip.J = inertia_matrix_q;
    cip.J_inv = cip.J.inverse();
    cip.mixer_inv = mixer_matrix_quadm.inverse();

    /// Translational - Initialize matrices (Eqs. (66) and (70))
    // Initialize to zero the 6x6 matrix and set the top-right 3x3 block as follows
    initMat(cip.A_tran);
    cip.A_tran.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    // Initialize to zero the 6x3 matrix and set the bottom 3x3 block as follows
    initMat(cip.B_tran);
    cip.B_tran.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();

    // Initialize to zero the 6x6 matrix
    initMat(cip.A_ref_tran);
    // Set the top-right 3x3 block as follows
    cip.A_ref_tran.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    // Set the bottom-left 3x3 block as follows
    cip.A_ref_tran.block<3, 3>(3, 0) = -cip.Kp_refmod_tran;
    // Set the bottom-right 3x3 block as follows
    cip.A_ref_tran.block<3, 3>(3, 3) = -cip.Kv_refmod_tran;

    // Initialize to zero the 6x3 matrix and set the bottom 3x3 block as follows (B_ref = B_o Kp)
    initMat(cip.B_ref_tran);
    cip.B_ref_tran.block<3, 3>(3, 0) = cip.Kp_refmod_tran;

    // Inverse of Kp for the reference command (Eq. (71))
    cip.Kp_refmod_tran_inv = cip.Kp_refmod_tran.inverse();

    // Solve the continuous Lyapunov equation to compute P_translational (Eq. (5))
    cip.P_tran = RealContinuousLyapunovEquation(cip.A_ref_tran, cip.Q_tran);

    /// Rotational - Initialize matrices (Eqs. (73) and (79))
    // Initlize to identity the 3x3 matrix
    cip.B_rot = Eigen::Matrix3d::Identity();

    // Set the 3x3 matrix as follows (A_ref = -K_omega)
    cip.A_ref_rot << -cip.K_omega_ref;

    // Set the 3x3 matrix as follows
    cip.B_ref_rot = Eigen::Matrix3d::Identity();

    // Solve the continuous Lyapunov eequation to compute P_rotational
    cip.P_rot = RealContinuousLyapunovEquation(cip.A_ref_rot, cip.Q_rot);

    /// Terminal costs: infinite horizon LQR cost-to-go of the nominal models linearized at hover
    // The horizons can be shorter than the slowest closed loop time constant, so the terminal cost must carry the
    // cost of the rest of the trajectory (nominal LQ design of the terminal ingredients, Sec. II-B.3).
    if (cip.terminal_cost_from_lqr_outer)
    {
        Mat6 A_outer;
        Mat6x3 B_outer;
        outer_linearize(A_outer, B_outer);
        cip.Qf_outer = compute_lqr_terminal_cost(A_outer, B_outer, cip.Q_outer, cip.R_outer);
    }
    if (cip.terminal_cost_from_lqr_inner)
    {
        Mat6 A_inner;
        Mat6x3 B_inner;
        inner_linearize(inner_state(), A_inner, B_inner);       // Hover: R = I, omega = 0
        cip.Qf_inner = compute_lqr_terminal_cost(A_inner, B_inner, cip.Q_inner, cip.R_inner);
    }

    /// Outer NMPC - warm start: hover with the weight carried by the thrust (NED, so the force points up)
    const Eigen::Vector3d mu_hover = -MASS * G * e3_basis;
    onm.U.fill(mu_hover);
    onm.U_ref.fill(mu_hover);
    onm.U_plan.fill(mu_hover);
    for (auto& k : onm.K_fb) k.setZero();
    for (auto& k : onm.k_ff) k.setZero();
    onm.cost_initial = 0.0;
    onm.cost = 0.0;
    onm.iterations = 0;
    onm.alpha = 0.0;
    onm.first_solve = true;
    onm.solved_this_tick = false;
    onm.t_solve = 0.0;
    onm.p_solve.setZero();
    onm.v_solve.setZero();
    onm.a_solve.setZero();
    onm.mu_hold = mu_hover;

    /// Inner NMPC - warm start: zero torque
    inm.U.fill(Eigen::Vector3d::Zero());
    for (auto& k : inm.K_fb) k.setZero();
    for (auto& k : inm.k_ff) k.setZero();
    for (auto& w : inm.Omega_d_dot) w.setZero();
    inm.cost_initial = 0.0;
    inm.cost = 0.0;
    inm.iterations = 0;
    inm.alpha = 0.0;
    inm.first_solve = true;
    inm.solved_this_tick = false;
    inm.t_solve = 0.0;
    inm.thrust_preview = MASS * G;
    inm.tau_hold.setZero();

    /// Initialize the remaining internal members
    cim.first_run = true;
    cim.d_hat.setZero();
    cim.e_tran_pos_integrand.setZero();
    cim.mu_tran_baseline = mu_hover;
    cim.mu_tran_adaptive.setZero();
    cim.mu_tran_I = mu_hover;
    cim.mu_tran_I_dot.setZero();
    cim.mu_tran_I_filtered = mu_hover;
    cim.R_d.setIdentity();
    cim.tau_rot_baseline.setZero();
    cim.tau_rot_adaptive.setZero();
    cim.tau_rot.setZero();
    cim.hybrid_reset_triggered = false;
    cim.number_of_hybrid_resets = 0;
    cim.time_of_last_hybrid_reset = -cip.hybrid_dwell_time;     // Allows a reset from t = 0
    cim.u.setZero();
    cim.Thrust.setZero();
    cim.Sat_Thrust.setZero();
}

// Update function for the controller
//   - Called by the simulator before run() with the latest measured state.
void nmpc_hybrid::update(double time,
                         double x,
                         double y,
                         double z,
                         double vx,
                         double vy,
                         double vz,
                         double q0,
                         double q1,
                         double q2,
                         double q3,
                         double roll,
                         double pitch,
                         double yaw,
                         double w_x,
                         double w_y,
                         double w_z)
{
    // 1. Assign all the states ----------------------------------------------------
    cim.t = time;

    // Assign the Translational States
    cim.x_tran_pos << x, y, z;
    cim.x_tran_vel << vx, vy, vz;
    cim.x_tran << cim.x_tran_pos,
                  cim.x_tran_vel;

    // Assign the quaternions and ensure it is normalized
    // q0 = w, q1 = x, q2 = y, q3 = z
    cim.q = Eigen::Quaterniond(q0, q1, q2, q3).normalized();

    // Assign the rotation matrices
    cim.Rji = cim.q.toRotationMatrix(); // rotation from the body to the inertial frame
    cim.Rij = cim.Rji.transpose();      // rotation from the inertial to the body frame

    // Assign the Euler angles and the angular velocity
    cim.eta_rot << roll, pitch, yaw;
    cim.omega << w_x, w_y, w_z;

    // 2. Get the reference trajectory ---------------------------------------------
    ud.updateUserDefinedTrajectory(time);
    cim.r_user = ud.getUserDefinedPosition();
    cim.r_dot_user = ud.getUserDefinedVelocity();
    cim.r_ddot_user = ud.getUserDefinedAcceleration();
    cim.psi_user = ud.getUserDefinedYaw();
    cim.psi_dot_user = ud.getUserDefinedYawRate();

    // 3. Capture the time before the execution of the controller ------------------
    cim.alg_start_time = std::chrono::high_resolution_clock::now();

    // 4. Assign the values from the integrator ------------------------------------
    assign_from_rk4();

    // 5. Start the reference models and the filters at the measured state ----------
    if (cim.first_run) {
        initialize_from_measured_state();
        cim.first_run = false;
    }
}

// Function to assign elements from the rk4 integrator
//   - Also records the offsets of the members that are written back into y (reference models and filters).
void nmpc_hybrid::assign_from_rk4()
{
    // Running position in the rk4 array (must follow the same order as model())
    int index = 0;

    //------------ assign after integration  ------------//
    cim.rk4_index_state_mu_x_filter = index;
    assignElementsToMembers(csm.state_mu_x_filter, y, index);
    assignElementsToMembers(csm.state_mu_y_filter, y, index);
    assignElementsToMembers(csm.state_mu_z_filter, y, index);
    cim.rk4_index_x_tran_ref = index;
    assignElementsToMembers(csm.x_tran_ref, y, index);
    assignElementsToMembers(csm.K_hat_x_tran, y, index);
    assignElementsToMembers(csm.K_hat_r_tran, y, index);
    assignElementsToMembers(csm.Theta_hat_tran, y, index);
    cim.rk4_index_omega_ref = index;
    assignElementsToMembers(csm.omega_ref, y, index);
    assignElementsToMembers(csm.K_hat_x_rot, y, index);
    assignElementsToMembers(csm.K_hat_r_rot, y, index);
    assignElementsToMembers(csm.Theta_hat_rot, y, index);
    assignElementsToMembers(csm.e_tran_pos_I, y, index);
}

// Model function for integration
void nmpc_hybrid::model(const rk4_array<double, NSI> &y, rk4_array<double, NSI> &dy, double t)
{
    // Running position in the rk4 array (must follow the same order as assign_from_rk4())
    int index = 0;

    //------------ Fill up the dy for integration  ------------//
    assignElementsToDxdt(cim.internal_state_mu_x_filter, dy, index);
    assignElementsToDxdt(cim.internal_state_mu_y_filter, dy, index);
    assignElementsToDxdt(cim.internal_state_mu_z_filter, dy, index);
    assignElementsToDxdt(cim.x_tran_ref_dot, dy, index);
    assignElementsToDxdt(cim.K_hat_x_tran_dot, dy, index);
    assignElementsToDxdt(cim.K_hat_r_tran_dot, dy, index);
    assignElementsToDxdt(cim.Theta_hat_tran_dot, dy, index);
    assignElementsToDxdt(cim.omega_ref_dot, dy, index);
    assignElementsToDxdt(cim.K_hat_x_rot_dot, dy, index);
    assignElementsToDxdt(cim.K_hat_r_rot_dot, dy, index);
    assignElementsToDxdt(cim.Theta_hat_rot_dot, dy, index);
    assignElementsToDxdt(cim.e_tran_pos_integrand, dy, index);
}

// Function to start the reference models and the filters at the measured state
//   - Quality of life improvement, removes the initial jump. Theoretically supported as the initial conditions of
//     the plant and the reference model can be set arbitrarily.
void nmpc_hybrid::initialize_from_measured_state()
{
    int index = 0;

    // Translational reference model at the measured [p; v]
    csm.x_tran_ref = cim.x_tran;
    index = cim.rk4_index_x_tran_ref;
    assignElementsToDxdt(csm.x_tran_ref, this->y, index);

    // Angular velocity reference model at the measured omega
    csm.omega_ref = cim.omega;
    index = cim.rk4_index_omega_ref;
    assignElementsToDxdt(csm.omega_ref, this->y, index);

    // mu filters at their steady state for the hover force, so that mu_dot starts at zero
    // Steady state of x_dot = A x + B u is x = -A^-1 B u
    const Eigen::Vector3d mu_hover = -MASS * G * e3_basis;
    const Eigen::Matrix<double, 2, 1> ss_gain = -cip.A_filter_mu.inverse() * cip.B_filter_mu;
    csm.state_mu_x_filter = ss_gain * mu_hover(0);
    csm.state_mu_y_filter = ss_gain * mu_hover(1);
    csm.state_mu_z_filter = ss_gain * mu_hover(2);
    index = cim.rk4_index_state_mu_x_filter;
    assignElementsToDxdt(csm.state_mu_x_filter, this->y, index);
    assignElementsToDxdt(csm.state_mu_y_filter, this->y, index);
    assignElementsToDxdt(csm.state_mu_z_filter, this->y, index);
}

// Function to reset the outer reference model with the hybrid algorithm
//   - Runs at the start of the tick, where the measured state and the integrated reference model refer to the same
//     time, so the reset map x_ref(t+) = x - rho e is exact (Eq. (19)). The adaptive gains are retained (Eq. (20)).
//   - Only the outer loop is hybrid. The trigger is the position error norm, with a dwell time between resets.
void nmpc_hybrid::compute_hybrid_reference_reset()
{
    cim.hybrid_reset_triggered = false;

    // If hybrid is switched off, return
    if (!cip.use_hybrid) { return; }

    // Tracking error before the reset
    const Eigen::Matrix<double, 6, 1> e = cim.x_tran - csm.x_tran_ref;

    // Call the hybrid step on the OUTERLOOP (trigger on the position part of the error)
    std::tie(cim.hybrid_reset_triggered, cim.time_of_last_hybrid_reset)
    =
    _adaptive_laws_hybrid_error_norm_::runErrorNormHybridStep(
        cim.x_tran,
        e,
        e.head<3>(),
        cip.hybrid_error_norm_threshold,
        cip.hybrid_rho,
        cip.hybrid_dwell_time,
        cim.t,
        cim.time_of_last_hybrid_reset,
        csm.x_tran_ref
    );

    // Nothing to write back
    if (!cim.hybrid_reset_triggered) { return; }

    // Count the resets
    ++cim.number_of_hybrid_resets;

    // Assign the y's -> Use the dxdt function to reassign the y rk4 vector at the recorded offset.
    int index = cim.rk4_index_x_tran_ref;
    assignElementsToDxdt(csm.x_tran_ref, this->y, index);
}

// Function to compute the disturbance estimate
//   - Not part of the paper, switched off by default. The estimate integrates the error with respect to the
//     reference model and is injected in the outer NMPC model, so its effect is part of mu_baseline, which is in the
//     MRAC regressor.
void nmpc_hybrid::compute_disturbance_estimate()
{
    // Switched off: no estimate and nothing to integrate
    if (!cip.use_nmpc_disturbance_estimate) {
        cim.d_hat.setZero();
        cim.e_tran_pos_integrand.setZero();
        return;
    }

    // Integrate the position error with respect to the reference model
    cim.e_tran_pos_integrand = cim.x_tran_pos - csm.x_tran_ref.head<3>();

    // Constant acceleration acting on the vehicle over the whole horizon
    cim.d_hat = cip.Ki_disturbance * csm.e_tran_pos_I;

    // Saturate the estimate to avoid wind-up
    cim.d_hat = cim.d_hat.cwiseMin(cip.d_hat_max).cwiseMax(-cip.d_hat_max);
}

// =====================================================================================================================
// OUTER (TRANSLATIONAL) NMPC
// =====================================================================================================================

// Function to sample, solve and hold the outer NMPC
//   - The NMPC is solved every dt_outer from the measured [p; v] (Eq. (42)) and its first input is held as the
//     baseline until the next solve (Eqs. (43)-(44)).
//   - Between solves, the reference command is computed from the nominal plan (Eqs. (51)-(52), (71)). For the double
//     integrator the plan is exact under the held force, and Eq. (53) holds, so r = p* + Kp^-1 (Kv v* + a*).
void nmpc_hybrid::compute_outer_nmpc()
{
    onm.solved_this_tick = false;

    // 1. Solve at the sampling instants ----------------------------------------------------------------------------
    if (onm.first_solve || cim.t >= onm.t_solve + cip.dt_outer - 1e-9)
    {
        // Sample the user trajectory over the horizon
        compute_outer_reference_horizon();

        // Every plan starts at the measured state
        onm.x0.p = cim.x_tran_pos;
        onm.x0.v = cim.x_tran_vel;

        // Solve the optimal control problem
        outer_solve_ocp();

        // Keep the accepted plan (used by the inner NMPC preview) and hold its first input
        onm.U_plan = onm.U;
        onm.mu_hold = onm.U[0];

        // Anchor of the nominal plan for the reference command
        onm.t_solve = cim.t;
        onm.p_solve = cim.x_tran_pos;
        onm.v_solve = cim.x_tran_vel;
        onm.a_solve = G * e3_basis + cim.d_hat + onm.mu_hold / MASS;

        // Shift the warm start for the next solve
        outer_shift_warm_start();

        onm.solved_this_tick = true;
        onm.first_solve = false;
    }

    // 2. Baseline force (zero-order hold of the first planned input) -----------------------------------------------
    cim.mu_tran_baseline = onm.mu_hold;

    // 3. Reference command from the nominal plan -------------------------------------------------------------------
    const double tau = cim.t - onm.t_solve;
    const Eigen::Vector3d v_star = onm.v_solve + onm.a_solve * tau;
    const Eigen::Vector3d p_star = onm.p_solve + onm.v_solve * tau + 0.5 * onm.a_solve * tau * tau;
    cim.r_cmd_tran = p_star + cip.Kp_refmod_tran_inv * (cip.Kv_refmod_tran * v_star + onm.a_solve);
}

// Function to build the reference states and forces over the outer horizon
void nmpc_hybrid::compute_outer_reference_horizon()
{
    const Eigen::Vector3d e3 = e3_basis;            // Down axis in NED

    // Sample the trajectory over the horizon
    // NOTE: The samples are taken in increasing time so the hover/landing logic of the minjerkpoly module that
    //       stores the previous position stays consistent. The module is restored to the current time afterwards.
    for (int k = 0; k <= NH_O; ++k)
    {
        ud.updateUserDefinedTrajectory(cim.t + k * cip.dt_outer);

        onm.X_ref[k].p = ud.getUserDefinedPosition();
        onm.X_ref[k].v = ud.getUserDefinedVelocity();

        // Reference force from the nominal model a = g e3 + d_hat + mu / m  ->  mu = m (a_ref - g e3 - d_hat)
        if (k < NH_O) {
            const Eigen::Vector3d a_ref = ud.getUserDefinedAcceleration();
            onm.U_ref[k] = outer_project_force(MASS * (a_ref - G * e3 - cim.d_hat));
        }
    }

    // Restore the trajectory module to the current time
    ud.updateUserDefinedTrajectory(cim.t);
}

// Function to propagate the outer nominal model by one step
//   - Exact discretization of the double integrator under a zero-order-held force.
outer_state nmpc_hybrid::outer_predict_step(const outer_state& x, const Eigen::Vector3d& mu) const
{
    const double dt = cip.dt_outer;

    // Translational acceleration in NED
    const Eigen::Vector3d acc = G * e3_basis + cim.d_hat + mu / MASS;

    outer_state n;
    n.p = x.p + x.v * dt + 0.5 * acc * dt * dt;                             // Position
    n.v = x.v + acc * dt;                                                   // Velocity
    return n;
}

// Function to linearize the outer nominal model (the model is linear, so A and B are constant)
void nmpc_hybrid::outer_linearize(Mat6& A, Mat6x3& B) const
{
    const double dt = cip.dt_outer;
    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

    // State Jacobian
    A.setIdentity();
    A.block<3, 3>(0, 3) = I3 * dt;                                          // dp <- dv

    // Input Jacobian
    B.block<3, 3>(0, 0) = 0.5 * dt * dt / MASS * I3;                        // dp <- mu
    B.block<3, 3>(3, 0) = dt / MASS * I3;                                   // dv <- mu
}

// Function to project the force onto the thrust magnitude and tilt limits
//   - The thrust direction is b3 = -mu / |mu| (Eq. (60)), and its tilt is the angle between b3 and e3 (NED down).
Eigen::Vector3d nmpc_hybrid::outer_project_force(const Eigen::Vector3d& mu) const
{
    double norm_mu = mu.norm();

    // No defined direction, fall back to the min thrust pointing up
    if (norm_mu < 1e-9) { return -cip.thrust_min * e3_basis; }

    // 1. Thrust magnitude limits
    const double norm_sat = std::min(std::max(norm_mu, cip.thrust_min), cip.thrust_max);
    Eigen::Vector3d mu_proj = mu * (norm_sat / norm_mu);
    norm_mu = norm_sat;

    // 2. Tilt limit: cos(tilt) = b3 . e3 = -mu_z / |mu|
    if (-mu_proj(2) / norm_mu < cip.cos_tilt_max)
    {
        // Keep the horizontal direction and the magnitude, and move the force onto the cone boundary
        const Eigen::Vector2d mu_h = mu_proj.head<2>();
        const double norm_h = mu_h.norm();
        const Eigen::Vector2d dir_h = (norm_h > 1e-9) ? Eigen::Vector2d(mu_h / norm_h) : Eigen::Vector2d::UnitX();
        const double sin_tilt_max = std::sqrt(std::max(0.0, 1.0 - cip.cos_tilt_max * cip.cos_tilt_max));

        mu_proj << norm_mu * sin_tilt_max * dir_h,
                   -norm_mu * cip.cos_tilt_max;
    }

    return mu_proj;
}

// Function to compute the outer stage cost at step k (Eq. (34))
double nmpc_hybrid::outer_stage_cost(const int k, const outer_state& x, const Eigen::Vector3d& mu) const
{
    const Vec6 e = box_minus_outer(x, onm.X_ref[k]);
    const Eigen::Vector3d dmu = mu - onm.U_ref[k];

    // 0.5 e'Qe + 0.5 dmu'R dmu
    return 0.5 * e.dot(cip.Q_outer * e) + 0.5 * dmu.dot(cip.R_outer * dmu);
}

// Function to compute the outer terminal cost (Eq. (35))
double nmpc_hybrid::outer_terminal_cost(const outer_state& x) const
{
    const Vec6 e = box_minus_outer(x, onm.X_ref[NH_O]);
    return 0.5 * e.dot(cip.Qf_outer * e);
}

// Function to simulate the outer warm start from the measured state
double nmpc_hybrid::outer_rollout()
{
    double c = 0.0;
    onm.X[0] = onm.x0;      // Every plan starts at the measured state
    for (int k = 0; k < NH_O; ++k)
    {
        c += outer_stage_cost(k, onm.X[k], onm.U[k]);
        onm.X[k + 1] = outer_predict_step(onm.X[k], onm.U[k]);
    }
    return c + outer_terminal_cost(onm.X[NH_O]);
}

// Function to compute the outer feedback and feed-forward gains by walking backward from the end of the horizon
//   - The gains satisfy: optimal input change dmu = k_ff + K_fb dx, for a state deviation dx from onm.X (Eq. (37)).
void nmpc_hybrid::outer_backward_pass()
{
    // Start from the terminal cost
    Vec6 Vx  = cip.Qf_outer * box_minus_outer(onm.X[NH_O], onm.X_ref[NH_O]);    // Gradient of the cost-to-go
    Mat6 Vxx = cip.Qf_outer;                                                    // Hessian of the cost-to-go
    Mat6 A;
    Mat6x3 B;

    // The model is linear, so A and B are the same at every step
    outer_linearize(A, B);

    for (int k = NH_O - 1; k >= 0; --k)
    {
        // Gradients of the stage cost
        const Vec6 lx = cip.Q_outer * box_minus_outer(onm.X[k], onm.X_ref[k]);
        const Eigen::Vector3d lu = cip.R_outer * (onm.U[k] - onm.U_ref[k]);

        // Expansion of the Q function
        const Vec6 Qx = lx + A.transpose() * Vx;
        const Eigen::Vector3d Qu = lu + B.transpose() * Vx;
        const Mat6 Qxx = cip.Q_outer + A.transpose() * Vxx * A;
        const Eigen::Matrix3d Quu = cip.R_outer + B.transpose() * Vxx * B
                                  + cip.regularization_outer * Eigen::Matrix3d::Identity();
        const Mat3x6 Qux = B.transpose() * Vxx * A;

        // Solve for the gains (3x3 solve): k_ff = -Quu^-1 Qu, K_fb = -Quu^-1 Qux
        const auto ldlt = Quu.ldlt();
        onm.k_ff[k] = -ldlt.solve(Qu);
        onm.K_fb[k] = -ldlt.solve(Qux);

        // Propagate the cost-to-go and keep the Hessian symmetric
        Vx  = Qx  + Qux.transpose() * onm.k_ff[k];
        Vxx = Qxx + Qux.transpose() * onm.K_fb[k];
        Vxx = 0.5 * (Vxx + Vxx.transpose());
    }
}

// Function to simulate the outer nominal model with the new gains
//   - alpha scales only the feed-forward step; the feedback keeps the candidate close to the linearized plan.
//   - Writes the candidate into X_new/U_new; outer_solve_ocp() keeps it only if the cost went down.
double nmpc_hybrid::outer_forward_pass(const double alpha)
{
    double c = 0.0;
    onm.X_new[0] = onm.x0;
    for (int k = 0; k < NH_O; ++k)
    {
        // Deviation from the current plan and the corrected input
        const Vec6 dx = box_minus_outer(onm.X_new[k], onm.X[k]);
        Eigen::Vector3d mu = onm.U[k] + alpha * onm.k_ff[k] + onm.K_fb[k] * dx;

        // Respect the thrust and tilt limits
        mu = outer_project_force(mu);

        // Store the input, add its cost and step the nominal model
        onm.U_new[k] = mu;
        c += outer_stage_cost(k, onm.X_new[k], mu);
        onm.X_new[k + 1] = outer_predict_step(onm.X_new[k], mu);
    }
    return c + outer_terminal_cost(onm.X_new[NH_O]);
}

// Function to solve the outer optimal control problem
void nmpc_hybrid::outer_solve_ocp()
{
    // Rollout the warm start from the measured state and reset the per solve statistics
    onm.cost_initial = outer_rollout();
    onm.cost = onm.cost_initial;
    onm.iterations = 0;
    onm.alpha = 0.0;

    for (int it = 0; it < cip.max_iterations_outer; ++it)
    {
        // Gains about the current plan
        outer_backward_pass();

        // Line search: halve the step until the cost decreases
        bool improved = false;
        double alpha = 1.0;
        for (int ls = 0; ls < cip.line_search_steps_outer; ++ls, alpha *= 0.5)
        {
            const double cost_new = outer_forward_pass(alpha);
            if (cost_new < onm.cost)
            {
                // Accept the candidate as the new plan
                onm.X = onm.X_new;
                onm.U = onm.U_new;
                onm.cost = cost_new;
                onm.alpha = alpha;
                improved = true;
                break;
            }
        }

        // Nothing better was found so stop early
        if (!improved) break;
        ++onm.iterations;
    }
}

// Function to shift the outer warm start
//   - The step length equals the sampling period, so the plan is shifted by exactly one step. The last input is
//     repeated.
void nmpc_hybrid::outer_shift_warm_start()
{
    for (int k = 0; k < NH_O - 1; ++k)
    {
        onm.U[k] = onm.U[k + 1];
    }
}

// =====================================================================================================================
// OUTER (TRANSLATIONAL) HYBRID MRAC
// =====================================================================================================================

// Function to compute the outerloop control in I (Eqs. (28)-(30), (49)-(50), (66)-(72))
void nmpc_hybrid::compute_translational_control_in_I()
{
    // Compute the error in the states
    cim.e_tran << cim.x_tran - csm.x_tran_ref;
    cim.e_tran_pos << cim.e_tran.head<3>();
    cim.e_tran_vel << cim.e_tran.tail<3>();

    // Reference model (Eq. (3) with Eq. (70))
    cim.x_tran_ref_dot << cip.A_ref_tran * csm.x_tran_ref
                        + cip.B_ref_tran * cim.r_cmd_tran;

    // Outerloop regressor (Eq. (67)): constant term (weight) and drag relative to the wind estimate
    const Eigen::Vector3d v_rel = cim.x_tran_vel - cip.wind_estimate;
    cim.outer_loop_regressor << 1.0,
                                v_rel.norm() * v_rel;

    // Compute the augmented regressor vector with the baseline (Eq. (29))
    cim.augmented_outer_loop_regressor << cim.outer_loop_regressor,
                                          cim.mu_tran_baseline;

    // Cache the transpose of the tracking error * P * B
    Eigen::Matrix<double, 1, 3> e_transpose_p_b = cim.e_tran.transpose() * cip.P_tran * cip.B_tran;

    // Robustification: the paper uses projection only (dead_zone = 1, sigma = 0)
    double sigma_x = 0.0, sigma_r = 0.0, sigma_Theta = 0.0;
    cim.dead_zone_value_translational = 1.0;
    if (cip.use_dead_zone_emod)
    {
        // Computing the scalar value output from the dead-zone modification modulation function
        cim.dead_zone_value_translational = deadZoneModulationFunction(cim.e_tran.transpose(),
                                                                                                    cip.dead_zone_delta_translational,
                                                                                                    cip.dead_zone_e0_translational);
        sigma_x = cip.sigma_x_translational;
        sigma_r = cip.sigma_r_translational;
        sigma_Theta = cip.sigma_Theta_translational;
    }

    // Adaptive laws (Eqs. (17)-(18)). pi = [x; r; -Phi_tilde], so Theta_hat has the opposite sign of K_hat_x, K_hat_r
    cim.K_hat_x_tran_dot = AdaptiveLawDeadZoneEMod(-cip.Gamma_x_tran,
                                                                                cim.dead_zone_value_translational,
                                                                                cim.x_tran,
                                                                                e_transpose_p_b,
                                                                                sigma_x,
                                                                                csm.K_hat_x_tran);

    cim.K_hat_r_tran_dot = AdaptiveLawDeadZoneEMod(-cip.Gamma_r_tran,
                                                                                cim.dead_zone_value_translational,
                                                                                cim.r_cmd_tran,
                                                                                e_transpose_p_b,
                                                                                sigma_r,
                                                                                csm.K_hat_r_tran);

    cim.Theta_hat_tran_dot = AdaptiveLawDeadZoneEMod(cip.Gamma_Theta_tran,
                                                                                  cim.dead_zone_value_translational,
                                                                                  cim.augmented_outer_loop_regressor,
                                                                                  e_transpose_p_b,
                                                                                  sigma_Theta,
                                                                                  csm.Theta_hat_tran);

    // Projection operator - Ball
    // Projection operator K_hat_x
    MatrixProjectionOutput<decltype(csm.K_hat_x_tran)> proj_op_output_K_hat_x_translational =
        _ball_::projectionMatrix(csm.K_hat_x_tran,
                                                                    cim.K_hat_x_tran_dot,
                                                                    cip.projection_x_max_x_translational,
                                                                    cip.projection_epsilon_x_translational);

    cim.K_hat_x_tran_dot = proj_op_output_K_hat_x_translational.projected_matrix;
    cim.proj_op_activated_K_hat_x_translational = proj_op_output_K_hat_x_translational.projection_operator_activated;

    // Projection operator K_hat_r
    MatrixProjectionOutput<decltype(csm.K_hat_r_tran)> proj_op_output_K_hat_r_translational =
        _ball_::projectionMatrix(csm.K_hat_r_tran,
                                                                    cim.K_hat_r_tran_dot,
                                                                    cip.projection_x_max_r_translational,
                                                                    cip.projection_epsilon_r_translational);

    cim.K_hat_r_tran_dot = proj_op_output_K_hat_r_translational.projected_matrix;
    cim.proj_op_activated_K_hat_r_translational = proj_op_output_K_hat_r_translational.projection_operator_activated;

    // Projection operator Theta_hat
    MatrixProjectionOutput<decltype(csm.Theta_hat_tran)> proj_op_output_Theta_hat_translational =
        _ball_::projectionMatrix(csm.Theta_hat_tran,
                                                                    cim.Theta_hat_tran_dot,
                                                                    cip.projection_x_max_Theta_translational,
                                                                    cip.projection_epsilon_Theta_translational);

    cim.Theta_hat_tran_dot = proj_op_output_Theta_hat_translational.projected_matrix;
    cim.proj_op_activated_Theta_hat_translational = proj_op_output_Theta_hat_translational.projection_operator_activated;

    // Adaptive control law (Eq. (49)): u_MRAC = K_hat' pi
    cim.mu_tran_adaptive << csm.K_hat_x_tran.transpose() * cim.x_tran
                          + csm.K_hat_r_tran.transpose() * cim.r_cmd_tran
                          - csm.Theta_hat_tran.transpose() * cim.augmented_outer_loop_regressor;

    // Compute the total baseline + adaptive control (Eq. (28))
    cim.mu_tran_I << cim.mu_tran_baseline + cim.mu_tran_adaptive;
}

// =====================================================================================================================
// COMMAND GENERATOR
// =====================================================================================================================

// Function to compute the rate of the outerloop control input
//   - mu jumps at every outer NMPC solve (held baseline), so the attitude command needs a smooth command generator
//     instead of differentiating the held signal (note below Eq. (63)).
void nmpc_hybrid::compute_translational_control_rate()
{
    // Compute the internal state for rate of change of mu
    cim.internal_state_mu_x_filter << cip.A_filter_mu * csm.state_mu_x_filter
                                    + cip.B_filter_mu * cim.mu_tran_I(0);

    cim.internal_state_mu_y_filter << cip.A_filter_mu * csm.state_mu_y_filter
                                    + cip.B_filter_mu * cim.mu_tran_I(1);

    cim.internal_state_mu_z_filter << cip.A_filter_mu * csm.state_mu_z_filter
                                    + cip.B_filter_mu * cim.mu_tran_I(2);

    // Compute the filtered translational virtual control input (low pass output of the same filter)
    cim.mu_tran_I_filtered(0) = cip.C_filter_mu_lowpass * csm.state_mu_x_filter;
    cim.mu_tran_I_filtered(1) = cip.C_filter_mu_lowpass * csm.state_mu_y_filter;
    cim.mu_tran_I_filtered(2) = cip.C_filter_mu_lowpass * csm.state_mu_z_filter;

    // Compute the rate of change of the filtered translational virtual control input
    cim.mu_tran_I_dot(0) = cip.C_filter_mu * csm.state_mu_x_filter;
    cim.mu_tran_I_dot(1) = cip.C_filter_mu * csm.state_mu_y_filter;
    cim.mu_tran_I_dot(2) = cip.C_filter_mu * csm.state_mu_z_filter;
}

// Function to propagate one axis of the mu filter by dt with a held input (RK4 on x_dot = A x + B u)
//   - This is the nominal command generator model used to build the inner NMPC preview (text of Eq. (87)).
Eigen::Matrix<double, 2, 1> nmpc_hybrid::propagate_mu_filter(const Eigen::Matrix<double, 2, 1>& state,
                                                             const double u,
                                                             const double dt) const
{
    const Eigen::Matrix<double, 2, 1> Bu = cip.B_filter_mu * u;
    const Eigen::Matrix<double, 2, 1> k1 = cip.A_filter_mu * state + Bu;
    const Eigen::Matrix<double, 2, 1> k2 = cip.A_filter_mu * (state + 0.5 * dt * k1) + Bu;
    const Eigen::Matrix<double, 2, 1> k3 = cip.A_filter_mu * (state + 0.5 * dt * k2) + Bu;
    const Eigen::Matrix<double, 2, 1> k4 = cip.A_filter_mu * (state + dt * k3) + Bu;
    return state + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

// Function to map a force and a heading to a desired rotation matrix (Eqs. (60)-(62))
//   - Used by compute_u1_R_d() and by the inner NMPC preview.
Eigen::Matrix3d nmpc_hybrid::compute_R_d_from_force(const Eigen::Vector3d& mu, const double psi) const
{
    // No defined thrust direction, keep the current desired attitude
    const double norm_mu = mu.norm();
    if (norm_mu < 1e-6) { return cim.R_d; }

    // Desired body z axis and heading vector
    const Eigen::Vector3d b3d = -mu / norm_mu;
    const Eigen::Vector3d c1(std::cos(psi), std::sin(psi), 0.0);

    // Construct an orthonormal triad
    const Eigen::Vector3d n_hat = b3d.cross(c1);
    const Eigen::Vector3d b2d = n_hat / n_hat.norm();
    const Eigen::Vector3d b1d = b2d.cross(b3d);

    Eigen::Matrix3d R_d;
    R_d.col(0) = b1d;
    R_d.col(1) = b2d;
    R_d.col(2) = b3d;
    return R_d;
}

// Compute the desired total thrust, the desired rotation matrix and the desired angular velocity (Eqs. (60)-(63))
//   - The thrust magnitude uses the force mu directly (realized instantly by the motors).
//   - The attitude uses the filtered force from the command generator, so R_d is smooth even though mu_baseline is
//     held and jumps at every outer NMPC solve (note below Eq. (63)). R_d and Omega_d are a consistent pair since
//     mu_dot is the rate of the same filtered force.
void nmpc_hybrid::compute_u1_R_d()
{
    // Compute the desired total thrust
    cim.u(0) = cim.mu_tran_I.norm();

    // Compute the desired body frame z axis from the filtered force
    const double norm_mu_filtered = cim.mu_tran_I_filtered.norm();
    cim.b3d = -cim.mu_tran_I_filtered / norm_mu_filtered;

    // Compute the desired "heading" vector
    cim.c1 << std::cos(cim.psi_user),
              std::sin(cim.psi_user),
              0.0;

    // Construct an orthonormal triad
    cim.n_hat = cim.b3d.cross(cim.c1);
    cim.sigma = cim.n_hat.norm();
    cim.b2d = cim.n_hat / cim.sigma;

    cim.b1d = cim.b2d.cross(cim.b3d);

    // Construct the desired rotation matrix
    cim.R_d.col(0) = cim.b1d;
    cim.R_d.col(1) = cim.b2d;
    cim.R_d.col(2) = cim.b3d;

    // Compute R_d_dot
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    // compute b3d_dot
    cim.b3d_dot = -1.0 * (1 / norm_mu_filtered) * (I - cim.b3d * cim.b3d.transpose()) * cim.mu_tran_I_dot;

    // compute b2d_dot
    cim.c1_dot << -1.0 * cim.psi_dot_user * std::sin(cim.psi_user),
                   cim.psi_dot_user * std::cos(cim.psi_user),
                   0.0;

    cim.n_hat_dot = cim.b3d_dot.cross(cim.c1) + cim.b3d.cross(cim.c1_dot);

    cim.b2d_dot = (1 / cim.sigma) * (I - cim.b2d * cim.b2d.transpose()) * cim.n_hat_dot;

    // compute b1d_dot
    cim.b1d_dot = cim.b2d_dot.cross(cim.b3d) + cim.b2d.cross(cim.b3d_dot);

    // assign to R_d_dot
    cim.R_d_dot.col(0) = cim.b1d_dot;
    cim.R_d_dot.col(1) = cim.b2d_dot;
    cim.R_d_dot.col(2) = cim.b3d_dot;

    // Desired angular velocity in the desired frame (Eq. (63)): Omega_d = (R_d^T R_d_dot)^v
    cim.Omega_d = veeMap(cim.R_d.transpose() * cim.R_d_dot);

    // Desired angular velocity in the current body frame (first term of Eq. (78))
    cim.omega_d = cim.Rij * cim.R_d * cim.Omega_d;
}

// =====================================================================================================================
// INNER (ATTITUDE) NMPC
// =====================================================================================================================

// Function to sample, solve and hold the inner NMPC
//   - The NMPC is solved every dt_inner from the measured [R; omega] and its first input is held as the baseline
//     until the next solve (Eq. (87)).
void nmpc_hybrid::compute_inner_nmpc()
{
    inm.solved_this_tick = false;

    // 1. Solve at the sampling instants ----------------------------------------------------------------------------
    if (inm.first_solve || cim.t >= inm.t_solve + cip.dt_inner - 1e-9)
    {
        // Build the preview of R_d and Omega_d
        compute_inner_reference_horizon();

        // Every plan starts at the measured state
        inm.x0.R = cim.Rji;
        inm.x0.w = cim.omega;

        // Solve the optimal control problem
        inner_solve_ocp();

        // Hold the first input
        inm.tau_hold = inm.U[0];
        inm.t_solve = cim.t;

        // Shift the warm start for the next solve
        inner_shift_warm_start();

        inm.solved_this_tick = true;
        inm.first_solve = false;
    }

    // 2. Baseline torque (zero-order hold of the first planned input) ----------------------------------------------
    cim.tau_rot_baseline = inm.tau_hold;
}

// Function to build the inner preview [R_d; Omega_d] over the horizon (text of Eq. (87))
//   - Initialized from the command generator state (the mu filter states) and propagated with the nominal command
//     generator model driven by the nominal outer plan: at t_j the input is the outer plan force that is applied
//     (held over each outer step, exactly like the baseline) plus the CURRENT adaptive force held constant.
//   - j = 0 uses the current filter states, so R_d,0 is exactly the R_d of the command generator.
//   - The filter output is smooth, so Omega_d and its rate (finite differences of the preview, consistent with
//     R+ = R Exp(w dt)) do not see the jumps of the held baseline.
void nmpc_hybrid::compute_inner_reference_horizon()
{
    const double dt = cip.dt_inner;

    // 1. Desired attitude over the horizon from the propagated command generator ---------------------------------
    Eigen::Matrix<double, 2, 1> x_filter = csm.state_mu_x_filter;
    Eigen::Matrix<double, 2, 1> y_filter = csm.state_mu_y_filter;
    Eigen::Matrix<double, 2, 1> z_filter = csm.state_mu_z_filter;

    for (int j = 0; j <= NH_I; ++j)
    {
        // Filtered force and heading at t_j
        Eigen::Vector3d mu_filtered;
        mu_filtered << cip.C_filter_mu_lowpass * x_filter,
                       cip.C_filter_mu_lowpass * y_filter,
                       cip.C_filter_mu_lowpass * z_filter;
        const double psi_j = cim.psi_user + cim.psi_dot_user * j * dt;

        inm.X_ref[j].R = compute_R_d_from_force(mu_filtered, psi_j);

        // No propagation needed after the last preview point
        if (j == NH_I) { break; }

        // Force applied over [t_j, t_j + dt): the current force for j = 0, then the held outer plan + held mu_adaptive
        Eigen::Vector3d mu_j = cim.mu_tran_I;
        if (j > 0)
        {
            const double t_j = cim.t + j * dt;
            int i = static_cast<int>(std::floor((t_j - onm.t_solve) / cip.dt_outer + 1e-9));
            i = std::min(std::max(i, 0), NH_O - 1);
            mu_j = onm.U_plan[i] + cim.mu_tran_adaptive;
        }

        // Propagate the command generator model
        x_filter = propagate_mu_filter(x_filter, mu_j(0), dt);
        y_filter = propagate_mu_filter(y_filter, mu_j(1), dt);
        z_filter = propagate_mu_filter(z_filter, mu_j(2), dt);
    }

    // 2. Desired angular velocity (inverts R+ = R Exp(w dt), finite difference on SO(3)) -------------------------
    for (int j = 0; j < NH_I; ++j)
    {
        inm.X_ref[j].w = logSO3(inm.X_ref[j].R.transpose() * inm.X_ref[j + 1].R) / dt;
    }
    // No next orientation after the last step, so hold the previous angular velocity
    inm.X_ref[NH_I].w = inm.X_ref[NH_I - 1].w;

    // 3. Desired angular acceleration --------------------------------------------------------------------------------
    for (int j = 0; j < NH_I; ++j)
    {
        inm.Omega_d_dot[j] = (inm.X_ref[j + 1].w - inm.X_ref[j].w) / dt;
    }

    // 4. Total thrust used for the torque limits ---------------------------------------------------------------------
    inm.thrust_preview = cim.u(0);
}

// Function to propagate the inner nominal model by one step (Eqs. (82)-(83))
inner_state nmpc_hybrid::inner_predict_step(const inner_state& x, const Eigen::Vector3d& tau) const
{
    const double dt = cip.dt_inner;

    // Explicit Euler step (the rotation is propagated with the exponential map so it stays on SO(3))
    inner_state n;
    n.R = x.R * expSO3(x.w * dt);                                           // Attitude
    n.w = x.w + cip.J_inv * (tau - x.w.cross(cip.J * x.w)) * dt;            // Euler's rotation equation
    return n;
}

// Function to linearize the inner nominal model in the error state d = [dtheta, dw] with R = R_bar Exp(dtheta)
void nmpc_hybrid::inner_linearize(const inner_state& x, Mat6& A, Mat6x3& B) const
{
    const double dt = cip.dt_inner;
    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

    // State Jacobian (starts at identity: every error carries over to the next step, then the couplings are added)
    A.setIdentity();
    A.block<3, 3>(0, 0) = expSO3(x.w * dt).transpose();                                                 // dtheta <- dtheta
    A.block<3, 3>(0, 3) = I3 * dt;                                                                      // dtheta <- dw (first order)
    A.block<3, 3>(3, 3) += cip.J_inv * (skewSymmetric(cip.J * x.w) - skewSymmetric(x.w) * cip.J) * dt;  // dw     <- dw

    // Input Jacobian
    B.setZero();
    B.block<3, 3>(3, 0) = cip.J_inv * dt;                                                               // dw     <- tau
}

// Function to compute the inner residual at step k and its Jacobian with respect to [dtheta; dw] (Eq. (84))
//   - eta     = Log(R_d^T R)                     -> d eta / d dtheta ~ I   (first order, as in the box minus)
//   - e_omega = omega - R^T R_d Omega_d = w - v  -> d e_omega / d dtheta = -skew(v), d e_omega / d dw = I
//     since R^T ~ (I - skew(dtheta)) R_bar^T gives R^T R_d Omega_d ~ v - skew(dtheta) v = v + skew(v) dtheta
Vec6 nmpc_hybrid::inner_residual(const int k, const inner_state& x, Mat6& E) const
{
    const inner_state& ref = inm.X_ref[k];

    // Desired angular velocity expressed in the predicted body frame
    const Eigen::Vector3d v = x.R.transpose() * ref.R * ref.w;

    Vec6 r;
    r << logSO3(ref.R.transpose() * x.R),
         x.w - v;

    E.setIdentity();
    E.block<3, 3>(3, 0) = -skewSymmetric(v);

    return r;
}

// Function to compute the inner feedforward torque at step k (Eq. (86))
//   - tau_ff = R^T R_d (J Omega_d_dot + Omega_d x J Omega_d)
Eigen::Vector3d nmpc_hybrid::inner_feedforward_torque(const int k, const inner_state& x) const
{
    const inner_state& ref = inm.X_ref[k];
    return x.R.transpose() * ref.R * (cip.J * inm.Omega_d_dot[k] + ref.w.cross(cip.J * ref.w));
}

// Function to scale the torque so that every motor stays within its thrust limits
//   - Motor thrusts are f = f_T + s f_tau with f_T = mixer [T; 0] and f_tau = mixer [0; tau]. The largest s in [0, 1]
//     that keeps every motor within [MIN_THRUST, MAX_THRUST] is used, so the torque direction is preserved.
Eigen::Vector3d nmpc_hybrid::inner_clamp_torque(const Eigen::Vector3d& tau) const
{
    Eigen::Matrix<double, 4, 1> wrench_T;
    Eigen::Matrix<double, 4, 1> wrench_tau;
    wrench_T << inm.thrust_preview, 0.0, 0.0, 0.0;
    wrench_tau << 0.0, tau;

    const Eigen::Matrix<double, 4, 1> f_T = mixer_matrix_quadm * wrench_T;
    const Eigen::Matrix<double, 4, 1> f_tau = mixer_matrix_quadm * wrench_tau;

    // Largest admissible scaling over the four motors
    double s = 1.0;
    for (int i = 0; i < 4; ++i)
    {
        if (f_tau(i) > 1e-12) {
            s = std::min(s, (MAX_THRUST - f_T(i)) / f_tau(i));
        }
        else if (f_tau(i) < -1e-12) {
            s = std::min(s, (MIN_THRUST - f_T(i)) / f_tau(i));
        }
    }

    // If the thrust alone already violates the limits, no torque is admissible
    s = std::min(std::max(s, 0.0), 1.0);

    return s * tau;
}

// Function to compute the inner stage cost at step k (Eq. (85))
double nmpc_hybrid::inner_stage_cost(const int k, const inner_state& x, const Eigen::Vector3d& tau) const
{
    Mat6 E;
    const Vec6 r = inner_residual(k, x, E);
    const Eigen::Vector3d dtau = tau - inner_feedforward_torque(k, x);

    // 0.5 r'Qr + 0.5 dtau'R dtau
    return 0.5 * r.dot(cip.Q_inner * r) + 0.5 * dtau.dot(cip.R_inner * dtau);
}

// Function to compute the inner terminal cost
double nmpc_hybrid::inner_terminal_cost(const inner_state& x) const
{
    Mat6 E;
    const Vec6 r = inner_residual(NH_I, x, E);
    return 0.5 * r.dot(cip.Qf_inner * r);
}

// Function to simulate the inner warm start from the measured state
double nmpc_hybrid::inner_rollout()
{
    double c = 0.0;
    inm.X[0] = inm.x0;      // Every plan starts at the measured state
    for (int k = 0; k < NH_I; ++k)
    {
        c += inner_stage_cost(k, inm.X[k], inm.U[k]);
        inm.X[k + 1] = inner_predict_step(inm.X[k], inm.U[k]);
    }
    return c + inner_terminal_cost(inm.X[NH_I]);
}

// Function to compute the inner feedback and feed-forward gains by walking backward from the end of the horizon
//   - Gauss-Newton: the residual Jacobian E gives lx = E'Qr and lxx = E'QE. The dependence of tau_ff on R is not
//     differentiated (it is re-evaluated in every rollout).
void nmpc_hybrid::inner_backward_pass()
{
    Mat6 E;
    Mat6 A;
    Mat6x3 B;

    // Start from the terminal cost
    Vec6 r = inner_residual(NH_I, inm.X[NH_I], E);
    Vec6 Vx  = E.transpose() * cip.Qf_inner * r;            // Gradient of the cost-to-go
    Mat6 Vxx = E.transpose() * cip.Qf_inner * E;            // Hessian of the cost-to-go

    for (int k = NH_I - 1; k >= 0; --k)
    {
        // Linearize about the current plan
        inner_linearize(inm.X[k], A, B);

        // Gradients of the stage cost
        r = inner_residual(k, inm.X[k], E);
        const Vec6 lx = E.transpose() * cip.Q_inner * r;
        const Mat6 lxx = E.transpose() * cip.Q_inner * E;
        const Eigen::Vector3d lu = cip.R_inner * (inm.U[k] - inner_feedforward_torque(k, inm.X[k]));

        // Expansion of the Q function (Gauss-Newton: no second order dynamics terms)
        const Vec6 Qx = lx + A.transpose() * Vx;
        const Eigen::Vector3d Qu = lu + B.transpose() * Vx;
        const Mat6 Qxx = lxx + A.transpose() * Vxx * A;
        const Eigen::Matrix3d Quu = cip.R_inner + B.transpose() * Vxx * B
                                  + cip.regularization_inner * Eigen::Matrix3d::Identity();
        const Mat3x6 Qux = B.transpose() * Vxx * A;

        // Solve for the gains (3x3 solve): k_ff = -Quu^-1 Qu, K_fb = -Quu^-1 Qux
        const auto ldlt = Quu.ldlt();
        inm.k_ff[k] = -ldlt.solve(Qu);
        inm.K_fb[k] = -ldlt.solve(Qux);

        // Propagate the cost-to-go and keep the Hessian symmetric
        Vx  = Qx  + Qux.transpose() * inm.k_ff[k];
        Vxx = Qxx + Qux.transpose() * inm.K_fb[k];
        Vxx = 0.5 * (Vxx + Vxx.transpose());
    }
}

// Function to simulate the inner nominal model with the new gains
double nmpc_hybrid::inner_forward_pass(const double alpha)
{
    double c = 0.0;
    inm.X_new[0] = inm.x0;
    for (int k = 0; k < NH_I; ++k)
    {
        // Deviation from the current plan and the corrected input
        const Vec6 dx = box_minus_inner(inm.X_new[k], inm.X[k]);
        Eigen::Vector3d tau = inm.U[k] + alpha * inm.k_ff[k] + inm.K_fb[k] * dx;

        // Respect the motor thrust limits
        tau = inner_clamp_torque(tau);

        // Store the input, add its cost and step the nominal model
        inm.U_new[k] = tau;
        c += inner_stage_cost(k, inm.X_new[k], tau);
        inm.X_new[k + 1] = inner_predict_step(inm.X_new[k], tau);
    }
    return c + inner_terminal_cost(inm.X_new[NH_I]);
}

// Function to solve the inner optimal control problem
void nmpc_hybrid::inner_solve_ocp()
{
    // Rollout the warm start from the measured state and reset the per solve statistics
    inm.cost_initial = inner_rollout();
    inm.cost = inm.cost_initial;
    inm.iterations = 0;
    inm.alpha = 0.0;

    for (int it = 0; it < cip.max_iterations_inner; ++it)
    {
        // Gains about the current plan
        inner_backward_pass();

        // Line search: halve the step until the cost decreases
        bool improved = false;
        double alpha = 1.0;
        for (int ls = 0; ls < cip.line_search_steps_inner; ++ls, alpha *= 0.5)
        {
            const double cost_new = inner_forward_pass(alpha);
            if (cost_new < inm.cost)
            {
                // Accept the candidate as the new plan
                inm.X = inm.X_new;
                inm.U = inm.U_new;
                inm.cost = cost_new;
                inm.alpha = alpha;
                improved = true;
                break;
            }
        }

        // Nothing better was found so stop early
        if (!improved) break;
        ++inm.iterations;
    }
}

// Function to shift the inner warm start
void nmpc_hybrid::inner_shift_warm_start()
{
    for (int k = 0; k < NH_I - 1; ++k)
    {
        inm.U[k] = inm.U[k + 1];
    }
}

// Function to compute the infinite horizon LQR cost-to-go of x+ = A x + B u with stage cost 0.5 x'Qx + 0.5 u'Ru
//   - Iterates the discrete Riccati equation P = Q + A'PA - A'PB (R + B'PB)^-1 B'PA until it converges.
Mat6 nmpc_hybrid::compute_lqr_terminal_cost(const Mat6& A, const Mat6x3& B, const Mat6& Q, const Eigen::Matrix3d& R) const
{
    Mat6 P = Q;
    for (int it = 0; it < 100000; ++it)
    {
        const Eigen::Matrix3d S = R + B.transpose() * P * B;
        const Mat3x6 K = S.ldlt().solve(B.transpose() * P * A);
        Mat6 P_next = Q + A.transpose() * P * A - A.transpose() * P * B * K;
        P_next = 0.5 * (P_next + P_next.transpose());

        // Converged
        if ((P_next - P).norm() <= 1e-10 * P.norm()) { return P_next; }
        P = P_next;
    }

    FLIGHTSTACK_INFO("NMPC HYBRID LQR TERMINAL COST DID NOT FULLY CONVERGE");
    return P;
}

// =====================================================================================================================
// INNER (ROTATIONAL) MRAC
// =====================================================================================================================

// Compute the rotational control (Eqs. (73)-(80))
void nmpc_hybrid::compute_rotational_control()
{
    // Compute the attitude error (Eq. (78)): e_R = 0.5 (R_d^T R - R^T R_d)^v
    cim.e_R = 0.5 * veeMap(cim.R_d.transpose() * cim.Rji - cim.Rij * cim.R_d);

    // Compute the attitude corrected angular velocity command (Eq. (78))
    cim.omega_c = cim.omega_d - cip.k_R * cim.e_R;

    // Compute the reference command and the reference model (Eq. (79)): omega_ref_dot = -K_omega omega_ref + K_omega omega_c
    cim.r_cmd_rot = cip.K_omega_ref * cim.omega_c;
    cim.omega_ref_dot << cip.A_ref_rot * csm.omega_ref + cip.B_ref_rot * cim.r_cmd_rot;

    // Compute the error in the angular velocities
    cim.omega_e << cim.omega - csm.omega_ref;

    // Innerloop regressor (Eq. (74)), keeps the product of inertia terms
    cim.inner_loop_regressor << cim.omega(0) * cim.omega(1),
                                cim.omega(0) * cim.omega(2),
                                cim.omega(1) * cim.omega(2),
                                cim.omega(2) * cim.omega(2) - cim.omega(0) * cim.omega(0);

    // Compute the augmented regressor vector with the baseline (Eq. (29))
    cim.augmented_inner_loop_regressor << cim.inner_loop_regressor,
                                          cim.tau_rot_baseline;

    // Cache the transpose of the tracking error * P * B
    Eigen::Matrix<double, 1, 3> e_transpose_p_b = cim.omega_e.transpose() * cip.P_rot * cip.B_rot;

    // Robustification: the paper uses projection only (dead_zone = 1, sigma = 0)
    double sigma_x = 0.0, sigma_r = 0.0, sigma_Theta = 0.0;
    cim.dead_zone_value_rotational = 1.0;
    if (cip.use_dead_zone_emod)
    {
        // Computing the scalar value output from the dead-zone modification modulation function
        cim.dead_zone_value_rotational = deadZoneModulationFunction(cim.omega_e.transpose(),
                                                                                                 cip.dead_zone_delta_rotational,
                                                                                                 cip.dead_zone_e0_rotational);
        sigma_x = cip.sigma_x_rotational;
        sigma_r = cip.sigma_r_rotational;
        sigma_Theta = cip.sigma_Theta_rotational;
    }

    // Adaptive laws (Eqs. (17)-(18))
    cim.K_hat_x_rot_dot = AdaptiveLawDeadZoneEMod(-cip.Gamma_x_rot,
                                                                               cim.dead_zone_value_rotational,
                                                                               cim.omega,
                                                                               e_transpose_p_b,
                                                                               sigma_x,
                                                                               csm.K_hat_x_rot);

    cim.K_hat_r_rot_dot = AdaptiveLawDeadZoneEMod(-cip.Gamma_r_rot,
                                                                               cim.dead_zone_value_rotational,
                                                                               cim.r_cmd_rot,
                                                                               e_transpose_p_b,
                                                                               sigma_r,
                                                                               csm.K_hat_r_rot);

    cim.Theta_hat_rot_dot = AdaptiveLawDeadZoneEMod(cip.Gamma_Theta_rot,
                                                                                 cim.dead_zone_value_rotational,
                                                                                 cim.augmented_inner_loop_regressor,
                                                                                 e_transpose_p_b,
                                                                                 sigma_Theta,
                                                                                 csm.Theta_hat_rot);

    // Projection operator - Ball
    // Projection operator K_hat_x
    MatrixProjectionOutput<decltype(csm.K_hat_x_rot)> proj_op_output_K_hat_x_rotational =
        _ball_::projectionMatrix(csm.K_hat_x_rot,
                                                                    cim.K_hat_x_rot_dot,
                                                                    cip.projection_x_max_x_rotational,
                                                                    cip.projection_epsilon_x_rotational);

    cim.K_hat_x_rot_dot = proj_op_output_K_hat_x_rotational.projected_matrix;
    cim.proj_op_activated_K_hat_x_rotational = proj_op_output_K_hat_x_rotational.projection_operator_activated;

    // Projection operator K_hat_r
    MatrixProjectionOutput<decltype(csm.K_hat_r_rot)> proj_op_output_K_hat_r_rotational =
        _ball_::projectionMatrix(csm.K_hat_r_rot,
                                                                    cim.K_hat_r_rot_dot,
                                                                    cip.projection_x_max_r_rotational,
                                                                    cip.projection_epsilon_r_rotational);

    cim.K_hat_r_rot_dot = proj_op_output_K_hat_r_rotational.projected_matrix;
    cim.proj_op_activated_K_hat_r_rotational = proj_op_output_K_hat_r_rotational.projection_operator_activated;

    // Projection operator Theta_hat
    MatrixProjectionOutput<decltype(csm.Theta_hat_rot)> proj_op_output_Theta_hat_rotational =
        _ball_::projectionMatrix(csm.Theta_hat_rot,
                                                                    cim.Theta_hat_rot_dot,
                                                                    cip.projection_x_max_Theta_rotational,
                                                                    cip.projection_epsilon_Theta_rotational);

    cim.Theta_hat_rot_dot = proj_op_output_Theta_hat_rotational.projected_matrix;
    cim.proj_op_activated_Theta_hat_rotational = proj_op_output_Theta_hat_rotational.projection_operator_activated;

    // Adaptive control law (Eq. (49)): u_MRAC = K_hat' pi with pi = [omega; r_in; -Phi_tilde]
    cim.tau_rot_adaptive << csm.K_hat_x_rot.transpose() * cim.omega
                          + csm.K_hat_r_rot.transpose() * cim.r_cmd_rot
                          - csm.Theta_hat_rot.transpose() * cim.augmented_inner_loop_regressor;

    // Total rotational control input (Eq. (28))
    cim.tau_rot << cim.tau_rot_baseline + cim.tau_rot_adaptive;

    // Assign the control inputs
    cim.u(1) = cim.tau_rot(0);
    cim.u(2) = cim.tau_rot(1);
    cim.u(3) = cim.tau_rot(2);
}

// Function to compute the normalized thrusts
void nmpc_hybrid::compute_normalized_thrusts()
{
    // Compute the individual thrusts in Newtons
    cim.Thrust << mixer_matrix_quadm * cim.u;

    // Saturate each element of the Thrust vector between MIN_THRUST and MAX_THRUST
    cim.Sat_Thrust = (cim.Thrust.cwiseMin(MAX_THRUST).cwiseMax(MIN_THRUST));

    // Compute the final control inputs (thrust [N] -> normalized motor command [0-1] via the thrust curve)
    control_input(0) = evaluatePolynomial(thrust_polynomial_coeff_quadm, cim.Sat_Thrust(0));
    control_input(1) = evaluatePolynomial(thrust_polynomial_coeff_quadm, cim.Sat_Thrust(1));
    control_input(2) = evaluatePolynomial(thrust_polynomial_coeff_quadm, cim.Sat_Thrust(2));
    control_input(3) = evaluatePolynomial(thrust_polynomial_coeff_quadm, cim.Sat_Thrust(3));
}

// Function that is called in control.cpp
void nmpc_hybrid::run(const double time_step_rk4_) {

    // Process the dynamics --------------------------------------------------------
    // 1. Hybrid reset of the outer reference model (start of the tick: x and x_ref refer to the same time)
    compute_hybrid_reference_reset();

    // 2. Compute the disturbance estimate (switched off by default)
    compute_disturbance_estimate();

    // 3. Outer NMPC: solve every dt_outer, hold mu_baseline and compute the reference command from the plan
    compute_outer_nmpc();

    // 4. Outer hybrid MRAC: mu = mu_baseline + mu_adaptive
    compute_translational_control_in_I();

    // 5. Command generator: rate of mu, then the thrust, R_d and Omega_d
    compute_translational_control_rate();
    compute_u1_R_d();

    // 6. Inner NMPC: solve every dt_inner, hold tau_baseline
    compute_inner_nmpc();

    // 7. Inner MRAC on omega: tau = tau_baseline + tau_adaptive
    compute_rotational_control();

    // 8. Compute the normalized thrust - Final Step
    compute_normalized_thrusts();

    // 9. Do the integration (reference models, adaptive gains, filters and the d_hat integral)
    rk4.do_step(boost::bind(&nmpc_hybrid::model, this, bph::_1, bph::_2, bph::_3),
                y, cim.t, time_step_rk4_);

    // Capture the time after the execution of the controller
    cim.alg_end_time = std::chrono::high_resolution_clock::now();

    // Calculate the duration of the execution
    cim.alg_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                            cim.alg_end_time - cim.alg_start_time).count();

    // 10. Log the Data after all the calculataions
    logger.logLogData();

}

// Implementing getter functions from controller_base
float nmpc_hybrid::get_t1() const {
    return control_input(0);
}

float nmpc_hybrid::get_t2() const {
    return control_input(1);
}

float nmpc_hybrid::get_t3() const {
    return control_input(2);
}

float nmpc_hybrid::get_t4() const {
    return control_input(3);
}

float nmpc_hybrid::get_t5() const {
    return control_input(4);
}

float nmpc_hybrid::get_t6() const {
    return control_input(5);
}

float nmpc_hybrid::get_t7() const {
    return control_input(6);
}

float nmpc_hybrid::get_t8() const {
    return control_input(7);
}

} // namespace _nmpc_hybrid_
} // namespace _quadm_
