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
 * File:        NMPC_TD.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        October 12, 2026
 * For info:    Andrea L'Afflitto
 *              a.lafflitto@vt.edu
 *
 * Description: Nonlinear model predictive controller on SO(3) for the QUADM. Inherts the class controller_base for
 *              the basic functionality that is to be used for all control algorithms.
 *
 *              Prediction model (inertial NED frame, thrust along -b3, Euler discretized with dt_mpc):
 *                  p+ = p + v dt
 *                  v+ = v + (g e3 + d_hat - (T/m) R e3) dt
 *                  R+ = R Exp(w dt)
 *                  w+ = w + J^-1 (tau - w x J w) dt
 *              where [T; tau] = mixer^-1 * [f1; f2; f3; f4] and d_hat is the disturbance estimate.
 *
 *              Every control tick (run):
 *                  1. d_hat   <- Ki * integral of the position error (saturated)
 *                  2. X_ref, U_ref over the horizon from the trajectory (differential flatness)
 *                  3. Gauss-Newton/iLQR: rollout, then backward pass + line search up to max_iterations times
 *                  4. Apply the first planned motor thrusts, then shift the plan as the next warm start
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

#include "NMPC_TD.hpp"

namespace _quadm_{
namespace _nmpc_{

// -------------------------------------------------------------------------
// Constructor Implementation:
//   - Calls the base (controller_base) constructor, builds the trajectory
//     and the logger (the logger creates the log file and writes the headers)
// -------------------------------------------------------------------------
nmpc::nmpc(flight_params* p, const std::string & controller_log_dir_) :
           controller_base(p), ud(p), logger(&cim, &csm, &control_input, controller_log_dir_)
{
    // Reading in the parameters (path is relative to the workspace folder the flightstack runs from)
    read_params("./src/acsl_flight/params/control_algorithms/quadm/NMPC/gains_NMPC.json");

    // Initial Conditions
    init();
}

// Destructor
nmpc::~nmpc() {
}

// -------------------------------------------------------------------------
// read_params Implementation:
// - Takes the hardcoded path for the gains and parameters and reads it in
// -------------------------------------------------------------------------
void nmpc::read_params(const std::string& jsonFile)
{
    // Open and parse the gains file
    std::ifstream file(jsonFile);
    nlohmann::json j;
    file >> j;

    // Horizon parameters -----------------------------------------------------------------------------------------------
    cip.dt_mpc = j["HORIZON"]["dt_mpc"];
    cip.max_iterations = j["HORIZON"]["max_iterations"];
    cip.line_search_steps = j["HORIZON"]["line_search_steps"];
    cip.regularization = j["HORIZON"]["regularization"];

    // Cost matrices (stored as diagonals in the json) ------------------------------------------------------------------
    // Each is a 1xN row (times its scaling_coef) that is turned into a diagonal matrix. Q/Qf are ordered as the error
    // state [dp, dv, dtheta, dw] and R_in as the motors [f1, f2, f3, f4].
    Eigen::VectorXd q_diag = jsonToScaledMatrixXd(j["COST"]["Q_diagonal"], 1, NX).transpose();
    Eigen::VectorXd qf_diag = jsonToScaledMatrixXd(j["COST"]["Qf_diagonal"], 1, NX).transpose();
    Eigen::VectorXd r_diag = jsonToScaledMatrixXd(j["COST"]["R_diagonal"], 1, NU).transpose();
    cip.Q = q_diag.asDiagonal();
    cip.Qf = qf_diag.asDiagonal();
    cip.R_in = r_diag.asDiagonal();

    // Disturbance estimate parameters ----------------------------------------------------------------------------------
    cip.Ki_tran = jsonToScaledMatrixXd(j["DISTURBANCE"]["KI_translational"], 3, 3);
    cip.d_hat_max = jsonToScaledMatrixXd(j["DISTURBANCE"]["d_hat_max"], 1, 3).transpose();
}

// Implementing virutal functions from controller_base
void nmpc::init(){
    // Set the inital conditions (zero integral error)
    y.fill(0.0);
    dy.fill(0.0);
    csm.e_tran_pos_I.setZero();

    // Physical parameters from the quadm parameter file
    // NOTE: mixer_matrix_quadm maps [T; tau] to the motor thrusts, so its inverse maps the thrusts back to [T; tau]
    cip.J = inertia_matrix_q;
    cip.J_inv = cip.J.inverse();
    cip.mixer_inv = mixer_matrix_quadm.inverse();
    cip.f_min.setConstant(MIN_THRUST);
    cip.f_max.setConstant(MAX_THRUST);

    // Warm start: plan to hover with the weight shared equally between the motors
    const Eigen::Matrix<double, NU, 1> f_hover = Eigen::Matrix<double, NU, 1>::Constant(MASS * G / 4.0);
    cim.U.fill(f_hover);
    cim.U_ref.fill(f_hover);

    // No gains until the first backward pass
    for (auto& k : cim.K_fb) k.setZero();
    for (auto& k : cim.k_ff) k.setZero();

    // Initialize the remaining internal members
    cim.d_hat.setZero();
    cim.cost_initial = 0.0;
    cim.cost = 0.0;
    cim.iterations = 0;
    cim.alpha = 0.0;
    cim.u.setZero();
    cim.Thrust = f_hover;
    cim.Sat_Thrust = f_hover;
}

// Update function for the controller
//   - Called by the simulator before run() with the latest measured state. It only stores the state and the
//     reference, all the optimization happens in run().
void nmpc::update(double time,
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

    // Assign the quaternions and ensure it is normalized
    // q0 = w, q1 = x, q2 = y, q3 = z
    cim.q = Eigen::Quaterniond(q0, q1, q2, q3).normalized();

    // Assign the rotation matrices
    cim.Rji = cim.q.toRotationMatrix(); // rotation from the body to the inertial frame
    cim.Rij = cim.Rji.transpose();      // rotation from the inertial to the body frame

    // Assign the Euler angles and the angular velocity
    cim.eta_rot << roll, pitch, yaw;
    cim.omega << w_x, w_y, w_z;

    // Assign the measured state for the NMPC
    cim.x0.p = cim.x_tran_pos;
    cim.x0.v = cim.x_tran_vel;
    cim.x0.R = cim.Rji;
    cim.x0.w = cim.omega;

    // 2. Get the reference trajectory ---------------------------------------------
    ud.updateUserDefinedTrajectory(time);
    cim.r_user = ud.getUserDefinedPosition();
    cim.r_dot_user = ud.getUserDefinedVelocity();
    cim.r_ddot_user = ud.getUserDefinedAcceleration();
    cim.psi_user = ud.getUserDefinedYaw();

    // Compute the tracking errors (logged, and the position error is integrated for d_hat)
    cim.e_tran_pos = cim.x_tran_pos - cim.r_user;
    cim.e_tran_vel = cim.x_tran_vel - cim.r_dot_user;

    // 3. Capture the time before the execution of the controller ------------------
    cim.alg_start_time = std::chrono::high_resolution_clock::now();

    // 4. Assign the values from the integrator ------------------------------------
    assign_from_rk4();
}

// Function to assign elements from the rk4 integrator
void nmpc::assign_from_rk4()
{
    // Running position in the rk4 array (must follow the same order as model())
    int index = 0;

    //------------ assign after integration  ------------//
    assignElementsToMembers(csm.e_tran_pos_I, y, index);
}

// Model function for integration
//   - The only integrated state is the position error integral, so its derivative is the position error itself.
void nmpc::model(const rk4_array<double, NSI> &y, rk4_array<double, NSI> &dy, double t)
{
    // Running position in the rk4 array (must follow the same order as assign_from_rk4())
    int index = 0;

    //------------ Fill up the dy for integration  ------------//
    assignElementsToDxdt(cim.e_tran_pos, dy, index);
}

// Function to compute the disturbance estimate
void nmpc::compute_disturbance_estimate()
{
    // The integral of the position error is treated as a constant acceleration acting on the vehicle over the
    // whole horizon. If the vehicle sags below the reference (e_z > 0 in NED) the model sees an extra downward
    // acceleration and the NMPC adds thrust to compensate.
    cim.d_hat = cip.Ki_tran * csm.e_tran_pos_I;

    // Saturate the estimate to avoid wind-up
    cim.d_hat = cim.d_hat.cwiseMin(cip.d_hat_max).cwiseMax(-cip.d_hat_max);
}

// Function to build the reference states and inputs over the horizon
void nmpc::compute_reference_horizon()
{
    const Eigen::Vector3d e3 = e3_basis;            // Down axis in NED
    std::array<double, NH + 1> T_ref;               // Reference total thrust at each horizon step [N]

    // 1. Sample the trajectory and compute the reference orientation via differential flatness -------------------
    // NOTE: The samples are taken in increasing time so the hover/landing logic of the minjerkpoly module that
    //       stores the previous position stays consistent. The module is restored to the current time afterwards.
    for (int k = 0; k <= NH; ++k)
    {
        ud.updateUserDefinedTrajectory(cim.t + k * cip.dt_mpc);

        cim.X_ref[k].p = ud.getUserDefinedPosition();
        cim.X_ref[k].v = ud.getUserDefinedVelocity();
        const Eigen::Vector3d a_ref = ud.getUserDefinedAcceleration();
        const double psi_ref = ud.getUserDefinedYaw();

        // Required thrust vector: T b3 = m (g e3 + d_hat - a_ref)
        // (from the prediction model with v+ = v + a_ref dt). Its norm is the thrust and its direction is b3.
        const Eigen::Vector3d thrust_vec = MASS * (G * e3 + cim.d_hat - a_ref);
        T_ref[k] = thrust_vec.norm();
        // Free fall (zero thrust) has no defined b3, so fall back to level
        const Eigen::Vector3d b3 = (T_ref[k] > 1e-6) ? Eigen::Vector3d(thrust_vec / T_ref[k]) : e3;

        // Construct an orthonormal triad with the desired heading
        // c1 is the heading direction in the horizontal plane; b2 and b1 complete a right handed frame around b3
        const Eigen::Vector3d c1(std::cos(psi_ref), std::sin(psi_ref), 0.0);
        const Eigen::Vector3d b2 = b3.cross(c1).normalized();
        const Eigen::Vector3d b1 = b2.cross(b3);

        cim.X_ref[k].R.col(0) = b1;
        cim.X_ref[k].R.col(1) = b2;
        cim.X_ref[k].R.col(2) = b3;
    }

    // Restore the trajectory module to the current time
    ud.updateUserDefinedTrajectory(cim.t);

    // 2. Reference angular velocity from consecutive orientations -----------------------------------------------
    // Inverts R+ = R Exp(w dt) from the prediction model (finite difference on SO(3))
    for (int k = 0; k < NH; ++k)
    {
        cim.X_ref[k].w = logSO3(cim.X_ref[k].R.transpose() * cim.X_ref[k + 1].R) / cip.dt_mpc;
    }
    // No next orientation after the last step, so hold the previous angular velocity
    cim.X_ref[NH].w = cim.X_ref[NH - 1].w;

    // 3. Reference motor thrusts: tau = J w_dot + w x J w, then map [T; tau] through the mixer -------------------
    for (int k = 0; k < NH; ++k)
    {
        // Moments that produce the reference angular acceleration (finite difference of w_ref)
        const Eigen::Vector3d& w = cim.X_ref[k].w;
        const Eigen::Vector3d tau = cip.J * (cim.X_ref[k + 1].w - w) / cip.dt_mpc + w.cross(cip.J * w);

        // Stack the total thrust and the moments
        Eigen::Matrix<double, 4, 1> wrench;
        wrench << T_ref[k], tau;

        // Motor thrusts, clipped to the motor limits so the reference is always feasible
        cim.U_ref[k] = (mixer_matrix_quadm * wrench).cwiseMax(cip.f_min).cwiseMin(cip.f_max);
    }
}

// Function to propagate the prediction model by one step
mpc_state nmpc::predict_step(const mpc_state& x, const Eigen::Matrix<double, NU, 1>& f) const
{
    const Eigen::Vector3d e3 = e3_basis;
    const double dt = cip.dt_mpc;

    // Map the motor thrusts to the total thrust and the moments
    const Eigen::Matrix<double, 4, 1> wrench = cip.mixer_inv * f;
    const double T = wrench(0);
    const Eigen::Vector3d tau = wrench.tail<3>();

    // Translational acceleration in NED (thrust acts along -b3)
    const Eigen::Vector3d acc = G * e3 + cim.d_hat - (T / MASS) * (x.R * e3);

    // Explicit Euler step (the rotation is propagated with the exponential map so it stays on SO(3))
    mpc_state n;
    n.p = x.p + x.v * dt;                                                   // Position
    n.v = x.v + acc * dt;                                                   // Velocity
    n.R = x.R * expSO3(x.w * dt);                                           // Attitude
    n.w = x.w + cip.J_inv * (tau - x.w.cross(cip.J * x.w)) * dt;            // Euler's rotation equation
    return n;
}

// Function to linearize the prediction model in the error state d = [dp, dv, dtheta, dw] with R = R_bar Exp(dtheta)
void nmpc::linearize(const mpc_state& x, const Eigen::Matrix<double, NU, 1>& f, Mat12& A, Mat12x4& B) const
{
    const Eigen::Vector3d e3 = e3_basis;
    const double dt = cip.dt_mpc;
    const double T = (cip.mixer_inv * f)(0);
    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

    // State Jacobian (starts at identity: every error carries over to the next step, then the couplings are added)
    A.setIdentity();
    A.block<3, 3>(0, 3) = I3 * dt;                                                          // dp     <- dv
    A.block<3, 3>(3, 6) = (T / MASS) * x.R * skewSymmetric(e3) * dt;                        // dv     <- dtheta
    A.block<3, 3>(6, 6) = expSO3(x.w * dt).transpose();                                     // dtheta <- dtheta
    A.block<3, 3>(6, 9) = I3 * dt;                                                          // dtheta <- dw (first order)
    A.block<3, 3>(9, 9) += cip.J_inv * (skewSymmetric(cip.J * x.w) - skewSymmetric(x.w) * cip.J) * dt;  // dw     <- dw

    // Input Jacobian with respect to [T; tau]
    Eigen::Matrix<double, NX, 4> B_wrench = Eigen::Matrix<double, NX, 4>::Zero();
    B_wrench.block<3, 1>(3, 0) = -(x.R * e3) / MASS * dt;                                   // dv     <- T
    B_wrench.block<3, 3>(9, 1) = cip.J_inv * dt;                                            // dw     <- tau

    // Chain rule through the mixer to get the Jacobian with respect to the motor thrusts
    B = B_wrench * cip.mixer_inv;
}

// Function to compute the stage cost at step k
double nmpc::stage_cost(const int k, const mpc_state& x, const Eigen::Matrix<double, NU, 1>& f) const
{
    const Vec12 e = box_minus(x, cim.X_ref[k]);
    const Eigen::Matrix<double, NU, 1> df = f - cim.U_ref[k];

    // 0.5 e'Qe + 0.5 df'R df
    return 0.5 * e.dot(cip.Q * e) + 0.5 * df.dot(cip.R_in * df);
}

// Function to compute the terminal cost
double nmpc::terminal_cost(const mpc_state& x) const
{
    const Vec12 e = box_minus(x, cim.X_ref[NH]);
    return 0.5 * e.dot(cip.Qf * e);
}

// Function to simulate the warm start inputs from the measured state
//   - Fills cim.X (the trajectory the backward pass linearizes about) and returns its cost.
double nmpc::rollout()
{
    double c = 0.0;
    cim.X[0] = cim.x0;      // Every plan starts at the measured state
    for (int k = 0; k < NH; ++k)
    {
        c += stage_cost(k, cim.X[k], cim.U[k]);
        cim.X[k + 1] = predict_step(cim.X[k], cim.U[k]);
    }
    return c + terminal_cost(cim.X[NH]);
}

// Function to compute the feedback and feed-forward gains by walking backward from the end of the horizon
//   - The gains satisfy: optimal input change df = k_ff + K_fb dx, for a state deviation dx from cim.X.
void nmpc::backward_pass()
{
    // Start from the terminal cost
    Vec12 Vx  = cip.Qf * box_minus(cim.X[NH], cim.X_ref[NH]);   // Gradient of the cost-to-go
    Mat12 Vxx = cip.Qf;                                         // Hessian of the cost-to-go
    Mat12 A;
    Mat12x4 B;

    for (int k = NH - 1; k >= 0; --k)
    {
        // Linearize about the current plan
        linearize(cim.X[k], cim.U[k], A, B);

        // Gradients of the stage cost
        const Vec12 lx = cip.Q * box_minus(cim.X[k], cim.X_ref[k]);
        const Eigen::Matrix<double, NU, 1> lu = cip.R_in * (cim.U[k] - cim.U_ref[k]);

        // Expansion of the Q function (Gauss-Newton: no second order dynamics terms)
        const Vec12 Qx = lx + A.transpose() * Vx;
        const Eigen::Matrix<double, NU, 1> Qu = lu + B.transpose() * Vx;
        const Mat12 Qxx = cip.Q + A.transpose() * Vxx * A;
        const Eigen::Matrix<double, NU, NU> Quu = cip.R_in + B.transpose() * Vxx * B
                                                + cip.regularization * Eigen::Matrix<double, NU, NU>::Identity();
        const Mat4x12 Qux = B.transpose() * Vxx * A;

        // Solve for the gains (4x4 solve): k_ff = -Quu^-1 Qu, K_fb = -Quu^-1 Qux
        const auto ldlt = Quu.ldlt();
        cim.k_ff[k] = -ldlt.solve(Qu);
        cim.K_fb[k] = -ldlt.solve(Qux);

        // Propagate the cost-to-go and keep the Hessian symmetric
        // (the usual K'Quu k + K'Qu and K'Quu K + K'Qux terms cancel for these gains, leaving the short form)
        Vx  = Qx  + Qux.transpose() * cim.k_ff[k];
        Vxx = Qxx + Qux.transpose() * cim.K_fb[k];
        Vxx = 0.5 * (Vxx + Vxx.transpose());
    }
}

// Function to simulate the nonlinear model with the new gains
//   - alpha scales only the feed-forward step; the feedback keeps the candidate close to the linearized plan.
//   - Writes the candidate into X_new/U_new; solve_ocp() keeps it only if the cost went down.
double nmpc::forward_pass(const double alpha)
{
    double c = 0.0;
    cim.X_new[0] = cim.x0;
    for (int k = 0; k < NH; ++k)
    {
        // Deviation from the current plan and the corrected input
        const Vec12 dx = box_minus(cim.X_new[k], cim.X[k]);
        Eigen::Matrix<double, NU, 1> f = cim.U[k] + alpha * cim.k_ff[k] + cim.K_fb[k] * dx;

        // Respect the motor thrust limits
        f = f.cwiseMax(cip.f_min).cwiseMin(cip.f_max);

        // Store the input, add its cost and step the nonlinear model
        cim.U_new[k] = f;
        c += stage_cost(k, cim.X_new[k], f);
        cim.X_new[k + 1] = predict_step(cim.X_new[k], f);
    }
    return c + terminal_cost(cim.X_new[NH]);
}

// Function to solve the optimal control problem
void nmpc::solve_ocp()
{
    // Rollout the warm start from the measured state and reset the per tick statistics
    cim.cost_initial = rollout();
    cim.cost = cim.cost_initial;
    cim.iterations = 0;
    cim.alpha = 0.0;

    // Few iterations per tick are enough because the warm start is already close to optimal
    for (int it = 0; it < cip.max_iterations; ++it)
    {
        // Gains about the current plan
        backward_pass();

        // Line search: halve the step until the cost decreases
        bool improved = false;
        double alpha = 1.0;
        for (int ls = 0; ls < cip.line_search_steps; ++ls, alpha *= 0.5)
        {
            const double cost_new = forward_pass(alpha);
            if (cost_new < cim.cost)
            {
                // Accept the candidate as the new plan
                cim.X = cim.X_new;
                cim.U = cim.U_new;
                cim.cost = cost_new;
                cim.alpha = alpha;
                improved = true;
                break;
            }
        }

        // Nothing better was found so stop early
        if (!improved) break;
        ++cim.iterations;
    }
}

// Function to shift the warm start
void nmpc::shift_warm_start(const double time_step)
{
    // The controller runs faster than dt_mpc, so the plan is shifted by the fraction of a step that has elapsed
    // by blending each input with the next one. The last input is repeated.
    // NOTE: Only U is shifted; X is rebuilt from the measured state by the next rollout().
    const double s = std::min(time_step / cip.dt_mpc, 1.0);
    for (int k = 0; k < NH - 1; ++k)
    {
        cim.U[k] = (1.0 - s) * cim.U[k] + s * cim.U[k + 1];
    }
}

// Function to compute the normalized thrusts
void nmpc::compute_normalized_thrusts()
{
    // Saturate each element of the Thrust vector between MIN_THRUST and MAX_THRUST
    cim.Sat_Thrust = (cim.Thrust.cwiseMin(MAX_THRUST).cwiseMax(MIN_THRUST));

    // Compute the final control inputs (thrust [N] -> normalized motor command [0-1] via the thrust curve)
    control_input(0) = evaluatePolynomial(thrust_polynomial_coeff_quadm, cim.Sat_Thrust(0));
    control_input(1) = evaluatePolynomial(thrust_polynomial_coeff_quadm, cim.Sat_Thrust(1));
    control_input(2) = evaluatePolynomial(thrust_polynomial_coeff_quadm, cim.Sat_Thrust(2));
    control_input(3) = evaluatePolynomial(thrust_polynomial_coeff_quadm, cim.Sat_Thrust(3));
}

// Function that is called in control.cpp
void nmpc::run(const double time_step_rk4_) {

    // Process the dynamics --------------------------------------------------------
    // 1. Compute the disturbance estimate from the integral of the position error
    compute_disturbance_estimate();

    // 2. Sample the trajectory over the horizon and build the reference
    compute_reference_horizon();

    // 3. Solve the optimal control problem
    solve_ocp();

    // 4. Apply the first motor thrusts of the plan and compute [thrust; mx; my; mz]
    cim.Thrust = cim.U[0];
    cim.u = cip.mixer_inv * cim.Thrust;

    // 5. Compute the normalized thrust - Final Step
    compute_normalized_thrusts();

    // 6. Shift the warm start for the next control tick
    shift_warm_start(time_step_rk4_);

    // 7. Do the integration (advances the position error integral used by the next tick's d_hat)
    rk4.do_step(boost::bind(&nmpc::model, this, bph::_1, bph::_2, bph::_3),
                y, cim.t, time_step_rk4_);

    // Capture the time after the execution of the controller
    cim.alg_end_time = std::chrono::high_resolution_clock::now();

    // Calculate the duration of the execution
    cim.alg_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                            cim.alg_end_time - cim.alg_start_time).count();

    // 8. Log the Data after all the calculataions
    logger.logLogData();

}

// Implementing getter functions from controller_base
float nmpc::get_t1() const {
    return control_input(0);
}

float nmpc::get_t2() const {
    return control_input(1);
}

float nmpc::get_t3() const {
    return control_input(2);
}

float nmpc::get_t4() const {
    return control_input(3);
}

float nmpc::get_t5() const {
    return control_input(4);
}

float nmpc::get_t6() const {
    return control_input(5);
}

float nmpc::get_t7() const {
    return control_input(6);
}

float nmpc::get_t8() const {
    return control_input(7);
}

} // namespace _nmpc_
} // namespace _quadm_
