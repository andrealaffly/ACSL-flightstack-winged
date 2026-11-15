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
 * File:        NMPC_TD_logger.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        November 15, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: logging for the NMPC controller. Inherits the Blackbox class
 *              for logging and takes in the internal members for the NMPC.
 *              Column order matches the simulator's nmpc-quadm.cpp (MATLAB reads by index).
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

#include "NMPC_TD_logger.hpp"

namespace _quadm_{
namespace _nmpc_{

nmpc_logger::nmpc_logger(controller_internal_members* cim,
                         controller_integrated_state_members* csm,
                         Eigen::Vector<float, 8>* control_input,
                         const std::string & controller_log_dir_) :
                         CIM(cim), CSM(csm), control_input_ptr(control_input),
                         flight_run_log_directory(controller_log_dir_)
{
    // Initilize the logging from blackbox class.
    if(logInitLogging()) {FLIGHTSTACK_INFO("quadm NMPC Logger Created");};
}

// Implementing virtual functions from Blackbox
//   - The column order must match logLogData() and the simulator's nmpc-quadm.cpp. Columns are read by index
//     (MATLAB), so add new ones at the end only.
void nmpc_logger::logInitHeaders() {
    std::ostringstream oss;

    oss << ", "
        << "Controller Time [s], "
        << "Algorithm Execution Time [us], "
        << "Position x [m], "
        << "Position y [m], "
        << "Position z [m], "
        << "Velocity x [m/s], "
        << "Velocity y [m/s], "
        << "Velocity z [m/s], "
        << "User Position x [m], "
        << "User Position y [m], "
        << "User Position z [m], "
        << "User Velocity x [m/s], "
        << "User Velocity y [m/s], "
        << "User Velocity z [m/s], "
        << "User Acceleration x [m/s^2], "
        << "User Acceleration y [m/s^2], "
        << "User Acceleration z [m/s^2], "
        << "User Yaw [rad], "
        << "Error in x [m], "
        << "Error in y [m], "
        << "Error in z [m], "
        << "Error in vx [m/s], "
        << "Error in vy [m/s], "
        << "Error in vz [m/s], "
        << "Roll [rad], "
        << "Pitch [rad], "
        << "Yaw [rad], "
        << "Angular Velocity x [rad/s], "
        << "Angular Velocity y [rad/s], "
        << "Angular Velocity z [rad/s], "
        << "Reference Angular Velocity x [rad/s], "
        << "Reference Angular Velocity y [rad/s], "
        << "Reference Angular Velocity z [rad/s], "
        << "Integral Error x [m s], "
        << "Integral Error y [m s], "
        << "Integral Error z [m s], "
        << "Disturbance Estimate x [m/s^2], "
        << "Disturbance Estimate y [m/s^2], "
        << "Disturbance Estimate z [m/s^2], "
        << "Initial Cost [-], "
        << "Final Cost [-], "
        << "Iterations [-], "
        << "Line Search Step [-], "
        << "U1 Thrust [N], "
        << "U2 Moment x [N m], "
        << "U3 Moment y [N m], "
        << "U4 Moment z [N m], "
        << "Thrust 1 [N], "
        << "Thrust 2 [N], "
        << "Thrust 3 [N], "
        << "Thrust 4 [N], "
        << "Saturated Thrust 1 [N], "
        << "Saturated Thrust 2 [N], "
        << "Saturated Thrust 3 [N], "
        << "Saturated Thrust 4 [N], "
        << "Normalized Thrust 1 [-], "
        << "Normalized Thrust 2 [-], "
        << "Normalized Thrust 3 [-], "
        << "Normalized Thrust 4 [-], ";
        generateMatrixHeaders(oss, "R_ji", CIM->Rji, "[-]");
        generateMatrixHeaders(oss, "R_ref", CIM->X_ref[0].R, "[-]");

    BOOST_LOG(logger_logdata) << oss.str();
}

// Implementing virtual functions from Blackbox
bool nmpc_logger::logInitLogging() {
    try {
        // Create the directory path for the controller specific flight run logs
        std::string flight_run_controller_specific_directory = flight_run_log_directory + "/" + "NMPC";

        // Check if the directory exits, and create it if it doesn't
        if (!std::filesystem::exists(flight_run_controller_specific_directory))
        {
            std::filesystem::create_directories(flight_run_controller_specific_directory);
        }

        // Generate the log file name with a timestamp
        std::stringstream log_ss;
        log_ss << flight_run_controller_specific_directory << "/controller_log" << ".log";
        std::string log_filename = log_ss.str();

        // Add the "Tag" attribute with a constant value of "LogDataTag" to the logger
        logger_logdata.add_attribute("Tag", attrs::constant<std::string>("LogDataTag"));

        // Define a synchronous sink with a text ostream backend
        typedef sinks::synchronous_sink<sinks::text_ostream_backend> text_sink;
        boost::shared_ptr<text_sink> logdata_sink = boost::make_shared<text_sink>();

        // Add a stream to the backend (in this case, a file stream)
        logdata_sink->locked_backend()->add_stream(boost::make_shared<std::ofstream>(log_filename));

        // Set the formatter for the sink
        logdata_sink->set_formatter(
            expr::stream
            << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f") << "] " // Format date and time
            << "[" << expr::attr<boost::log::attributes::current_thread_id::value_type>("ThreadID") << "] " // Current thread ID
            << "[" << expr::attr<std::string>("Tag") << "] " // Tag attribute value
            << "[" << expr::attr<boost::log::attributes::current_process_id::value_type>("ProcessID") << "] " // Current process ID
            << "[" << expr::attr<unsigned int>("LineID") << "] " // Line ID
            << expr::smessage // Log message
        );

        // Add the sink to the logging core
        logging::core::get()->add_sink(logdata_sink);

        // Set a filter for the sink
        logdata_sink->set_filter(expr::has_attr("Tag") && expr::attr<std::string>("Tag") == "LogDataTag");

        logging::add_common_attributes(); // Add attributes like timestamp

        // Initilaize the logging headers
        logInitHeaders();

        // Create the directory path for the controller specific flight run for parameters and gains used
        std::string flight_run_controller_specific_parameter_directory = flight_run_controller_specific_directory + "/" + "params";

        // Check if the direcotry exits, and create it if it doesn't
        if (!std::filesystem::exists(flight_run_controller_specific_parameter_directory))
        {
            std::filesystem::create_directories(flight_run_controller_specific_parameter_directory);
        }

        // Copy the flight parameters over
        std::string params_source_file = "./src/acsl_flight/params/flight_params/flight_main_params.json";
        std::stringstream params_target_ss;
        params_target_ss << flight_run_controller_specific_parameter_directory << "/flight_main_params.json" ;
        std::string params_target_file = params_target_ss.str();

        if (!std::filesystem::exists(params_source_file)) {
            FLIGHTSTACK_ERROR("MAIN PARAMS FILE NOT PRESENT");
        }

        if (std::filesystem::exists(params_source_file)) {
            std::filesystem::copy(params_source_file, params_target_file);
        }

        // Copy the gains parameters over
        std::string gains_source_file = "./src/acsl_flight/params/control_algorithms/quadm/NMPC/gains_NMPC.json";
        std::stringstream gains_target_ss;
        gains_target_ss << flight_run_controller_specific_parameter_directory << "/gains_NMPC.json" ;
        std::string gains_target_file = gains_target_ss.str();

        if (!std::filesystem::exists(gains_source_file)) {
            FLIGHTSTACK_ERROR("GAINS FILE NOT PRESENT");
        }

        if (std::filesystem::exists(gains_source_file)) {
            std::filesystem::copy(gains_source_file, gains_target_file);
        }

        // Return true if all successful
        return true;       

    } catch (const std::filesystem::filesystem_error& e) {
        FLIGHTSTACK_ERROR("Filesystem error:", e.what());
    } catch (const std::exception& e) {
        FLIGHTSTACK_ERROR("Exeption:", e.what());
    }

    return false; // IF there are errors
}

// Implementing virtual functions from Blackbox
//   - Same order as the headers in logInitHeaders().
void nmpc_logger::logLogData() {
    // Log the data
    std::ostringstream oss;

    oss << ", "
        << CIM->t << ", "
        << CIM->alg_duration << ", "
        << CIM->x_tran_pos(0) << ", "
        << CIM->x_tran_pos(1) << ", "
        << CIM->x_tran_pos(2) << ", "
        << CIM->x_tran_vel(0) << ", "
        << CIM->x_tran_vel(1) << ", "
        << CIM->x_tran_vel(2) << ", "
        << CIM->r_user(0) << ", "
        << CIM->r_user(1) << ", "
        << CIM->r_user(2) << ", "
        << CIM->r_dot_user(0) << ", "
        << CIM->r_dot_user(1) << ", "
        << CIM->r_dot_user(2) << ", "
        << CIM->r_ddot_user(0) << ", "
        << CIM->r_ddot_user(1) << ", "
        << CIM->r_ddot_user(2) << ", "
        << CIM->psi_user << ", "
        << CIM->e_tran_pos(0) << ", "
        << CIM->e_tran_pos(1) << ", "
        << CIM->e_tran_pos(2) << ", "
        << CIM->e_tran_vel(0) << ", "
        << CIM->e_tran_vel(1) << ", "
        << CIM->e_tran_vel(2) << ", "
        << CIM->eta_rot(0) << ", "
        << CIM->eta_rot(1) << ", "
        << CIM->eta_rot(2) << ", "
        << CIM->omega(0) << ", "
        << CIM->omega(1) << ", "
        << CIM->omega(2) << ", "
        << CIM->X_ref[0].w(0) << ", "
        << CIM->X_ref[0].w(1) << ", "
        << CIM->X_ref[0].w(2) << ", "
        << CSM->e_tran_pos_I(0) << ", "
        << CSM->e_tran_pos_I(1) << ", "
        << CSM->e_tran_pos_I(2) << ", "
        << CIM->d_hat(0) << ", "
        << CIM->d_hat(1) << ", "
        << CIM->d_hat(2) << ", "
        << CIM->cost_initial << ", "
        << CIM->cost << ", "
        << CIM->iterations << ", "
        << CIM->alpha << ", "
        << CIM->u(0) << ", "
        << CIM->u(1) << ", "
        << CIM->u(2) << ", "
        << CIM->u(3) << ", "
        << CIM->Thrust(0) << ", "
        << CIM->Thrust(1) << ", "
        << CIM->Thrust(2) << ", "
        << CIM->Thrust(3) << ", "
        << CIM->Sat_Thrust(0) << ", "
        << CIM->Sat_Thrust(1) << ", "
        << CIM->Sat_Thrust(2) << ", "
        << CIM->Sat_Thrust(3) << ", "
        << (*control_input_ptr)(0) << ", "
        << (*control_input_ptr)(1) << ", "
        << (*control_input_ptr)(2) << ", "
        << (*control_input_ptr)(3) << ", ";
        appendEigenData(oss, CIM->Rji);
        appendEigenData(oss, CIM->X_ref[0].R);

    BOOST_LOG(logger_logdata) << oss.str();
}

} // namespace _nmpc_
} // namespace _quadm_
