/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <memory>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "utils/dataset_reader.h"
#include "intnavlib.h"

#if ROS_AVAILABLE == 1
#include "ros/ROS1Visualizer.h"
#include <ros/ros.h>
#elif ROS_AVAILABLE == 2
#include "ros/ROS2Visualizer.h"
#include <rclcpp/rclcpp.hpp>
#endif

using namespace ov_msckf;
using namespace intnavlib;

std::shared_ptr<VioManager> sys;
#if ROS_AVAILABLE == 1
std::shared_ptr<ROS1Visualizer> viz;
#elif ROS_AVAILABLE == 2
std::shared_ptr<ROS2Visualizer> viz;
#endif

// Main function
int main(int argc, char **argv) {

  // Ensure we have a path, if the user passes it then we should use it
  std::string config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }

#if ROS_AVAILABLE == 1
  // Launch our ros node
  ros::init(argc, argv, "run_subscribe_msckf");
  auto nh = std::make_shared<ros::NodeHandle>("~");
  nh->param<std::string>("config_path", config_path, config_path);
#elif ROS_AVAILABLE == 2
  // Launch our ros node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("run_subscribe_msckf", options);
  node->get_parameter<std::string>("config_path", config_path);
#endif

  // Load the config
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
#if ROS_AVAILABLE == 1
  parser->set_node_handler(nh);
#elif ROS_AVAILABLE == 2
  parser->set_node(node);
#endif

  // Verbosity
  std::string verbosity = "DEBUG";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create our VIO system
  VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = true;
  sys = std::make_shared<VioManager>(params);

  // =============== Init estimator manually, reading from config file ==============

  double init_start_time;

  std::vector<double> init_lla;
  std::vector<double> init_v_eb_n;
  std::vector<double> init_rpy_n_b;

  double init_att_unc;
  double init_vel_unc;
  double init_pos_unc;
  double init_b_a_unc;
  double init_b_g_unc;

  parser->parse_config("init_start_time", init_start_time);

  parser->parse_config("init_lla", init_lla);
  parser->parse_config("init_v_eb_n", init_v_eb_n);
  parser->parse_config("init_rpy_n_b", init_rpy_n_b);

  // standard deviations
  parser->parse_config("init_att_unc", init_att_unc);
  parser->parse_config("init_vel_unc", init_vel_unc);
  parser->parse_config("init_pos_unc", init_pos_unc);
  parser->parse_config("init_b_a_unc", init_b_a_unc);
  parser->parse_config("init_b_g_unc", init_b_g_unc);

  NavSolutionNed est_nav_ned = NavSolutionNed{0.0,
                                deg_to_rad * init_lla[0], 
                                deg_to_rad * init_lla[1], 
                                init_lla[2], 
                                Eigen::Vector3d(init_v_eb_n[0], init_v_eb_n[1], init_v_eb_n[2]), 
                                rpyToR(deg_to_rad * Eigen::Vector3d(init_rpy_n_b[0], init_rpy_n_b[1], init_rpy_n_b[2])).transpose()};

  NavSolutionEcef est_nav_ecef = nedToEcef(est_nav_ned);

  // Openvins needs global_to_imu (not vice versa)
  Eigen::Quaterniond q_GtoI(est_nav_ecef.C_b_e);

  // [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]

  Eigen::Matrix<double, 17, 1> init_imustate;

  init_imustate(0,0) = init_start_time; // t0
  init_imustate.block<4,1>(1,0) = q_GtoI.coeffs(); // q_GtoI
  init_imustate.block<3,1>(5,0) =  est_nav_ecef.r_eb_e; // p_IinG
  init_imustate.block<3,1>(8,0) = est_nav_ecef.v_eb_e; // v_IinG
  init_imustate.block<3,1>(11,0) = Eigen::Vector3d::Zero(); //  b_gyro
  init_imustate.block<3,1>(14,0) = Eigen::Vector3d::Zero(); // b_accel

  sys->initialize_with_prior(init_imustate,
                              init_att_unc,
                              init_vel_unc,
                              init_pos_unc,
                              init_b_a_unc,
                              init_b_g_unc);

  // ===============================================

#if ROS_AVAILABLE == 1
  viz = std::make_shared<ROS1Visualizer>(nh, sys);
  viz->setup_subscribers(parser);
#elif ROS_AVAILABLE == 2
  viz = std::make_shared<ROS2Visualizer>(node, sys);
  viz->setup_subscribers(parser);
#endif

  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT_ERROR(RED "unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Spin off to ROS
  PRINT_DEBUG("done...spinning to ros\n");
#if ROS_AVAILABLE == 1
  // ros::spin();
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
#elif ROS_AVAILABLE == 2
  // rclcpp::spin(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
#endif

  // Final visualization
  viz->visualize_final();
#if ROS_AVAILABLE == 1
  ros::shutdown();
#elif ROS_AVAILABLE == 2
  rclcpp::shutdown();
#endif

  // Done!
  return EXIT_SUCCESS;
}
