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

#include "UpdaterGNSS.h"

#include "UpdaterHelper.h"

#include "feat/Feature.h"
#include "feat/FeatureInitializer.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/LandmarkRepresentation.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include "intnavlib.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

using namespace intnavlib;

UpdaterGNSS::UpdaterGNSS() {

  // Initialize the chi squared test table with confidence level 0.95
  // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
  for (int i = 1; i < 500; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }

}

void UpdaterGNSS::update(std::shared_ptr<State> state, const ov_core::GNSSData &message) {

  // Start timing
  boost::posix_time::ptime rT0, rT1;
  rT0 = boost::posix_time::microsec_clock::local_time();

  // Convert LLA to position in ecef
  Eigen::Vector3d r_eb_e_gnss = nedToEcef(NavSolutionNed{0,message.latitude, message.longitude, message.altitude, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity()}).r_eb_e;

  // TODO do X square test!
  
  // Residual 
  Eigen::VectorXd res_gnss =  r_eb_e_gnss - state->_imu->pos();

  // Measurement covariance
  Eigen::MatrixXd R_gnss = std::pow(message.pos_std, 2) * Eigen::MatrixXd::Identity(res_gnss.rows(), res_gnss.rows());
  
  // Residual order (dumb here, just pos)
  
  std::vector<std::shared_ptr<Type>> Hx_order_gnss;
  Hx_order_gnss.clear();
  Hx_order_gnss.push_back(state->_imu->p());
  
  // Observation jacobian
  Eigen::MatrixXd Hx_gnss = Eigen::Matrix3d::Identity();
  
  // Perform loosely coupled ECEF position update
  StateHelper::EKFUpdate(state, Hx_order_gnss, Hx_gnss, res_gnss, R_gnss);

  // Take time
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Debug print timing information
  PRINT_ALL("[GNSS-UP]: %.4f seconds total\n", (rT1 - rT0).total_microseconds() * 1e-6);
}
