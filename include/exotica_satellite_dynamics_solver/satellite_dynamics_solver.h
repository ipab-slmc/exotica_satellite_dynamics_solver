//
// Copyright (c) 2019-2020, The University of Edinburgh, University of Oxford
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef EXOTICA_SATELLITE_DYNAMICS_SOLVER_SATELLITE_DYNAMICS_SOLVER_H_
#define EXOTICA_SATELLITE_DYNAMICS_SOLVER_SATELLITE_DYNAMICS_SOLVER_H_

/// TODO: remove this pragma once Pinocchio removes neutralConfiguration/
/// and fixes their deprecation warnings. (Relates to #547)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

/// fwd.hpp needs to be included first (before Boost, which comes with ROS),
/// else everything breaks for Pinocchio >=2.1.5
#include <pinocchio/fwd.hpp>

#include <exotica_core/dynamics_solver.h>
#include <exotica_core/scene.h>

#include <exotica_satellite_dynamics_solver/satellite_dynamics_solver_initializer.h>

#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#pragma GCC diagnostic pop

namespace exotica
{
class SatelliteDynamicsSolver : public DynamicsSolver, public Instantiable<SatelliteDynamicsSolverInitializer>
{
    typedef Eigen::Matrix<double, 6, 1> Vector6;

public:
    void AssignScene(ScenePtr scene_in) override;

    StateVector f(const StateVector& x, const ControlVector& u) override;
    void ComputeDerivatives(const StateVector& x, const ControlVector& u) override;
    StateDerivative fx(const StateVector& x, const ControlVector& u) override;
    ControlDerivative fu(const StateVector& x, const ControlVector& u) override;

    Eigen::VectorXd GetPosition(Eigen::VectorXdRefConst x_in) override;
    StateVector StateDelta(const StateVector& x_1, const StateVector& x_2) override;
    Eigen::MatrixXd dStateDelta(const StateVector& x_1, const StateVector& x_2, const ArgumentPosition first_or_second) override;
    Hessian ddStateDelta(const StateVector& x_1, const StateVector& x_2, const ArgumentPosition first_or_second) override;
    void Integrate(const StateVector& x, const StateVector& dx, const double dt, StateVector& xout) override;

private:
    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> pinocchio_data_;

    Eigen::VectorXd xdot_analytic_;
    Eigen::VectorXd tau_;  // Control vector
    Eigen::MatrixXd Minv_;

    // Thrusters
    int num_thrusters_ = 0;
    int num_aux_joints_ = 0;
    std::vector<pinocchio::FrameIndex> thruster_frame_ids_;
    std::vector<Vector6> thruster_force_directions_;
    pinocchio::container::aligned_vector<pinocchio::SE3::ActionMatrixType> thruster_action_matrices_;
    pinocchio::container::aligned_vector<pinocchio::Force> fext_;
    Eigen::MatrixXd daba_dfext_;
    Eigen::MatrixXd dtau_du_;

    void UpdateExternalForceInputFromThrusters(const ControlVector& u);
};
}  // namespace exotica

#endif  // EXOTICA_SATELLITE_DYNAMICS_SOLVER_SATELLITE_DYNAMICS_SOLVER_H_
