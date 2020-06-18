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

#include <exotica_satellite_dynamics_solver/force_input_initializer.h>
#include <exotica_satellite_dynamics_solver/satellite_dynamics_solver.h>
#include <pinocchio/algorithm/joint-configuration.hpp>

REGISTER_DYNAMICS_SOLVER_TYPE("SatelliteDynamicsSolver", exotica::SatelliteDynamicsSolver)

namespace exotica
{
void SatelliteDynamicsSolver::AssignScene(ScenePtr scene_in)
{
    if (scene_in->GetKinematicTree().GetControlledBaseType() != BaseType::FLOATING)
    {
        ThrowPretty("Kinematic scene does not have a floating base.");
    }

    constexpr bool verbose = false;
    pinocchio::urdf::buildModel(scene_in->GetKinematicTree().GetRobotModel()->getURDF(), pinocchio::JointModelFreeFlyer(), model_, verbose);

    // Thrusters are specified in the initialiser:
    num_thrusters_ = parameters_.Thrusters.size();

    num_positions_ = model_.nq;
    num_velocities_ = model_.nv;
    // We manually specify the number of thrusters (10) + active arm joints. We exclude the unactuated floating base (6).
    num_aux_joints_ = model_.nv - 6;
    num_controls_ = num_thrusters_ + num_aux_joints_;
    HIGHLIGHT("nv=" << model_.nv << ", num_controls_=" << num_controls_ << ", num_aux_joints_=" << num_aux_joints_);

    // We are in space!
    model_.gravity.setZero();

    // Create Pinocchio data
    pinocchio_data_.reset(new pinocchio::Data(model_));

    // Get frame Ids and force directions for each thruster
    thruster_frame_ids_.reserve(num_thrusters_);
    thruster_force_directions_.reserve(num_thrusters_);
    for (int i = 0; i < num_thrusters_; ++i)
    {
        ForceInputInitializer force_init(parameters_.Thrusters[i]);
        auto frame_id = model_.getFrameId(force_init.LinkName);
        if (frame_id > model_.nframes) ThrowPretty("Frame '" << force_init.LinkName << "' does not exist.");
        thruster_frame_ids_.emplace_back(frame_id);
        Vector6 force_direction = Vector6::Zero();
        force_direction.head<3>() = force_init.ForceDirection;
        thruster_force_directions_.emplace_back(force_direction);
    }

    // Pre-allocate data for f, fx, fu
    const int ndx = get_num_state_derivative();
    xdot_analytic_.setZero(ndx);
    fx_.setZero(ndx, ndx);
    fx_.topRightCorner(num_velocities_, num_velocities_).setIdentity();
    fu_.setZero(ndx, num_controls_);
    tau_.setZero(model_.nv);
    Minv_.setZero(model_.nv, model_.nv);

    // Instantiate vector of external forces
    fext_.assign(model_.njoints, pinocchio::Force::Zero());
    daba_dfext_.setZero(model_.nv, num_controls_);

    // Set up derivative of external forces
    thruster_action_matrices_.resize(num_thrusters_);
    for (int i = 0; i < num_thrusters_; ++i)
    {
        thruster_action_matrices_[i] = model_.frames[thruster_frame_ids_[i]].placement.toDualActionMatrix();
        daba_dfext_.block(0, i, 6, 1).noalias() = thruster_action_matrices_[i] * thruster_force_directions_[i];
    }

    // Set up derivative of actuated joints
    dtau_du_.setZero(model_.nv, num_controls_);
    dtau_du_.block(6, num_thrusters_, num_aux_joints_, num_aux_joints_).setIdentity();
}

void SatelliteDynamicsSolver::UpdateExternalForceInputFromThrusters(const ControlVector& u)
{
    // Non-actuated joints
    fext_[1].setZero();
    for (int i = 0; i < num_thrusters_; ++i)
    {
        const pinocchio::SE3& thruster_frame_placement = model_.frames[thruster_frame_ids_[i]].placement;
        fext_[1] += thruster_frame_placement.act(pinocchio::Force(thruster_force_directions_[i] * u(i)));
    }
}

Eigen::VectorXd SatelliteDynamicsSolver::f(const StateVector& x, const ControlVector& u)
{
    // The control vector u is comprised of: thrusters, arm/auxiliary joints
    UpdateExternalForceInputFromThrusters(u.head(num_thrusters_));
    tau_.tail(num_aux_joints_) = u.segment(num_thrusters_, num_aux_joints_);

    pinocchio::aba(model_, *pinocchio_data_, x.head(num_positions_), x.tail(num_velocities_), tau_, fext_);
    xdot_analytic_.head(num_velocities_) = x.tail(num_velocities_);
    xdot_analytic_.tail(num_velocities_) = pinocchio_data_->ddq;

    return xdot_analytic_;
}

Eigen::VectorXd SatelliteDynamicsSolver::GetPosition(Eigen::VectorXdRefConst x_in)
{
    // Convert quaternion to Euler angles.
    Eigen::VectorXd xyz_rpy(num_positions_ - 1);
    xyz_rpy.head<3>() = x_in.head<3>();
    xyz_rpy.segment<3>(3) = Eigen::Quaterniond(x_in.segment<4>(3)).toRotationMatrix().eulerAngles(0, 1, 2);
    xyz_rpy.segment(6, num_aux_joints_) = x_in.segment(7, num_aux_joints_);
    return xyz_rpy;
}

Eigen::VectorXd SatelliteDynamicsSolver::StateDelta(const StateVector& x_1, const StateVector& x_2)
{
    if (x_1.size() != num_positions_ + num_velocities_ || x_2.size() != num_positions_ + num_velocities_)
    {
        ThrowPretty("x_1 or x_2 do not have correct size, x1=" << x_1.size() << " x2=" << x_2.size() << " expected " << num_positions_ + num_velocities_);
    }

    Eigen::VectorXd dx(2 * num_velocities_);
    pinocchio::difference(model_, x_2.head(num_positions_), x_1.head(num_positions_), dx.head(num_velocities_));
    dx.tail(num_velocities_) = x_1.tail(num_velocities_) - x_2.tail(num_velocities_);
    return dx;
}

Eigen::MatrixXd SatelliteDynamicsSolver::dStateDelta(const StateVector& x_1, const StateVector& x_2, const ArgumentPosition first_or_second)
{
    if (x_1.size() != num_positions_ + num_velocities_ || x_2.size() != num_positions_ + num_velocities_)
    {
        ThrowPretty("x_1 or x_2 do not have correct size, x1=" << x_1.size() << " x2=" << x_2.size() << " expected " << num_positions_ + num_velocities_);
    }

    if (first_or_second != ArgumentPosition::ARG0 && first_or_second != ArgumentPosition::ARG1)
    {
        ThrowPretty("Can only take derivative w.r.t. x_1 or x_2, i.e., ARG0 or ARG1. Provided: " << first_or_second);
    }

    Eigen::MatrixXd J = Eigen::MatrixXd::Identity(2 * num_velocities_, 2 * num_velocities_);

    if (first_or_second == ArgumentPosition::ARG0)
    {
        pinocchio::dDifference(model_, x_2.head(num_positions_), x_1.head(num_positions_), J.topLeftCorner(num_velocities_, num_velocities_), pinocchio::ArgumentPosition::ARG1);
    }
    else
    {
        pinocchio::dDifference(model_, x_2.head(num_positions_), x_1.head(num_positions_), J.topLeftCorner(num_velocities_, num_velocities_), pinocchio::ArgumentPosition::ARG0);
        J.bottomRightCorner(num_velocities_, num_velocities_) *= -1.0;
    }

    return J;
}

Hessian SatelliteDynamicsSolver::ddStateDelta(const StateVector& x_1, const StateVector& x_2, const ArgumentPosition first_or_second)
{
    if (x_1.size() != num_positions_ + num_velocities_ || x_2.size() != num_positions_ + num_velocities_)
    {
        ThrowPretty("x_1 or x_2 do not have correct size, x1=" << x_1.size() << " x2=" << x_2.size() << " expected " << num_positions_ + num_velocities_);
    }

    if (first_or_second != ArgumentPosition::ARG0 && first_or_second != ArgumentPosition::ARG1)
    {
        ThrowPretty("Can only take derivative w.r.t. x_1 or x_2, i.e., ARG0 or ARG1. Provided: " << first_or_second);
    }

    if (first_or_second == ArgumentPosition::ARG1) ThrowPretty("Not yet implemented.");

    // In Euclidean spaces, this is zero.
    Hessian ddStateDelta;
    ddStateDelta.setConstant(get_num_state_derivative(), Eigen::MatrixXd::Zero(get_num_state_derivative(), get_num_state_derivative()));

    // Use finite differences for the first 6 components (Lie group)
    constexpr double eps = 1e-8;
    Eigen::VectorXd dq_numdiff(num_velocities_);
    Eigen::VectorXd x0_plus(get_num_state()), x0_minus(get_num_state()), x_1_normalized(get_num_state());
    Eigen::MatrixXd dJdiff_dx0_plus(get_num_state_derivative(), get_num_state_derivative()), dJdiff_dx0_minus(get_num_state_derivative(), get_num_state_derivative());

    // Normalize input quaternion
    x_1_normalized = x_1;
    NormalizeQuaternionInConfigurationVector(x_1_normalized);

    // NB: We only differentiate the first 6 dimensions for the free-flyer joint.
    // The other dimensions evaluate to 0.
    for (int j = 0; j < 6; ++j)
    {
        dq_numdiff.setZero();
        dq_numdiff(j) = eps / 2.0;

        x0_plus.setZero();
        x0_minus.setZero();

        pinocchio::integrate(model_, x_1_normalized.head(num_positions_), dq_numdiff, x0_plus.head(num_positions_));
        pinocchio::integrate(model_, x_1_normalized.head(num_positions_), -dq_numdiff, x0_minus.head(num_positions_));

        dJdiff_dx0_plus = dStateDelta(x0_plus, x_2, ArgumentPosition::ARG0);    // J1
        dJdiff_dx0_minus = dStateDelta(x0_minus, x_2, ArgumentPosition::ARG0);  // J2

        for (int i = 0; i < 6; ++i)
        {
            for (int k = 0; k < 6; ++k)
            {
                ddStateDelta(k)(i, j) = (dJdiff_dx0_plus(k, i) - dJdiff_dx0_minus(k, i)) / eps;
            }
        }
    }

    return ddStateDelta;
}

void SatelliteDynamicsSolver::Integrate(const StateVector& x, const StateVector& dx, const double dt, StateVector& xout)
{
    Eigen::VectorXd dx_times_dt = dt * dx;
    Eigen::VectorXd x_normalized = x;
    NormalizeQuaternionInConfigurationVector(x_normalized);
    pinocchio::integrate(model_, x_normalized.head(num_positions_), dx_times_dt.head(num_velocities_), xout.head(num_positions_));
    xout.tail(num_velocities_) = x.tail(num_velocities_) + dx_times_dt.tail(num_velocities_);
}

Eigen::VectorXd SatelliteDynamicsSolver::SimulateOneStep(const StateVector& x, const ControlVector& u)
{
    Eigen::VectorXd xout(num_positions_ + num_velocities_);
    Eigen::VectorXd dx = f(x, u);
    Integrate(x, dx, dt_, xout);
    return xout;
}

Eigen::MatrixXd SatelliteDynamicsSolver::fx(const StateVector& x, const ControlVector& u)
{
    UpdateExternalForceInputFromThrusters(u.head(num_thrusters_));
    tau_.tail(num_aux_joints_) = u.segment(num_thrusters_, num_aux_joints_);

    // Four quadrants should be: 0, Identity, ddq_dq, ddq_dv
    // 0 and Identity are set during initialisation. Here, we pass references to ddq_dq, ddq_dv to the algorithm.
    pinocchio::computeABADerivatives(model_, *pinocchio_data_,
                                     x.head(num_positions_),
                                     x.tail(num_velocities_),
                                     tau_,
                                     fext_,
                                     fx_.block(num_velocities_, 0, num_velocities_, num_velocities_),
                                     fx_.block(num_velocities_, num_velocities_, num_velocities_, num_velocities_),
                                     Minv_);

    return fx_;
}

Eigen::MatrixXd SatelliteDynamicsSolver::fu(const StateVector& x, const ControlVector& u)
{
    UpdateExternalForceInputFromThrusters(u.head(num_thrusters_));
    tau_.tail(num_aux_joints_) = u.segment(num_thrusters_, num_aux_joints_);

    pinocchio::computeABADerivatives(model_, *pinocchio_data_, x.head(num_positions_), x.tail(num_velocities_), tau_, fext_);

    // da/du = daba/dtau * dtau/du + daba/dfext * dfext/du
    //       = M^-1      * dtau/du + M^-1 J^T   * dfext/du

    fu_.bottomRows(num_velocities_).setZero();
    // The arm, if any:
    fu_.bottomRows(num_velocities_).noalias() += pinocchio_data_->Minv * dtau_du_;

    // The thrusters:
    fu_.bottomRows(num_velocities_).noalias() += pinocchio_data_->Minv * daba_dfext_;

    return fu_;
}

void SatelliteDynamicsSolver::ComputeDerivatives(const StateVector& x, const ControlVector& u)
{
    UpdateExternalForceInputFromThrusters(u.head(num_thrusters_));
    tau_.tail(num_aux_joints_) = u.segment(num_thrusters_, num_aux_joints_);

    // Four quadrants should be: 0, Identity, ddq_dq, ddq_dv
    // 0 and Identity are set during initialisation. Here, we pass references to ddq_dq, ddq_dv to the algorithm.
    pinocchio::computeABADerivatives(model_, *pinocchio_data_,
                                     x.head(num_positions_),
                                     x.tail(num_velocities_),
                                     tau_,
                                     fext_,
                                     fx_.block(num_velocities_, 0, num_velocities_, num_velocities_),
                                     fx_.block(num_velocities_, num_velocities_, num_velocities_, num_velocities_),
                                     Minv_);

    fu_.bottomRows(num_velocities_).setZero();

    // The arm, if any:
    fu_.bottomRows(num_velocities_).noalias() += Minv_ * dtau_du_;

    // The thrusters:
    fu_.bottomRows(num_velocities_).noalias() += Minv_ * daba_dfext_;
}
}  // namespace exotica
