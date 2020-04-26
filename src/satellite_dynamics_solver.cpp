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

    // Get frame Ids
    bot0_id_ = model_.getFrameId("base_to_thruster_bot");
    bot1_id_ = model_.getFrameId("base_to_thruster_bot_1");
    bot2_id_ = model_.getFrameId("base_to_thruster_bot_2");
    bot3_id_ = model_.getFrameId("base_to_thruster_bot_3");
    bot4_id_ = model_.getFrameId("base_to_thruster_bot_4");
    top0_id_ = model_.getFrameId("base_to_thruster_top");
    top1_id_ = model_.getFrameId("base_to_thruster_top_1");
    top2_id_ = model_.getFrameId("base_to_thruster_top_2");
    top3_id_ = model_.getFrameId("base_to_thruster_top_3");
    top4_id_ = model_.getFrameId("base_to_thruster_top_4");

    // Force directions
    f1_ << 0, 0, -1, 0, 0, 0;
    f2_ << 0, 1, 0, 0, 0, 0;
    f3_ << -1, 0, 0, 0, 0, 0;
    f4_ << 0, -1, 0, 0, 0, 0;
    f5_ << 1, 0, 0, 0, 0, 0;

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
    auto bot0 = model_.frames[bot0_id_].placement.toDualActionMatrix(),
         bot1 = model_.frames[bot1_id_].placement.toDualActionMatrix(),
         bot2 = model_.frames[bot2_id_].placement.toDualActionMatrix(),
         bot3 = model_.frames[bot3_id_].placement.toDualActionMatrix(),
         bot4 = model_.frames[bot4_id_].placement.toDualActionMatrix();

    auto top0 = model_.frames[top0_id_].placement.toDualActionMatrix(),
         top1 = model_.frames[top1_id_].placement.toDualActionMatrix(),
         top2 = model_.frames[top2_id_].placement.toDualActionMatrix(),
         top3 = model_.frames[top3_id_].placement.toDualActionMatrix(),
         top4 = model_.frames[top4_id_].placement.toDualActionMatrix();

    daba_dfext_.block(0, 0, 6, 1) = bot0 * f1_;
    daba_dfext_.block(0, 1, 6, 1) = bot1 * f2_;
    daba_dfext_.block(0, 2, 6, 1) = bot2 * f3_;
    daba_dfext_.block(0, 3, 6, 1) = bot3 * f4_;
    daba_dfext_.block(0, 4, 6, 1) = bot4 * f5_;
    daba_dfext_.block(0, 5, 6, 1) = -top0 * f1_;
    daba_dfext_.block(0, 6, 6, 1) = top1 * f2_;
    daba_dfext_.block(0, 7, 6, 1) = top2 * f3_;
    daba_dfext_.block(0, 8, 6, 1) = top3 * f4_;
    daba_dfext_.block(0, 9, 6, 1) = top4 * f5_;

    // Set up derivative of actuated joints
    dtau_du_.setZero(model_.nv, num_controls_);
    dtau_du_.block(6, num_thrusters_, num_aux_joints_, num_aux_joints_).setIdentity();
}

void SatelliteDynamicsSolver::UpdateExternalForceInputFromThrusters(const ControlVector& u)
{
    // Non-actuated joints
    auto bot0 = model_.frames[bot0_id_].placement,
         bot1 = model_.frames[bot1_id_].placement,
         bot2 = model_.frames[bot2_id_].placement,
         bot3 = model_.frames[bot3_id_].placement,
         bot4 = model_.frames[bot4_id_].placement;

    auto top0 = model_.frames[top0_id_].placement,
         top1 = model_.frames[top1_id_].placement,
         top2 = model_.frames[top2_id_].placement,
         top3 = model_.frames[top3_id_].placement,
         top4 = model_.frames[top4_id_].placement;

    fext_[1] = pinocchio::Force(bot0.act(pinocchio::Force(f1_ * u(0))) +
                                bot1.act(pinocchio::Force(f2_ * u(1))) +
                                bot2.act(pinocchio::Force(f3_ * u(2))) +
                                bot3.act(pinocchio::Force(f4_ * u(3))) +
                                bot4.act(pinocchio::Force(f5_ * u(4))) +

                                top0.act(pinocchio::Force(-1.0 * f1_ * u(5))) +
                                top1.act(pinocchio::Force(f2_ * u(6))) +
                                top2.act(pinocchio::Force(f3_ * u(7))) +
                                top3.act(pinocchio::Force(f4_ * u(8))) +
                                top4.act(pinocchio::Force(f5_ * u(9))));
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
    Eigen::VectorXd q0_plus(num_positions_), q0_minus(num_positions_);
    Eigen::MatrixXd dJdiff_dq0_plus(num_velocities_, num_velocities_), dJdiff_dq0_minus(num_velocities_, num_velocities_);
    // NB: We only differentiate the first 6 dimensions for the free-flyer joint.
    // The other dimensions evaluate to 0.
    for (int i = 0; i < 6; ++i)
    {
        dq_numdiff.setZero();
        dq_numdiff(i) = eps / 2.0;

        pinocchio::integrate(model_, x_1.head(num_positions_), dq_numdiff, q0_plus);
        pinocchio::integrate(model_, x_1.head(num_positions_), -dq_numdiff, q0_minus);

        pinocchio::dDifference(model_, x_2.head(num_positions_), q0_plus, dJdiff_dq0_plus, pinocchio::ArgumentPosition::ARG1);
        pinocchio::dDifference(model_, x_2.head(num_positions_), q0_minus, dJdiff_dq0_minus, pinocchio::ArgumentPosition::ARG1);

        ddStateDelta(i).topLeftCorner(num_velocities_, num_velocities_).noalias() = (dJdiff_dq0_plus - dJdiff_dq0_minus) / eps;
    }

    return ddStateDelta;
}

void SatelliteDynamicsSolver::Integrate(const StateVector& x, const StateVector& dx, const double dt, StateVector& xout)
{
    Eigen::VectorXd dx_times_dt = dt * dx;
    pinocchio::integrate(model_, x.head(num_positions_), dx_times_dt.head(num_velocities_), xout.head(num_positions_));
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
