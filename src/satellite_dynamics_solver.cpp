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
    num_controls_ = 10;  // We manually specify the number of thrusters

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
    fx_analytic_.setZero(ndx, ndx);
    fx_analytic_.topRightCorner(num_velocities_, num_velocities_).setIdentity();
    fu_analytic_.setZero(ndx, num_controls_);
}

pinocchio::container::aligned_vector<pinocchio::Force> SatelliteDynamicsSolver::GetExternalForceInputFromThrusters(const ControlVector& u)
{
    // external forces
    pinocchio::container::aligned_vector<pinocchio::Force> f_ext;

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

    f_ext.push_back(pinocchio::Force::Zero());
    f_ext.push_back(bot0.act(pinocchio::Force(f1_ * u(0))) +
                    bot1.act(pinocchio::Force(f2_ * u(1))) +
                    bot2.act(pinocchio::Force(f3_ * u(2))) +
                    bot3.act(pinocchio::Force(f4_ * u(3))) +
                    bot4.act(pinocchio::Force(f5_ * u(4))) +

                    top0.act(pinocchio::Force(-1 * f1_ * u(5))) +
                    top1.act(pinocchio::Force(f2_ * u(6))) +
                    top2.act(pinocchio::Force(f3_ * u(7))) +
                    top3.act(pinocchio::Force(f4_ * u(8))) +
                    top4.act(pinocchio::Force(f5_ * u(9))));

    return f_ext;
}

Eigen::VectorXd SatelliteDynamicsSolver::f(const StateVector& x, const ControlVector& u)
{
    auto f_ext = GetExternalForceInputFromThrusters(u);
    pinocchio::aba(model_, *pinocchio_data_, x.head(num_positions_).eval(), x.tail(num_velocities_).eval(), Eigen::VectorXd::Zero(model_.nv), f_ext);
    xdot_analytic_.head(num_velocities_) = x.tail(num_velocities_);
    xdot_analytic_.tail(num_velocities_) = pinocchio_data_->ddq;

    return xdot_analytic_;
}

Eigen::VectorXd SatelliteDynamicsSolver::GetPosition(Eigen::VectorXdRefConst x_in)
{
    // Convert quaternion to Euler angles.
    Eigen::VectorXd xyz_rpy = Eigen::VectorXd::Zero(num_positions_ - 1);
    xyz_rpy.head<3>() = x_in.head<3>();
    xyz_rpy.segment<3>(3) = Eigen::Quaterniond(x_in.segment<4>(3)).toRotationMatrix().eulerAngles(0, 1, 2);
    return xyz_rpy;
}

Eigen::VectorXd SatelliteDynamicsSolver::StateDelta(const StateVector& x_1, const StateVector& x_2)
{
    Eigen::VectorXd dx(2 * num_velocities_);
    pinocchio::difference(model_, x_2.head(num_positions_), x_1.head(num_positions_), dx.head(num_velocities_));
    dx.tail(num_velocities_) = x_1.tail(num_velocities_) - x_2.tail(num_velocities_);
    return dx;
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
    auto f_ext = GetExternalForceInputFromThrusters(u);

    // Four quadrants should be: 0, Identity, ddq_dq, ddq_dv
    // 0 and Identity are set during initialisation. Here, we pass references to ddq_dq, ddq_dv to the algorithm.
    pinocchio::computeABADerivatives(model_, *pinocchio_data_,
                                     x.head(num_positions_).eval(),
                                     x.tail(num_velocities_).eval(),
                                     Eigen::VectorXd::Zero(num_velocities_).eval(),
                                     f_ext,
                                     fx_analytic_.block(num_velocities_, 0, num_velocities_, num_velocities_),
                                     fx_analytic_.block(num_velocities_, num_velocities_, num_velocities_, num_velocities_),
                                     fu_analytic_.bottomRightCorner(num_velocities_, num_velocities_));

    return fx_analytic_;
}

// Eigen::MatrixXd SatelliteDynamicsSolver::fu(const StateVector& x, const ControlVector& u)
// {
//     const int NV = num_velocities_;
//     const int NDX = 2 * NV;
//     const int NU = num_controls_;

//     auto f_ext = GetExternalForceInputFromThrusters(u);
//     pinocchio::computeABADerivatives(model_, *pinocchio_data_,
//     x.head(num_positions_).eval(), x.tail(num_velocities_).eval(),
//     Eigen::VectorXd::Zero(model_.nv), f_ext);

//     Eigen::MatrixXd fu_symb = Eigen::MatrixXd::Zero(NDX, NU);
//     HIGHLIGHT("fu_symb = " << fu_symb.rows() << "x" << fu_symb.cols());
//     HIGHLIGHT("pinocchio_data_->Minv = " << pinocchio_data_->Minv.rows() <<
//     "x" << pinocchio_data_->Minv.cols());
//     fu_symb.bottomRightCorner(NV, NU) = pinocchio_data_->Minv;

//     return fu_symb;
// }
}  // namespace exotica
