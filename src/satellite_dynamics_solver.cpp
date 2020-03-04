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
}

pinocchio::container::aligned_vector<pinocchio::Force> SatelliteDynamicsSolver::GetExternalForceInputFromThrusters(const ControlVector& u)
{
    // external forces
    pinocchio::container::aligned_vector<pinocchio::Force> f_ext;

    // Non-actuated joints

    // Get frame Ids
    int bot0_id = model_.getFrameId("base_to_thruster_bot"),
        bot1_id = model_.getFrameId("base_to_thruster_bot_1"),
        bot2_id = model_.getFrameId("base_to_thruster_bot_2"),
        bot3_id = model_.getFrameId("base_to_thruster_bot_3"),
        bot4_id = model_.getFrameId("base_to_thruster_bot_4");

    auto bot0 = model_.frames[bot0_id].placement,
         bot1 = model_.frames[bot1_id].placement,
         bot2 = model_.frames[bot2_id].placement,
         bot3 = model_.frames[bot3_id].placement,
         bot4 = model_.frames[bot4_id].placement;

    int top0_id = model_.getFrameId("base_to_thruster_top"),
        top1_id = model_.getFrameId("base_to_thruster_top_1"),
        top2_id = model_.getFrameId("base_to_thruster_top_2"),
        top3_id = model_.getFrameId("base_to_thruster_top_3"),
        top4_id = model_.getFrameId("base_to_thruster_top_4");

    auto top0 = model_.frames[top0_id].placement,
         top1 = model_.frames[top1_id].placement,
         top2 = model_.frames[top2_id].placement,
         top3 = model_.frames[top3_id].placement,
         top4 = model_.frames[top4_id].placement;

    Eigen::VectorXd f1(6), f2(6), f3(6), f4(6), f5(6);
    f1 << 0, 0, -1, 0, 0, 0;
    f2 << 0, 1, 0, 0, 0, 0;
    f3 << -1, 0, 0, 0, 0, 0;
    f4 << 0, -1, 0, 0, 0, 0;
    f5 << 1, 0, 0, 0, 0, 0;

    f_ext.push_back(pinocchio::Force::Zero());
    f_ext.push_back(bot0.act(pinocchio::Force(f1 * u(0))) +
                    bot1.act(pinocchio::Force(f2 * u(1))) +
                    bot2.act(pinocchio::Force(f3 * u(2))) +
                    bot3.act(pinocchio::Force(f4 * u(3))) +
                    bot4.act(pinocchio::Force(f5 * u(4))) +

                    top0.act(pinocchio::Force(-1 * f1 * u(5))) +
                    top1.act(pinocchio::Force(f2 * u(6))) +
                    top2.act(pinocchio::Force(f3 * u(7))) +
                    top3.act(pinocchio::Force(f4 * u(8))) +
                    top4.act(pinocchio::Force(f5 * u(9))));

    // HIGHLIGHT_NAMED("force",
    //     bot0.act(pinocchio::Force(f1 * u(0))) +
    //     bot1.act(pinocchio::Force(f2 * u(1))) +
    //     bot2.act(pinocchio::Force(f3 * u(2))) +
    //     bot3.act(pinocchio::Force(f4 * u(3))) +
    //     bot4.act(pinocchio::Force(f5 * u(4)))
    // );

    return f_ext;
}

Eigen::VectorXd SatelliteDynamicsSolver::f(const StateVector& x, const ControlVector& u)
{
    Eigen::Quaterniond quaternion;

    if (x.segment<4>(3).isApprox(Eigen::Vector4d::Zero()))
        quaternion = Eigen::Quaterniond(1, 0, 0, 0);
    else
        quaternion = Eigen::Quaterniond(x.segment<4>(3)).normalized();

    const Eigen::Vector3d translation = x.head<3>();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(num_positions_);
    // HIGHLIGHT_NAMED("asdf",  quaternion.vec());
    // HIGHLIGHT_NAMED("asdf",  translation);
    q << translation, quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w();
    const Eigen::VectorXd v = x.segment<3>(7);
    const Eigen::Vector3d omega = x.tail<3>();

    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(num_velocities_);

    // HIGHLIGHT_NAMED("satellite x", x);
    q_dot << v, omega;

    auto f_ext = GetExternalForceInputFromThrusters(u);
    pinocchio::aba(model_, *pinocchio_data_, q, q_dot, Eigen::VectorXd::Zero(model_.nv), f_ext);

    Eigen::VectorXd x_dot = Eigen::VectorXd::Zero(x.size());

    x_dot.head<3>() = v;  // velocity in world frame
    x_dot.segment<4>(3) = 0.5 * (quaternion * Eigen::Quaterniond(0, omega(0), omega(1), omega(2))).coeffs();  // via quaternion derivative (cf. https://math.stackexchange.com/a/2099673)

    // x_dot.head(num_positions_) = x.tail(num_positions_);
    x_dot.tail(num_velocities_) = pinocchio_data_->ddq;

    // HIGHLIGHT_NAMED("Satellite x_dot", x_dot);
    for (int i = 0; i < x_dot.size(); ++i)
        if (!std::isfinite(x_dot(i)))
            x_dot(i) = 0;

    return x_dot;
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

Eigen::MatrixXd SatelliteDynamicsSolver::fx(const StateVector& x, const ControlVector& u)
{
    const int NV = num_velocities_;
    const int NDX = 2 * NV;

    auto f_ext = GetExternalForceInputFromThrusters(u);
    pinocchio::computeABADerivatives(model_, *pinocchio_data_, x.head(num_positions_).eval(), x.tail(num_velocities_).eval(), Eigen::VectorXd::Zero(model_.nv), f_ext);

    Eigen::MatrixXd fx_symb = Eigen::MatrixXd::Zero(NDX, NDX);
    fx_symb.topRightCorner(NV, NV) = Eigen::MatrixXd::Identity(NV, NV);
    fx_symb.bottomLeftCorner(NV, NV) = pinocchio_data_->ddq_dq;

    return fx_symb;
}

// Eigen::MatrixXd SatelliteDynamicsSolver::fu(const StateVector& x, const
// ControlVector& u)
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
