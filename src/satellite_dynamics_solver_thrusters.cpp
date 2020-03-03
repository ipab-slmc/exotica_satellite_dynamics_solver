//
// Copyright (c) 2019, Wolfgang Merkt
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

REGISTER_DYNAMICS_SOLVER_TYPE("SatelliteDynamicsSolver", exotica::SatelliteDynamicsSolver)

namespace exotica
{
void SatelliteDynamicsSolver::AssignScene(ScenePtr scene_in)
{
    const bool verbose = false;
    // if (scene_in->GetKinematicTree().GetControlledBaseType() == BaseType::FIXED)
    // {
    //     pinocchio::urdf::buildModel(scene_in->GetKinematicTree().GetRobotModel()->getURDF(), model_, verbose);
    // }
    // else if (scene_in->GetKinematicTree().GetControlledBaseType() == BaseType::PLANAR)
    // {
    //     pinocchio::urdf::buildModel(scene_in->GetKinematicTree().GetRobotModel()->getURDF(), pinocchio::JointModelPlanar(), model_, verbose);
    // }
    // else if (scene_in->GetKinematicTree().GetControlledBaseType() == BaseType::FLOATING)
    // {
        pinocchio::urdf::buildModel(scene_in->GetKinematicTree().GetRobotModel()->getURDF(), pinocchio::JointModelFreeFlyer(), model_, verbose);
    // }
    // else
    // {
    //     ThrowPretty("This condition should never happen. Unknown BaseType.");
    // }

    num_positions_ = model_.nq;
    num_velocities_ = model_.nv;
    // num_controls_ = model_.nv;
    num_controls_ = 6;

    model_.gravity = Eigen::Vector3d::Zero();
    // model_.gravity = Eigen::Vector3d(0, 0, 9.81);

    HIGHLIGHT_NAMED("Satellite njoints", model_.njoints);

    pinocchio_data_.reset(new pinocchio::Data(model_));
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
    q << translation, quaternion.x();
    const Eigen::VectorXd v = x.segment<3>(6);
    const Eigen::Vector3d omega = x.tail<3>();

    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(num_velocities_);

    HIGHLIGHT_NAMED("satellite x", x);
    q_dot << v, omega;

    // external forces
    pinocchio::container::aligned_vector<pinocchio::Force> f_ext;
    
    // NOn-actuated joints
    // f_ext.push_back(pinocchio::Force::Zero());
    
    // Eigen::VectorXd f1(6), f2(6), f3(6), f4(6);
    // f1 << 0, 1, 0, 0, 0, 0;
    // f2 << 0, -1, 0, 0, 0, 0;
    // f3 << -1, 0, 0, 0, 0, 0;
    // f4 << 1, 0, 0, 0, 0, 0;
    
    // f_ext.push_back(pinocchio::Force::Zero());
    // f_ext.push_back(pinocchio::Force::Zero());
    // f_ext.push_back(pinocchio::Force(f1 * u(1)));
    // f_ext.push_back(pinocchio::Force(f2 * u(2)));
    // f_ext.push_back(pinocchio::Force(f3 * u(3)));
    // f_ext.push_back(pinocchio::Force(f4 * u(4)));

    // Eigen::VectorXd u_pin = Eigen::VectorXd::Zero(model_.nv);
    // z acceleration
    // u_pin(2) = u(0);

    pinocchio::aba(model_, *pinocchio_data_, q, q_dot, u);
    // pinocchio::aba(model_, *pinocchio_data_, q, q_dot, Eigen::VectorXd::Zero(model_.nv));

    Eigen::VectorXd x_dot = Eigen::VectorXd::Zero(x.size());

    x_dot.head<3>() = v; // velocity in world frame
    x_dot.segment<4>(3) = 0.5 * (quaternion * Eigen::Quaterniond(0, omega(0), omega(1), omega(2))).coeffs();  // via quaternion derivative (cf. https://math.stackexchange.com/a/2099673)

    // x_dot.head(num_positions_) = x.tail(num_positions_);
    x_dot.tail(num_velocities_) = pinocchio_data_->ddq;

    for (int i = 0; i < x_dot.size(); ++ i)
        if (!std::isfinite(x_dot(i)))
            x_dot(i) = 0;

    HIGHLIGHT_NAMED("Satellite x_dot", x_dot);
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

// Eigen::MatrixXd SatelliteDynamicsSolver::fx(const StateVector& x, const ControlVector& u)
// {
//     const int NQ = num_positions_;
//     const int NV = num_velocities_;
//     const int NX = NQ + NV;
//     const int NU = num_controls_;

//     pinocchio::computeABADerivatives(model_, *pinocchio_data_, x.head(num_positions_).eval(), x.tail(num_velocities_).eval(), u.eval());

//     Eigen::MatrixXd fx_symb = Eigen::MatrixXd::Zero(NX, NX);
//     fx_symb.topRightCorner(NV, NV) = Eigen::MatrixXd::Identity(NV, NV);
//     fx_symb.bottomLeftCorner(NQ, NV) = pinocchio_data_->ddq_dq;

//     return fx_symb;
// }

// Eigen::MatrixXd SatelliteDynamicsSolver::fu(const StateVector& x, const ControlVector& u)
// {
//     const int NQ = num_positions_;
//     const int NV = num_velocities_;
//     const int NX = NQ + NV;
//     const int NU = num_controls_;

//     pinocchio::computeABADerivatives(model_, *pinocchio_data_, x.head(num_positions_).eval(), x.tail(num_velocities_).eval(), u.eval());

//     Eigen::MatrixXd fu_symb = Eigen::MatrixXd::Zero(NX, NU);
//     fu_symb.bottomRightCorner(NV, NU) = pinocchio_data_->Minv;

//     return fu_symb;
// }

// Eigen::VectorXd SatelliteDynamicsSolver::InverseDynamics(const StateVector& x)
// {
//     // compute dynamic drift -- Coriolis, centrifugal, gravity
//     // Assume 0 acceleration
//     Eigen::VectorXd u = pinocchio::rnea(model_, *pinocchio_data_,
//                                         x.head(num_positions_).eval(), x.tail(num_velocities_).eval(),
//                                         Eigen::VectorXd::Zero(num_velocities_).eval());

//     return u;
// }

}  // namespace exotica
