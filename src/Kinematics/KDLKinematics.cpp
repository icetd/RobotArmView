#include "KDLKinematics.h"
#include <iostream>
#include <cmath>

#define M_PI 3.14159265358979323846

KDLKinematics::KDLKinematics(const KDL::Tree &robot_tree,
                             const std::string &base_link,
                             const std::string &tip_link)
{
    if (!robot_tree.getChain(base_link, tip_link, kdl_chain_)) {
        std::cerr << "[KDLKinematics] Failed to extract chain from " << base_link << " to " << tip_link << std::endl;
        valid_ = false;
        return;
    }

    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_chain_);
    ik_pos_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(kdl_chain_, *fk_solver_, *ik_vel_solver_, 100, 1e-6);
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_, 1.0e-5, 500, 1.0e-15);

    size_t dof = kdl_chain_.getNrOfJoints();
    q_min_ = KDL::JntArray(dof);
    q_max_ = KDL::JntArray(dof);
    for (size_t i = 0; i < dof; ++i) {
        q_min_(i) = -M_PI;
        q_max_(i) = M_PI;
    }

    trac_ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(kdl_chain_, q_min_, q_max_, 0.005, 1.0E-5, TRAC_IK::SolveType::Distance);

    valid_ = true;
}

bool KDLKinematics::isValid() const
{
    return valid_;
}

bool KDLKinematics::computeFK(const KDL::JntArray &joint_positions, KDL::Frame &out_pose) const
{
    if (!valid_ || joint_positions.rows() != getDOF()) {
        std::cerr << "[KDLKinematics] Invalid FK input." << std::endl;
        return false;
    }

    return fk_solver_->JntToCart(joint_positions, out_pose) >= 0;
}

inline double normalizeAngle(double angle)
{
    angle = fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0)
        angle += 2.0 * M_PI;
    return angle - M_PI;
}

bool KDLKinematics::computeIK(const KDL::Frame &desired_pose,
                              const KDL::JntArray &initial_guess,
                              KDL::JntArray &out_joint_positions,
                              IKSolverType solver_type) const
{
    if (!valid_ || initial_guess.rows() != getDOF()) {
        std::cerr << "[KDLKinematics] Invalid IK input." << std::endl;
        return false;
    }

    bool success = false;
    out_joint_positions = initial_guess;

    switch (solver_type) {
    case IKSolverType::LMA:
        success = ik_solver_->CartToJnt(initial_guess, desired_pose, out_joint_positions) >= 0;
        break;
    case IKSolverType::NR_JL:
        success = ik_pos_solver_->CartToJnt(initial_guess, desired_pose, out_joint_positions) >= 0;
        for (unsigned int i = 0; i < out_joint_positions.rows(); ++i) {
            out_joint_positions(i) = normalizeAngle(out_joint_positions(i));
        }
        break;
    case IKSolverType::TRAC_IK:
        success = trac_ik_solver_->CartToJnt(initial_guess, desired_pose, out_joint_positions) >= 0;
        break;
    }

    return success;
}

size_t KDLKinematics::getDOF() const
{
    return kdl_chain_.getNrOfJoints();
}
