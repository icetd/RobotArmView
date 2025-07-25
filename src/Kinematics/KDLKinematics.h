#pragma once

#include <tree.hpp>
#include <chain.hpp>
#include <jntarray.hpp>
#include <frames.hpp>
#include <memory>
#include <string>

#include <chainfksolverpos_recursive.hpp>
#include <chainiksolvervel_pinv.hpp>
#include <chainiksolverpos_nr_jl.hpp>
#include <chainiksolverpos_lma.hpp>
#include <Kinematics/trac_ik/trac_ik.hpp>

class KDLKinematics
{
public:
    enum class IKSolverType
    {
        LMA,
        NR_JL,
        TRAC_IK
    };

    KDLKinematics(const KDL::Tree &robot_tree,
                  const std::string &base_link,
                  const std::string &tip_link);

    bool isValid() const;

    bool computeFK(const KDL::JntArray &joint_positions, KDL::Frame &out_pose) const;

    bool computeIK(const KDL::Frame &desired_pose,
                   const KDL::JntArray &initial_guess,
                   KDL::JntArray &out_joint_positions,
                   IKSolverType solver_type = IKSolverType::TRAC_IK) const;

    size_t getDOF() const;

private:
    KDL::Chain kdl_chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    std::unique_ptr<TRAC_IK::TRAC_IK> trac_ik_solver_;

    KDL::JntArray q_min_, q_max_;
    bool valid_;
};
