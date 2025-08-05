#include "Kinematics/kdl_parser.h"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "frames_io.hpp"
#include "urdf_parser/urdf_parser.h"
#include "Core/log.h"

namespace kdl_parser
{

    KDL::Vector toKdl(urdf::Vector3 v)
    {
        return KDL::Vector(v.x, v.y, v.z);
    }

    KDL::Rotation toKdl(urdf::Rotation r)
    {
        return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
    }

    KDL::Frame toKdl(urdf::Pose p)
    {
        return KDL::Frame(toKdl(p.rotation), toKdl(p.position));
    }

    KDL::Joint toKdl(urdf::JointSharedPtr jnt)
    {
        KDL::Frame F_parent_jnt = toKdl(jnt->parent_to_joint_origin_transform);

        switch (jnt->type) {
        case urdf::Joint::FIXED:
            return KDL::Joint(jnt->name, KDL::Joint::None);
        case urdf::Joint::REVOLUTE:
        case urdf::Joint::CONTINUOUS: {
            KDL::Vector axis = toKdl(jnt->axis);
            return KDL::Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, KDL::Joint::RotAxis);
        }
        case urdf::Joint::PRISMATIC: {
            KDL::Vector axis = toKdl(jnt->axis);
            return KDL::Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, KDL::Joint::TransAxis);
        }
        default:
            LOG(WARN, "kdl_parser Converting unknown joint type of joint '%s' into a fixed joint", jnt->name.c_str());
            return KDL::Joint(jnt->name, KDL::Joint::None);
        }
    }

    KDL::RigidBodyInertia toKdl(urdf::InertialSharedPtr i)
    {
        KDL::Frame origin = toKdl(i->origin);
        double kdl_mass = i->mass;
        KDL::Vector kdl_com = origin.p;
        KDL::RotationalInertia urdf_inertia(i->ixx, i->iyy, i->izz, i->ixy, i->ixz, i->iyz);

        KDL::RigidBodyInertia kdl_inertia_wrt_com_workaround =
            origin.M * KDL::RigidBodyInertia(0, KDL::Vector::Zero(), urdf_inertia);

        KDL::RotationalInertia kdl_inertia_wrt_com =
            kdl_inertia_wrt_com_workaround.getRotationalInertia();

        return KDL::RigidBodyInertia(kdl_mass, kdl_com, kdl_inertia_wrt_com);
    }

    bool addChildrenToTree(urdf::LinkConstSharedPtr root, KDL::Tree &tree)
    {
        std::vector<urdf::LinkSharedPtr> children = root->child_links;
        LOG(INFO, "kdl_parser Link %s had %zu children.", root->name.c_str(), children.size());

        KDL::RigidBodyInertia inert(0);
        if (root->inertial) {
            inert = toKdl(root->inertial);
        }

        KDL::Joint jnt = toKdl(root->parent_joint);

        KDL::Segment sgm(root->name, jnt,
                         toKdl(root->parent_joint->parent_to_joint_origin_transform), inert);

        tree.addSegment(sgm, root->parent_joint->parent_link_name);

        for (size_t i = 0; i < children.size(); ++i) {
            if (!addChildrenToTree(children[i], tree)) {
                return false;
            }
        }
        return true;
    }

    bool treeFromFile(const std::string &file, KDL::Tree &tree)
    {
        std::ifstream t(file);
        if (!t.is_open()) {
            LOG(ERRO, "kdl_parser Failed to open file: %s", file.c_str());
            return false;
        }

        std::stringstream buffer;
        buffer << t.rdbuf();
        return treeFromString(buffer.str(), tree);
    }

    bool treeFromString(const std::string &xml, KDL::Tree &tree)
    {
        urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDF(xml);
        if (!robot_model) {
            LOG(ERRO, "kdl_parser Could not parse the URDF string.");
            return false;
        }
        return treeFromUrdfModel(*robot_model, tree);
    }

    bool treeFromUrdfModel(const urdf::ModelInterfaceSharedPtr &robot_model, KDL::Tree &tree)
    {
        if (!robot_model || !robot_model->getRoot()) {
            return false;
        }
        return treeFromUrdfModel(*robot_model, tree);
    }

    bool treeFromUrdfModel(const urdf::ModelInterface &robot_model, KDL::Tree &tree)
    {
        if (!robot_model.getRoot()) {
            return false;
        }

        tree = KDL::Tree(robot_model.getRoot()->name);

        if (robot_model.getRoot()->inertial) {
            LOG(WARN, "kdl_parser The root link %s has an inertia specified in the URDF, but KDL does not support it.", robot_model.getRoot()->name.c_str());
        }

        for (size_t i = 0; i < robot_model.getRoot()->child_links.size(); ++i) {
            if (!addChildrenToTree(robot_model.getRoot()->child_links[i], tree)) {
                return false;
            }
        }

        return true;
    }
} // namespace kdl_parser
