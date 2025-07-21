#pragma once

#include <string>
#include <urdf_model/model.h>
#include <urdf_world/types.h>
#include <tree.hpp>

namespace kdl_parser
{

    bool treeFromString(const std::string &xml, KDL::Tree &tree);
    bool treeFromFile(const std::string &file, KDL::Tree &tree);
    bool treeFromUrdfModel(const urdf::ModelInterface &robot_model, KDL::Tree &tree);

} // namespace kdl_parser