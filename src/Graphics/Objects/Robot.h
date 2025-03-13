#ifndef ROBOT_H
#define ROBOT_H

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "urdf_parser/urdf_parser.h"

#include "../Renderer/Shader.h"
#include "../Renderer/Renderer.h"
#include "../Renderer/Texture.h"
#include "Model.h"
#include "Object.h"
#include "../Renderer/Camera.h"
#include "Model.h"
#include "Object.h"

class Robot
{
public:
    Robot();
    ~Robot();
    void Draw(Shader &shader, GLenum mode);
    void Robot::updateJointAngles(Shader &shader, Camera &camera);
    int loadURDF(const std::string filepath, const std::string filename);
    void removeAll();
    std::string getName() { return m_name; }
    std::vector<Model *> &getJoints() { return m_Joints; }
    std::vector<ObjectStructure *> &getJointObjects() { return m_JointObjects; }
    glm::mat4 calTransMat(ObjectStructure *link);

private:
    std::string m_name;
    std::string m_token;

    Model *m_joint;
    std::vector<Model *> m_Joints;
    std::vector<ObjectStructure *> m_JointObjects;

    void addChildLinks(urdf::LinkConstSharedPtr link, ObjectStructure *parent);
};

#endif