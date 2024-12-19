#include "Robot.h"

#include <iostream>
#include <fstream>
#include <windows.h>
#include "Core/log.h"

const glm::mat4 init = glm::mat4(1.0f);

Robot::Robot()
{
}

Robot::~Robot() {
    for (auto joint : m_Joints) {
        delete joint;
    }
}

void Robot::Draw(Shader& shader, GLenum mode) {
    for (auto joint : m_Joints) {
        joint->Draw(shader, mode);
    }
}

glm::mat4 Robot::calTransMat (ObjectStructure *link)
{    
    if (link->parent == nullptr)
        return link->joint_transmat;
        
    return calTransMat(link->parent) * link->joint_transmat;
}


void Robot::updateJointAngles(Shader &shader, Camera &camera) {
    for (int i = 0; i < m_JointObjects.size(); i++)
    {
        if (!m_JointObjects[i]->modelDefined)
        {
            m_joint = new Model(m_JointObjects[i]);
            m_Joints.push_back(m_joint);
            m_JointObjects[i]->modelDefined = true;
        }
    }

    for (int i = 0; i < m_JointObjects.size(); i++)
    {
        shader.Bind();
        //glm::vec3 lightPos = glm::vec3(100.0f, 100.0f, 100.0f);
        glm::vec3 lightPos = camera.GetEye();
        shader.SetUniformMat4f("model", calTransMat(m_JointObjects[i]));
        shader.SetUniformMat4f("projection", camera.GetProjMatrix());
        shader.SetUniformMat4f("view", camera.GetViewMatrix());
        shader.SetUniformVec3f("lightColor", glm::vec3(1.0f, 1.0f, 1.0f));
        shader.SetUniformVec3f("lightPos", lightPos);
        shader.SetUniformVec3f("camPos", camera.GetEye());
        m_Joints[i]->Draw(shader, GL_FILL);

        // update status
        m_JointObjects[i]->objTranslation = m_JointObjects[i]->joint_transmat[3];
        m_JointObjects[i]->objRotation = glm::mat3(m_JointObjects[i]->joint_transmat);
    }
}

int Robot::loadURDF(const std::string filepath, const std::string filename) 
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "ERROR: Failed to open URDF file: " << filename << std::endl;
        return -1;
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string xml_string = buffer.str();
    file.close();
    
    urdf::ModelInterfaceSharedPtr robot = urdf::parseURDF(xml_string);
    if (!robot) {
        std::cerr << "ERROR: Failed to parse URDF file" << std::endl;
        return -1;
    }

    m_name = robot->getName();

    urdf::LinkConstSharedPtr link = robot->getRoot();
    urdf::GeometrySharedPtr geom = link->visual->geometry;
    urdf::MeshSharedPtr mesh = urdf::dynamic_pointer_cast<urdf::Mesh>(geom);
 
    ObjectStructure *obj = new ObjectStructure();
    
    glm::vec3 scale = glm::vec3(mesh->scale.x, mesh->scale.y, mesh->scale.z);
    glm::vec3 axis = glm::vec3(0.0f);
    std::string mat_name;
    
    m_token = filepath + "/../";
    obj->name = link->name;
    obj->path = m_token + mesh->filename;
    obj->joint_type = 0;
    obj->joint_transmat = init;
    obj->vis_transmat = init;
    obj->scale = scale;
    obj->axis = axis;

    // 处理材质
    if (link->visual->material != nullptr) {
        mat_name = link->visual->material->name;
        obj->setColor(mat_name);
    } else {
        mat_name = "White";
    }

    // 如果材质为空且有颜色，使用这个颜色
    if (mat_name.empty() && link->visual->material != nullptr)
    {
        obj->modelColor = glm::vec3(
            link->visual->material->color.r,
            link->visual->material->color.g,
            link->visual->material->color.b);
    }

    m_JointObjects.push_back(obj);
    addChildLinks(link, obj);
    return 0;
}

void Robot::addChildLinks(urdf::LinkConstSharedPtr link, ObjectStructure *parent)
{
    double roll, pitch, yaw;
    double x, y, z;
    double roll_, pitch_, yaw_;
    double x_, y_, z_;
    double axis_x, axis_y, axis_z = 0.0f;
    urdf::GeometrySharedPtr geom;
    std::string token;
    std::string file_name;
    ObjectStructure *child_node;

    for (const auto& child : link->child_links)
    {
        // 获取关节变换（从 parent_joint 获取旋转和平移）
        child->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw); // rotation
        x = child->parent_joint->parent_to_joint_origin_transform.position.x;
        y = child->parent_joint->parent_to_joint_origin_transform.position.y;
        z = child->parent_joint->parent_to_joint_origin_transform.position.z;

        // 获取关节轴
        axis_x = child->parent_joint->axis.x;
        axis_y = child->parent_joint->axis.y;
        axis_z = child->parent_joint->axis.z;
        glm::vec3 axis(axis_x, axis_y, axis_z);

        // 处理视觉信息
        if (child->visual)
        {
            geom = child->visual->geometry;
            glm::mat4 joint_transmat = glm::mat4(1.0f);
            joint_transmat = glm::translate(joint_transmat, glm::vec3(x, y, z)); 
            joint_transmat = glm::rotate(joint_transmat, float(roll), glm::vec3(-1.0f, 0.0f, 0.0f)); 
            joint_transmat = glm::rotate(joint_transmat, float(pitch), glm::vec3(0.0f, -1.0f, 0.0f)); 
            joint_transmat = glm::rotate(joint_transmat, float(yaw), glm::vec3(0.0f, 0.0f, -1.0f));     

            glm::mat4 vis_transmat = glm::mat4(1.0f);
            child->visual->origin.rotation.getRPY(roll_, pitch_, yaw_);
            x_ = child->visual->origin.position.x;
            y_ = child->visual->origin.position.y;
            z_ = child->visual->origin.position.z;
            vis_transmat = glm::translate(vis_transmat, glm::vec3(x_, y_, z_));
            vis_transmat = glm::rotate(vis_transmat, float(roll_), glm::vec3(1.0f, 0.0f, 0.0f)); 
            vis_transmat = glm::rotate(vis_transmat, float(pitch_), glm::vec3(0.0f, 1.0f, 0.0f)); 
            vis_transmat = glm::rotate(vis_transmat, float(yaw_), glm::vec3(0.0f, 0.0f, 1.0f));   

            std::string mat_name;

            // 处理网格（Mesh）几何体
            if (geom->type == urdf::Geometry::MESH)
            {
                child_node = new ObjectStructure();
                urdf::MeshSharedPtr m = urdf::dynamic_pointer_cast<urdf::Mesh>(geom);
                if (m) {
                    // 生成文件路径（根据您的实际需求定义 m_token）
                    file_name = m_token + m->filename;

                    glm::vec3 scale = glm::vec3(m->scale.x, m->scale.y, m->scale.z);

                    // 设置子节点信息
                    child_node->name = child->name;
                    child_node->path = file_name;
                    child_node->joint_transmat = joint_transmat;
                    child_node->vis_transmat = vis_transmat;
                    child_node->scale = scale;
                    child_node->axis = axis;
                    child_node->parent = parent;
                    child_node->joint_type = child->parent_joint->type;

                    // 获取limits
                    if (child->parent_joint->limits) {
                        child_node->limitAngle.offort_angle = child->parent_joint->limits->effort * RADIANS_TO_ANGLE;
                        child_node->limitAngle.lower_angle = child->parent_joint->limits->lower * RADIANS_TO_ANGLE;
                        child_node->limitAngle.upper_angle = child->parent_joint->limits->upper * RADIANS_TO_ANGLE;
                        child_node->limitAngle.velocity_angle = child->parent_joint->limits->velocity * RADIANS_TO_ANGLE;
                    } else {
                        child_node->limitAngle.offort_angle = 0;
                        child_node->limitAngle.lower_angle = -90;
                        child_node->limitAngle.upper_angle = 90;
                        child_node->limitAngle.velocity_angle = 90; 
                    }

                    // 处理材质
                    if (child->visual->material != nullptr) {
                        mat_name = child->visual->material->name;
                        child_node->setColor(mat_name);
                    }
                    else {
                        mat_name = "White";
                    }

                    // 如果材质为空且有颜色，使用默认颜色
                    if (mat_name.empty() && child->visual->material != nullptr) {
                        child_node->modelColor = glm::vec3 (
                            child->visual->material->color.r,
                            child->visual->material->color.g,
                            child->visual->material->color.b
                        );
                    }
                    m_JointObjects.push_back(child_node);

                    // 递归解析子链接
                    addChildLinks(child, child_node);
                }
            }
        }
    }
}

void Robot::removeAll()
{
    std::vector<ObjectStructure*>().swap(m_JointObjects);
    std::vector<Model*>().swap(m_Joints); 
}