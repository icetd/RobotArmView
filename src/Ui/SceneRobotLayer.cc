#include "SceneRobotLayer.h"
#include "../Core/Application.h"
#include "../Core/Log.h"
#include <set>
#include <windows.h>
#include <regex>
#include <string>
#include <iostream>

void SceneRobotLayer::OnAttach()
{
    glm::vec3 eye(5.0f, 5.f, 5.f);
    glm::vec3 lookat(0.0f, 0.0f, 0.0f);
    glm::vec3 upVector(0.0f, 0.0f, 1.0f);

    float width = Application::GetInstance()->GetWidth();
    float height = Application::GetInstance()->GetHeight();

    m_frameBuffer = new FrameBuffer(width, height);

    m_TextureLightShader = new Shader("res/shaders/TextureLight.shader");
    m_TextureShader = new Shader("res/shaders/Texture.shader");

    m_ColorLightShader = new Shader("res/shaders/ColorLight.shader");
    m_ColorShader = new Shader("res/shaders/Color.shader");
    m_ColorDShader = new Shader("res/shaders/ColorD.shader");

    m_camera = new Camera(eye, lookat, upVector, width, height);
    Application::GetInstance()->InitCamera(m_camera, m_frameBuffer);

    m_robot = new Robot();
    m_senceRobot = new SceneRobot(m_robot);

    m_robot->loadURDF("res/robot/dummy_robot/urdf/", "res/robot/dummy_robot/urdf/dummy_ros_sim.urdf");

    // 加载 URDF 成功后初始化 KDLKinematics
    std::string urdfPath = "res/robot/dummy_robot/urdf/dummy_ros_sim.urdf";
    KDL::Tree tree;
    if (kdl_parser::treeFromFile(urdfPath, tree)) {
        m_kinematics = std::make_unique<KDLKinematics>(tree, m_robot->getJointObjects()[0]->name, m_robot->getJointObjects()[m_robot->getJointObjects().size() - 1]->name);
    }

    KDL::JntArray q(m_kinematics->getDOF());
    for (int i = 0; i < q.rows(); i++)
        q(i) = m_robot->getJointObjects()[i + 1]->getAngle() * deg2rad;

    KDL::Frame eeFrame;
    if (m_kinematics->computeFK(q, eeFrame)) {
        m_endEffectorMatrix = toGlmMatrix(eeFrame);
    }

    isIkSiledrMode = false;
    isIkDragMode = false;
    showAxis = false;
    showGrid = true;
    showProjectionLines = true;
}

void SceneRobotLayer::OnUpdate(float ts)
{
    m_camera->UpdateProjMatrix();
    m_camera->UpdateViewMatrix();
    m_proj = m_camera->GetProjMatrix();
    m_view = m_camera->GetViewMatrix();

    m_frameBuffer->Bind();

    m_senceRobot->UpdateStatus(*m_ColorLightShader, *m_camera);
    if (showGrid)
        m_senceRobot->DrawGrid(*m_ColorShader, *m_camera);
    if (showAxis)
        m_senceRobot->DrawAxis(*m_ColorShader, *m_camera);
    if (showProjectionLines)
        m_senceRobot->DrawVerticalLines(*m_ColorDShader, *m_camera);

    m_frameBuffer->Unbind();

    ShowModelSence();
    ShowModelLoad();
    m_frameBuffer->Unbind();
}

void SceneRobotLayer::OnDetach()
{
    delete m_camera;
    delete m_frameBuffer;
    delete m_TextureLightShader;
    delete m_TextureShader;
    delete m_ColorLightShader;
    delete m_ColorShader;
    delete m_senceRobot;
    delete m_robot;
}

void SceneRobotLayer::OnUIRender()
{}

void SceneRobotLayer::ShowModelSence()
{
    ImGui::Begin("Scene");
    {
        ImGui::BeginChild("GameRender");
        float width = ImGui::GetContentRegionAvail().x;
        float height = ImGui::GetContentRegionAvail().y;
        m_width = width;
        m_height = height;

        // load framebuffer
        ImGui::Image((ImTextureID)m_frameBuffer->GetFrameTexture(), ImGui::GetContentRegionAvail(), ImVec2(0, 1), ImVec2(1, 0));

        if (isIkDragMode) {
            // ImGuizmo 控制末端
            ImGuizmo::SetOrthographic(true);
            ImGuizmo::SetDrawlist();
            ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, width, height);
            ImGuizmo::Style &style = ImGuizmo::GetStyle();

            ImGuizmo::Manipulate(glm::value_ptr(m_view), glm::value_ptr(m_proj),
                                 ImGuizmo::OPERATION::TRANSLATE | ImGuizmo::OPERATION::ROTATE,
                                 ImGuizmo::MODE::LOCAL, glm::value_ptr(m_endEffectorMatrix));

            // 拖动后求IK
            KDL::Frame targetPose = fromGlmMatrix(m_endEffectorMatrix);
            KDL::JntArray q_seed(m_kinematics->getDOF());
            for (int i = 0; i < q_seed.rows(); ++i)
                q_seed(i) = 0.0;

            KDL::JntArray q_result;
            if (m_kinematics->computeIK(targetPose, q_seed, q_result)) {
                for (int i = 0; i < q_result.rows(); ++i)
                    m_robot->getJointObjects()[i + 1]->setAngle(q_result(i) * rad2deg);
                m_lastIkSuccessMatrix = m_endEffectorMatrix;
                LOG(INFO, "IK from ImGuizmo success");
            } else {
                m_endEffectorMatrix = m_lastIkSuccessMatrix;
                LOG(WARN, "IK from ImGuizmo failed");
            }
        }
    }
    ImGui::EndChild();
    ImGui::End();
}

void SceneRobotLayer::ShowModelLoad()
{
    ImGui::Begin(u8"模型加载");
    ImVec2 contentSize = ImGui::GetContentRegionAvail();
    const float buttonWidth = contentSize.x;
    if (ImGui::Button(u8"add urdf", ImVec2(buttonWidth, 25))) {
        ImGui::NewLine();
        OPENFILENAME ofn;
        char szFile[260];
        const char *filter = "All\0*.*\0Text\0*.TXT\0";
        HWND hwnd = NULL;
        HANDLE hf;
        // Initialize OPENFILENAME
        ZeroMemory(&ofn, sizeof(ofn));
        ofn.lStructSize = sizeof(ofn);
        ofn.hwndOwner = hwnd;
        ofn.lpstrFile = szFile;
        ofn.lpstrFile[0] = '\0';
        ofn.nMaxFile = sizeof(szFile);
        ofn.lpstrFilter = filter;
        ofn.nFilterIndex = 1;
        ofn.lpstrFileTitle = NULL;
        ofn.nMaxFileTitle = 0;
        ofn.lpstrInitialDir = NULL;
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
        if (GetOpenFileName(&ofn) == TRUE) {
            char *output = new char[strlen(szFile) + 1];
            strcpy(output, szFile);
            convertPath(output);

            ObjectStructure *obj = new ObjectStructure();
            obj->path = output;

            char *token = NULL;
            char *name = NULL;
            const char *delimeter = "\\";
            token = strtok(output, delimeter);
            while (token != NULL) {
                name = token;
                token = strtok(NULL, delimeter);
            }
            LOG(INFO, "TOKEN %s\n", name);

            std::string strName = name;
            size_t lastSlashPos = strName.find_last_of('/');
            std::string filename = strName.substr(lastSlashPos + 1);
            std::string filepath = strName.substr(0, lastSlashPos);

            m_robot->removeAll();
            m_robot->loadURDF(filepath, filename);

            std::string urdfPath = filepath + "/" + filename;
            KDL::Tree tree;

            if (kdl_parser::treeFromFile(urdfPath, tree)) {
                m_kinematics = std::make_unique<KDLKinematics>(tree, m_robot->getJointObjects()[0]->name, m_robot->getJointObjects()[m_robot->getJointObjects().size() - 1]->name);
            }

            KDL::JntArray q(m_kinematics->getDOF());
            for (int i = 0; i < q.rows(); i++)
                q(i) = m_robot->getJointObjects()[i + 1]->getAngle() * deg2rad;

            KDL::Frame eeFrame;
            if (m_kinematics->computeFK(q, eeFrame)) {
                m_endEffectorMatrix = toGlmMatrix(eeFrame);
            }
        }
    }

    ImGui::Separator();
    ImGui::Text("%s", m_robot->getName().c_str());
    static int selected = 0;
    for (int i = 0; i < m_robot->getJointObjects().size(); i++) {
        std::string name = m_robot->getJointObjects()[i]->name;

        int depth = i;
        std::string prefix(depth, '-');
        name = prefix + name;

        // 显示关节名称
        if (ImGui::Selectable(name.c_str(), selected == i)) {
            selected = i;
            LOG(INFO, "Selected Object %d", selected);
            m_selected = i;
        }
    }

    ImGui::NewLine();
    ImGui::Separator();
    for (int i = 1; i < m_robot->getJointObjects().size(); ++i) {
        ObjectStructure *jointObject = m_robot->getJointObjects()[i];

        if (jointObject->joint_type == ObjectStructure::JointType::REVOLUTE) {
            float angle = jointObject->getAngle();
            float minAngle = jointObject->limits.lower;
            float maxAngle = jointObject->limits.upper;

            if (ImGui::SliderFloat(("Joint " + std::to_string(i) + " Angle").c_str(), &angle, minAngle, maxAngle)) {
                jointObject->setAngle(angle); // 更新关节的角度
            }
        }

        if (jointObject->joint_type == ObjectStructure::JointType::CONTINUOUS) {
            float angle = jointObject->getAngle();
            float minAngle = 360;
            float maxAngle = -360;

            if (ImGui::SliderFloat(("Joint " + std::to_string(i) + " Angle").c_str(), &angle, minAngle, maxAngle)) {
                jointObject->setAngle(angle); // 更新关节的角度
            }
        }

        if (jointObject->joint_type == ObjectStructure::JointType::PRISMATIC) {
            float pos = jointObject->getPos();
            float minPos = jointObject->limits.lower;
            float maxPos = jointObject->limits.upper;

            if (ImGui::SliderFloat(("Joint " + std::to_string(i) + " pos").c_str(), &pos, minPos, maxPos)) {
                jointObject->setPos(pos);
            }
        }
    }

    if (ImGui::Button("Reset All Joints")) {
        for (int i = 1; i < m_robot->getJointObjects().size(); ++i) {
            ObjectStructure *jointObject = m_robot->getJointObjects()[i];
            if (jointObject->joint_type == ObjectStructure::JointType::REVOLUTE) {
                float defaultAngle = 0.0f;
                jointObject->setAngle(defaultAngle);
            }
        }
    }

    if (ImGui::Button("Solve FK")) {
        KDL::JntArray q(m_kinematics->getDOF());
        for (int i = 0; i < q.rows(); i++)
            q(i) = m_robot->getJointObjects()[i + 1]->getAngle() * deg2rad;
        KDL::Frame currentPose;
        if (m_kinematics->computeFK(q, currentPose)) {
            double rx, ry, rz;
            currentPose.M.GetRPY(rx, ry, rz);
            KDL::Vector pos = currentPose.p;

            targetX = pos.x() * 1000;
            targetY = pos.y() * 1000;
            targetZ = pos.z() * 1000;
            targetRoll = rx * rad2deg;
            targetPitch = ry * rad2deg;
            targetYaw = rz * rad2deg;

            LOG(INFO, "FK result: pos = [%.3f, %.3f, %.3f]  rpy = [%.3f, %.3f, %.3f]",
                pos.x(), pos.y(), pos.z(),
                rx * rad2deg, ry * rad2deg, rz * rad2deg);
        }

        m_endEffectorMatrix = toGlmMatrix(currentPose);
    }

    if (isIkDragMode) {
        KDL::Frame currentPose;
        double rx, ry, rz;
        currentPose = fromGlmMatrix(m_endEffectorMatrix);
        currentPose.M.GetRPY(rx, ry, rz);
        KDL::Vector pos = currentPose.p;

        targetX = pos.x() * 1000;
        targetY = pos.y() * 1000;
        targetZ = pos.z() * 1000;
        targetRoll = rx * rad2deg;
        targetPitch = ry * rad2deg;
        targetYaw = rz * rad2deg;

        LOG(INFO, "FK result: pos = [%.3f, %.3f, %.3f]  rpy = [%.3f, %.3f, %.3f]",
            pos.x(), pos.y(), pos.z(),
            rx * rad2deg, ry * rad2deg, rz * rad2deg);
    }

    ImGui::Separator();
    ImGui::Text("IK Target Pose");

    ImGui::SliderFloat("Target X (mm)", &targetX, -2000, 2000);
    ImGui::SliderFloat("Target Y (mm)", &targetY, -2000, 2000);
    ImGui::SliderFloat("Target Z (mm)", &targetZ, -2000, 2000);
    ImGui::SliderFloat("Roll (deg)", &targetRoll, -180, 180);
    ImGui::SliderFloat("Pitch (deg)", &targetPitch, -180, 180);
    ImGui::SliderFloat("Yaw (deg)", &targetYaw, -180, 180);

    static bool ik_success;

    if (ImGui::Button("Solve IK") || isIkSiledrMode) {
        if (!m_kinematics->isValid()) {
            LOG(ERRO, "KDLKinematics not initialized");
        } else {
            int dof = m_kinematics->getDOF();
            KDL::JntArray q_seed(dof);
            for (int i = 0; i < dof; i++)
                q_seed(i) = 0.0f;

            KDL::Frame targetPose(
                KDL::Rotation::RPY(targetRoll * deg2rad, targetPitch * deg2rad, targetYaw * deg2rad),
                KDL::Vector(targetX / 1000.0, targetY / 1000.0, targetZ / 1000.0));

            KDL::JntArray q_result;
            if (m_kinematics->computeIK(targetPose, q_seed, q_result)) {
                for (int i = 0; i < dof; ++i) {
                    m_robot->getJointObjects()[i + 1]->setAngle(q_result(i) * rad2deg);
                }

                // 再通过 FK 求出当前末端姿态
                KDL::Frame currentPose;
                if (m_kinematics->computeFK(q_result, currentPose)) {
                    double rx, ry, rz;
                    currentPose.M.GetRPY(rx, ry, rz);
                    KDL::Vector pos = currentPose.p;

                    LOG(INFO, "FK result: pos = [%.3f, %.3f, %.3f]  rpy = [%.3f, %.3f, %.3f]",
                        pos.x(), pos.y(), pos.z(),
                        rx * rad2deg, ry * rad2deg, rz * rad2deg);
                }
                m_endEffectorMatrix = toGlmMatrix(currentPose);
                ik_success = true;
            } else {
                ik_success = false;
                LOG(WARN, "IK Failed.");
            }


        }
    }
    ImGui::SameLine();
    (ik_success) ? ImGui::TextColored(ImVec4(0.0, 1.0, 0.0, 1.0), "success") : ImGui::TextColored(ImVec4(1.0, 0.0, 0.0, 1.0), "failed");

    if (ImGui::Checkbox("IkSilderMode", &isIkSiledrMode)) {
        if (isIkSiledrMode) {
            isIkDragMode = false;
        }
    }

    ImGui::SameLine();

    if (ImGui::Checkbox("IkDragMode", &isIkDragMode)) {
        if (isIkDragMode) {
            isIkSiledrMode = false;
        }
    }

    ImGui::Checkbox("Show Grid", &showGrid);
    ImGui::SameLine();
    ImGui::Checkbox("Show Axis", &showAxis);
    ImGui::SameLine();
    ImGui::Checkbox("Show ProjectionLines", &showProjectionLines);

    
    ImGui::Separator();
    ImGui::End();
}

void SceneRobotLayer::convertPath(char *path)
{
    while (*path != '\0') {
        if (*path == '\\') {
            *path = '/';
        }
        path++;
    }
}

glm::mat4 SceneRobotLayer::toGlmMatrix(const KDL::Frame& frame) {
    glm::mat4 result(1.0f);  // 初始化为单位矩阵

    for (int i = 0; i < 3; ++i) {
        result[0][i] = frame.M(i, 0);  // X轴方向向量
        result[1][i] = frame.M(i, 1);  // Y轴方向向量
        result[2][i] = frame.M(i, 2);  // Z轴方向向量
        result[3][i] = frame.p[i];     // 平移部分
    }

    return result;
}

KDL::Frame SceneRobotLayer::fromGlmMatrix(const glm::mat4& mat) {
    KDL::Rotation R(mat[0][0], mat[1][0], mat[2][0],  // X轴
                    mat[0][1], mat[1][1], mat[2][1],  // Y轴
                    mat[0][2], mat[1][2], mat[2][2]); // Z轴

    KDL::Vector p(mat[3][0], mat[3][1], mat[3][2]);    // 平移向量

    return KDL::Frame(R, p);
}
