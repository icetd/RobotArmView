#include "SceneRobotLayer.h"
#include "../Core/Application.h"
#include "../Core/log.h"
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
        m_kinematics = std::make_unique<KDLKinematics>(tree, m_robot->getJointObjects()[0]->name, m_robot->getJointObjects()[m_robot->getJointObjects().size() - 1]->name, urdfPath);
    }

    KDL::JntArray q(m_kinematics->getDOF());
    for (int i = 0; i < q.rows(); i++)
        q(i) = m_robot->getJointObjects()[i + 1]->getAngle() * deg2rad;

    KDL::Frame eeFrame;
    if (m_kinematics->computeFK(q, eeFrame)) {
        m_endEffectorMatrix = toGlmMatrix(eeFrame);
        m_lastIkSuccessMatrix = m_endEffectorMatrix;
    }

    // ruckigController
    size_t dof = m_robot->getJointObjects().size() - 1;
    double controlCycle = 0.01; // 10ms 控制周期

    m_ruckigController = std::make_unique<RuckigController>(dof, controlCycle);

    // 用于获取机械臂角度 (控制真实机械臂需要获取 start move 时的状态)
    m_ruckigController->SetGetStateFunction([this](std::vector<double> &pos, std::vector<double> &vel) {
        pos.clear();
        vel.clear();
        for (int i = 1; i < m_robot->getJointObjects().size(); ++i) {
            pos.push_back(m_robot->getJointObjects()[i]->getAngle() * deg2rad);
            vel.push_back(0.0);
        }
    });

    // 实时控制 (可以用于控制实际机械臂)
    m_ruckigController->SetSendCommandFunction([this](const std::vector<double> &pos) {
        for (int i = 0; i < pos.size(); ++i) {
            m_robot->getJointObjects()[i + 1]->setAngle(pos[i] * rad2deg);
        }
    });

    m_ruckigController->start();

    std::vector<double> maxVel(dof, 1.0);  // 1 rad/s
    std::vector<double> maxAcc(dof, 2.0);  // 2 rad/s^
    std::vector<double> maxJerk(dof, 5.0); // 5 rad/s^3
    m_ruckigController->SetLimits(maxVel, maxAcc, maxJerk);

    isIkDragMode = false;
    showAxis = false;
    showGrid = true;
    showProjectionLines = true;
    isOpenCollisionDetection = false;
    showAABB = false;
    m_wasUsingGizmo = false;
}

void SceneRobotLayer::OnUpdate(float ts)
{
    m_camera->UpdateProjMatrix();
    m_camera->UpdateViewMatrix();
    m_proj = m_camera->GetProjMatrix();
    m_view = m_camera->GetViewMatrix();

    m_frameBuffer->Bind();

    for (uint32_t i = 0; i < m_robot->getJointObjects().size(); i++) {
        if (showAABB)
            m_robot->getJointObjects()[i]->AABB = true;
        else
            m_robot->getJointObjects()[i]->AABB = false;
    }

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
    ShowController();
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
            ImGuizmo::SetOrthographic(false);
            ImGuizmo::SetDrawlist();
            ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, width, height);

            ImGuizmo::Manipulate(glm::value_ptr(m_view), glm::value_ptr(m_proj),
                                 ImGuizmo::OPERATION::TRANSLATE | ImGuizmo::OPERATION::ROTATE,
                                 ImGuizmo::MODE::LOCAL, glm::value_ptr(m_endEffectorMatrix));

            if (m_wasUsingGizmo && !ImGuizmo::IsUsing()) {
                // 拖动结束时，进行 IK + Ruckig 处理
                KDL::Frame targetPose = fromGlmMatrix(m_endEffectorMatrix);
                KDL::JntArray q_seed(m_kinematics->getDOF());
                for (int i = 0; i < q_seed.rows(); ++i)
                    q_seed(i) = 0.0;

                KDL::JntArray q_result;
                if (m_kinematics->computeIK(targetPose, q_seed, q_result)) {
                    if (isOpenCollisionDetection) {
                        if (simulateJointAnglesForCollisionCheck(q_result)) {
                            LOG(WARN, "Self-collision detected after IK!");
                            m_endEffectorMatrix = m_lastIkSuccessMatrix;
                        } else {
                            m_lastIkSuccessMatrix = m_endEffectorMatrix;
                            LOG(INFO, "IK success, no collision");
                        }
                    } else {
                        m_lastIkSuccessMatrix = m_endEffectorMatrix;
                    }
                } else {
                    m_endEffectorMatrix = m_lastIkSuccessMatrix;
                }
            }

            // 记录当前是否在使用 Gizmo（用于判断状态变更）
            m_wasUsingGizmo = ImGuizmo::IsUsing();
        } else {
            // 如果没开启拖动，重置状态记录
            m_wasUsingGizmo = false;
        }
    }

    ImGui::EndChild();
    ImGui::End();
}

void SceneRobotLayer::ShowModelLoad()
{
    ImGui::Begin(u8"模型加载");
    ImGui::PushItemWidth(-ImGui::GetWindowWidth() * 0.3f);
    ImGui::AlignTextToFramePadding();
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
                m_kinematics = std::make_unique<KDLKinematics>(tree, m_robot->getJointObjects()[0]->name, m_robot->getJointObjects()[m_robot->getJointObjects().size() - 1]->name, urdfPath);
            }

            KDL::JntArray q(m_kinematics->getDOF());
            for (int i = 0; i < q.rows(); i++)
                q(i) = m_robot->getJointObjects()[i + 1]->getAngle() * deg2rad;

            KDL::Frame eeFrame;
            if (m_kinematics->computeFK(q, eeFrame)) {
                m_endEffectorMatrix = toGlmMatrix(eeFrame);
                m_lastIkSuccessMatrix = m_endEffectorMatrix;
            }
        }
    }

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

    ImGui::Separator();
    ImGui::PopItemWidth();
    ImGui::End();
}

void SceneRobotLayer::ShowController()
{
    ImGui::Begin(u8"机械臂控制");
    ImGui::PushItemWidth(-ImGui::GetWindowWidth() * 0.3f);
    ImGui::AlignTextToFramePadding();
    ImVec2 contentSize = ImGui::GetContentRegionAvail();
    const float buttonWidth = contentSize.x;

    ImGui::SeparatorText("Joint Status");
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

    if (ImGui::Button("Reset All Joints", ImVec2(buttonWidth / 2, 25))) {
        for (int i = 1; i < m_robot->getJointObjects().size(); ++i) {
            ObjectStructure *jointObject = m_robot->getJointObjects()[i];
            if (jointObject->joint_type == ObjectStructure::JointType::REVOLUTE) {
                float defaultAngle = 0.0f;
                jointObject->setAngle(defaultAngle);
            }
        }
    }

    ImGui::SameLine();
    if (ImGui::Button("Solve FK", ImVec2(buttonWidth / 2, 25))) {
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

    ImGui::SeparatorText("IK Target Pose");

    ImGui::SliderFloat("Target X (mm)", &targetX, -2000, 2000);
    ImGui::SliderFloat("Target Y (mm)", &targetY, -2000, 2000);
    ImGui::SliderFloat("Target Z (mm)", &targetZ, -2000, 2000);
    ImGui::SliderFloat("Roll (deg)", &targetRoll, -180, 180);
    ImGui::SliderFloat("Pitch (deg)", &targetPitch, -180, 180);
    ImGui::SliderFloat("Yaw (deg)", &targetYaw, -180, 180);

    ImGui::SeparatorText("Trajectory Planning");

    static float vel = 1.0f;
    static float acc = 2.0f;
    static float jerk = 5.0f;

    bool updated = false;
    updated |= ImGui::SliderFloat(u8"MaxVel (rad/s)", &vel, 0.0f, 20.0f);
    updated |= ImGui::SliderFloat(u8"MaxAcc (rad/s²)", &acc, 0.0f, 20.0f);
    updated |= ImGui::SliderFloat(u8"MaxJerk (rad/s³)", &jerk, 0.0f, 50.0f);

    if (updated) {
        int dof = m_kinematics->getDOF();
        std::vector<double> maxVel(dof, vel);
        std::vector<double> maxAcc(dof, acc);
        std::vector<double> maxJerk(dof, jerk);
        m_ruckigController->SetLimits(maxVel, maxAcc, maxJerk);
    }

    ImGui::SeparatorText("Control");

    static bool ik_success;

    // === Solve IK 按钮 + 状态 ===
    if (ImGui::Button("Solve IK", ImVec2(buttonWidth, 25))) {
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
                std::vector<double> target;
                for (int i = 0; i < dof; ++i) {
                    target.push_back(q_result(i));
                }
                m_ruckigController->SetTarget(target);

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

    ImGui::Columns(2, nullptr, false); // 两列，无边线
    // === Checkbox 区块 ===
    ImGui::Checkbox("Ik Drag Mode", &isIkDragMode);
    ImGui::NextColumn();
    if (isIkDragMode) {
        ImGui::Checkbox("Enable Collision Detection", &isOpenCollisionDetection);
    } else {
        ImGui::Text(""); // 占位对齐
    }
    ImGui::NextColumn();

    ImGui::Checkbox("Show Grid", &showGrid);
    ImGui::NextColumn();
    ImGui::Checkbox("Show Axis", &showAxis);
    ImGui::NextColumn();

    ImGui::Checkbox("Show Projection Lines", &showProjectionLines);
    ImGui::NextColumn();
    ImGui::Checkbox("Show AABB", &showAABB);
    ImGui::NextColumn();

    // === 状态信息 ===

    ImGui::Text("Ik Status:");
    ImGui::NextColumn();
    (ik_success) ? ImGui::TextColored(ImVec4(0.0, 1.0, 0.0, 1.0), "IK success") : ImGui::TextColored(ImVec4(1.0, 0.0, 0.0, 1.0), "IK failed");
    ImGui::NextColumn();

    ImGui::Text("Ruckig:");
    ImGui::NextColumn();
    if (m_ruckigController->IsFinished()) {
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "Finished");
    } else {
        ImGui::TextColored(ImVec4(1, 1, 0, 1), "Running...");
    }
    ImGui::NextColumn();

    ImGui::Text("Collision:");
    ImGui::NextColumn();
    if (!m_collision) {
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "No collision");
    } else {
        ImGui::TextColored(ImVec4(1, 1, 0, 1), "Self-collision!");
    }
    ImGui::NextColumn();
    ImGui::Columns(1); // 关闭列模式

    ImGui::Separator();
    ImGui::PopItemWidth();
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

glm::mat4 SceneRobotLayer::toGlmMatrix(const KDL::Frame &frame)
{
    glm::mat4 result(1.0f); // 初始化为单位矩阵

    for (int i = 0; i < 3; ++i) {
        result[0][i] = frame.M(i, 0); // X轴方向向量
        result[1][i] = frame.M(i, 1); // Y轴方向向量
        result[2][i] = frame.M(i, 2); // Z轴方向向量
        result[3][i] = frame.p[i];    // 平移部分
    }

    return result;
}

KDL::Frame SceneRobotLayer::fromGlmMatrix(const glm::mat4 &mat)
{
    KDL::Rotation R(mat[0][0], mat[1][0], mat[2][0],  // X轴
                    mat[0][1], mat[1][1], mat[2][1],  // Y轴
                    mat[0][2], mat[1][2], mat[2][2]); // Z轴

    KDL::Vector p(mat[3][0], mat[3][1], mat[3][2]); // 平移向量

    return KDL::Frame(R, p);
}

bool SceneRobotLayer::simulateJointAnglesForCollisionCheck(const KDL::JntArray &q)
{
    // 保存当前状态
    std::vector<float> backup;
    for (int i = 1; i < m_robot->getJointObjects().size(); ++i)
        backup.push_back(m_robot->getJointObjects()[i]->getAngle());

    // 应用模拟角度
    for (int i = 0; i < q.rows(); ++i)
        m_robot->getJointObjects()[i + 1]->setAngle(q(i) * rad2deg);

    m_robot->updateJointAngles(*m_ColorLightShader, *m_camera);

    // 碰撞检测
    m_collision = m_robot->getSelfCollisionStatus();

    // 恢复原状态
    for (int i = 1; i < m_robot->getJointObjects().size(); ++i)
        m_robot->getJointObjects()[i]->setAngle(backup[i - 1]);

    m_robot->updateJointAngles(*m_ColorLightShader, *m_camera);

    return m_collision;
}