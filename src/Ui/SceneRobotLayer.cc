#include "SceneRobotLayer.h"
#include "../Core/Application.h"
#include "../Core/Log.h"
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

    m_camera = new Camera(eye, lookat, upVector, width, height);
    Application::GetInstance()->InitCamera(m_camera, m_frameBuffer);

    m_robot = new Robot();
    m_senceRobot = new SceneRobot(m_robot);

    m_robot->loadURDF("res/robot/abb_irb2400_support/urdf/", "res/robot/abb_irb2400_support/urdf/irb2400.urdf");

    showAxis = true;
    showGrid = true;
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

void SceneRobotLayer::OnUIRender() {}

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
    }
    ImGui::EndChild();
    ImGui::End();
}

void SceneRobotLayer::ShowModelLoad()
{
    ImGui::Begin(u8"模型加载");
    ImVec2 contentSize = ImGui::GetContentRegionAvail();
    const float buttonWidth = contentSize.x;
    if (ImGui::Button(u8"add urdf", ImVec2(buttonWidth, 25)))
    {
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
        if (GetOpenFileName(&ofn) == TRUE)
        {
            char *output = new char[strlen(szFile) + 1];
            strcpy(output, szFile);
            convertPath(output);

            ObjectStructure *obj = new ObjectStructure();
            obj->path = output;

            char *token = NULL;
            char *name = NULL;
            const char *delimeter = "\\";
            token = strtok(output, delimeter);
            while (token != NULL)
            {
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
            float minAngle = jointObject->limitAngle.lower_angle;
            float maxAngle = jointObject->limitAngle.upper_angle;

            if (ImGui::SliderFloat(("Joint " + std::to_string(i) + " Angle").c_str(), &angle, minAngle, maxAngle)) {
                jointObject->setAngle(angle); // 更新关节的角度
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
    ImGui::Checkbox("Show Grid", &showGrid);
    ImGui::Checkbox("Show Axis", &showAxis);

    ImGui::Separator();
    ImGui::End();
}

void SceneRobotLayer::convertPath(char *path)
{
    while (*path != '\0')
    {
        if (*path == '\\') {
            *path = '/';
        }
        path++;
    }
}
