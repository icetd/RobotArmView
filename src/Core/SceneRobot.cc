#include "SceneRobot.h"

#include <stb_image.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

static SceneRobot *Instance = nullptr;

SceneRobot::SceneRobot(Robot *robot) :
    m_robot(robot)
{
    Instance = this;

    GenerateGrid(slices, slices, 1, glm::vec3(0.1f, 0.1f, 0.1f));
    m_girdRender = new Renderer(m_verticesGird, m_indicesGird, m_emptyTexture); // reuse

    GenerateAxis(0.3);
    m_axisRender = new Renderer(m_verticesAxis, m_indicesAxis, m_emptyTexture); // reuse
}

SceneRobot::~SceneRobot()
{
}

SceneRobot *SceneRobot::GetInstance()
{
    return Instance;
}

void SceneRobot::UpdateStatus(Shader &shader, Camera &camera)
{
    m_robot->updateJointAngles(shader, camera);
}

void SceneRobot::DrawGrid(Shader &shader, Camera &camera)
{
    shader.Bind();
    shader.SetUniformMat4f("model", m_modelPlatform);
    shader.SetUniformMat4f("projection", camera.GetProjMatrix());
    shader.SetUniformMat4f("view", camera.GetViewMatrix());
    shader.SetUniformVec3f("camPos", camera.GetEye());
    m_girdRender->DrawLine(shader);
}

void SceneRobot::GenerateGrid(int rows, int cols, float spacing, glm::vec3 color)
{
    // Calculate the center position of the grid
    float centerX = -0.5f * cols * spacing;
    float centerY = -0.5f * rows * spacing;

    // Calculate the number of vertices and indices
    int numVertices = (rows + 1) * (cols + 1);
    int numIndices = rows * cols * 6;

    // Reserve space for vertices and indices
    m_verticesGird.reserve(numVertices);
    m_indicesGird.reserve(numIndices);

    // Generate vertex data
    for (int i = 0; i <= rows; ++i) {
        for (int j = 0; j <= cols; ++j) {
            float x = centerX + static_cast<float>(j) * spacing;
            float y = centerY + static_cast<float>(i) * spacing;
            float z = 0.0f; // You can adjust the height of the grid
            m_verticesGird.push_back(Vertex{glm::vec3(x, y, z), color,
                                            glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f)});
        }
    }

    // Generate index data
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            int topLeft = i * (cols + 1) + j;
            int topRight = topLeft + 1;
            int bottomLeft = (i + 1) * (cols + 1) + j;
            int bottomRight = bottomLeft + 1;

            // Add indices for two triangles
            m_indicesGird.push_back(topLeft);
            m_indicesGird.push_back(topRight);
            m_indicesGird.push_back(bottomLeft);

            m_indicesGird.push_back(bottomLeft);
            m_indicesGird.push_back(topRight);
            m_indicesGird.push_back(bottomRight);
        }
    }
}

void SceneRobot::DrawAxis(Shader &shader, Camera &camera)
{
    shader.Bind();
    for (auto jointsObj : m_robot->getJointObjects()) {
        shader.SetUniformMat4f("model", m_robot->calTransMat(jointsObj));
        shader.SetUniformMat4f("projection", camera.GetProjMatrix());
        shader.SetUniformMat4f("view", camera.GetViewMatrix());
        shader.SetUniformVec3f("camPos", camera.GetEye());
        m_axisRender->DrawLine(shader);
    }
}

void SceneRobot::GenerateAxis(float length)
{
    const float charWidth = length / 30.0f;
    const float charHeight = length / 20.0f;
    const float charShift = 1.04f * length;

    // Define colors for each axis
    glm::vec3 color_x(0.8f, 0.2f, 0.2f); // Red for X-axis
    glm::vec3 color_y(0.2f, 0.8f, 0.2f); // Green for Y-axis
    glm::vec3 color_z(0.2f, 0.2f, 0.8f); // Blue for Z-axis

    // X-axis vertices and indices
    m_verticesAxis.push_back(Vertex{glm::vec3(0.0f, 0.0f, 0.0f), color_x});   // Red
    m_verticesAxis.push_back(Vertex{glm::vec3(length, 0.0f, 0.0f), color_x}); // Red

    // Y-axis vertices and indices
    m_verticesAxis.push_back(Vertex{glm::vec3(0.0f, 0.0f, 0.0f), color_y});   // Green
    m_verticesAxis.push_back(Vertex{glm::vec3(0.0f, length, 0.0f), color_y}); // Green

    // Z-axis vertices and indices
    m_verticesAxis.push_back(Vertex{glm::vec3(0.0f, 0.0f, 0.0f), color_z});   // Blue
    m_verticesAxis.push_back(Vertex{glm::vec3(0.0f, 0.0f, length), color_z}); // Blue

    // X-axis char
    m_verticesAxis.push_back(Vertex{glm::vec3(charShift, charWidth, -charHeight), color_x});
    m_verticesAxis.push_back(Vertex{glm::vec3(charShift, -charWidth, charHeight), color_x});
    m_verticesAxis.push_back(Vertex{glm::vec3(charShift, -charWidth, -charHeight), color_x});
    m_verticesAxis.push_back(Vertex{glm::vec3(charShift, charWidth, charHeight), color_x});

    // Y-axis char
    m_verticesAxis.push_back(Vertex{glm::vec3(charWidth, charShift, charHeight), color_y});
    m_verticesAxis.push_back(Vertex{glm::vec3(0.0f, charShift, 0.0f), color_y});
    m_verticesAxis.push_back(Vertex{glm::vec3(-charWidth, charShift, charHeight), color_y});
    m_verticesAxis.push_back(Vertex{glm::vec3(0.0f, charShift, 0.0f), color_y});
    m_verticesAxis.push_back(Vertex{glm::vec3(0.0f, charShift, 0.0f), color_y});
    m_verticesAxis.push_back(Vertex{glm::vec3(0.0f, charShift, -charHeight), color_y});

    // Z-axis char
    m_verticesAxis.push_back(Vertex{glm::vec3(-charWidth, charHeight, charShift), color_z});
    m_verticesAxis.push_back(Vertex{glm::vec3(charWidth, charHeight, charShift), color_z});
    m_verticesAxis.push_back(Vertex{glm::vec3(charWidth, charHeight, charShift), color_z});
    m_verticesAxis.push_back(Vertex{glm::vec3(-charWidth, -charHeight, charShift), color_z});
    m_verticesAxis.push_back(Vertex{glm::vec3(-charWidth, -charHeight, charShift), color_z});
    m_verticesAxis.push_back(Vertex{glm::vec3(charWidth, -charHeight, charShift), color_z});

    for (int i = 0; i < m_verticesAxis.size(); i++) {
        m_indicesAxis.push_back(i);
    }

    m_axisRender = new Renderer(m_verticesAxis, m_indicesAxis, m_emptyTexture);
}