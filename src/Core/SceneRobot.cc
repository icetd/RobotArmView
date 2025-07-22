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
    m_girdRender = new Renderer(m_verticesGird, m_indicesGird, m_emptyTexture);

    GenerateAxis(0.2);
    m_axisRender = new Renderer(m_verticesAxis, m_indicesAxis, m_emptyTexture);

    GenerateXYZ(0.2);
    m_XYZRender = new Renderer(m_verticesXYZ, m_indicesXYZ, m_emptyTexture);
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
        m_axisRender->DrawTriangle(shader, GL_FILL);
        m_XYZRender->DrawLine(shader);
    }
}


// 生成圆柱体，沿 Y 轴正方向，底部中心在 (0,0,0)
static void GenerateCylinder(float radius, float height, int segments,
                      std::vector<Vertex> &vertices, std::vector<unsigned int> &indices,
                      glm::vec3 color, glm::vec3 offset = glm::vec3(0))
{
    int baseIndex = vertices.size();

    for (int i = 0; i <= segments; ++i) {
        float theta = 2.0f * M_PI * i / segments;
        float x = radius * cos(theta);
        float z = radius * sin(theta);
        vertices.push_back(Vertex{glm::vec3(x, 0, z) + offset, color});
        vertices.push_back(Vertex{glm::vec3(x, height, z) + offset, color});
    }

    for (int i = 0; i < segments; ++i) {
        int i0 = baseIndex + i * 2;
        int i1 = i0 + 1;
        int i2 = baseIndex + ((i + 1) % segments) * 2;
        int i3 = i2 + 1;

        // 两个三角形构成侧面四边形
        indices.push_back(i0);
        indices.push_back(i2);
        indices.push_back(i1);

        indices.push_back(i1);
        indices.push_back(i2);
        indices.push_back(i3);
    }
}

// 生成圆锥体，底面中心在 (0,0,0)，高沿 Y 轴正方向
static void GenerateCone(float radius, float height, int segments,
                  std::vector<Vertex> &vertices, std::vector<unsigned int> &indices,
                  glm::vec3 color, glm::vec3 offset = glm::vec3(0))
{
    int baseIndex = vertices.size();

    // 底面圆心顶点
    vertices.push_back(Vertex{offset, color});
    // 底面圆周顶点
    for (int i = 0; i <= segments; ++i) {
        float theta = 2.0f * M_PI * i / segments;
        float x = radius * cos(theta);
        float z = radius * sin(theta);
        vertices.push_back(Vertex{glm::vec3(x, 0, z) + offset, color});
    }
    // 顶点尖端
    vertices.push_back(Vertex{glm::vec3(0, height, 0) + offset, color});

    // 底面索引（三角扇）
    for (int i = 1; i <= segments; ++i) {
        indices.push_back(baseIndex);
        indices.push_back(baseIndex + i);
        indices.push_back(baseIndex + i + 1);
    }

    // 侧面三角形
    int tipIndex = baseIndex + segments + 2;
    for (int i = 1; i <= segments; ++i) {
        indices.push_back(tipIndex);
        indices.push_back(baseIndex + i + 1);
        indices.push_back(baseIndex + i);
    }
}

void SceneRobot::GenerateAxis(float length)
{
    m_verticesAxis.clear();
    m_indicesAxis.clear();


    const int segments = 32;            // 圆柱和圆锥圆周细分段数，决定圆的平滑度，值越大越圆滑
    float shaftRadius = length * 0.02f; // 箭头圆柱（箭杆）半径，取坐标轴长度的 2%
    float shaftLength = length * 0.8f;  // 箭头圆柱（箭杆）长度，占坐标轴总长度的 80%
    float headLength = length * 0.15f;  // 箭头圆锥（箭头尖端）长度，占坐标轴总长度的 10%
    glm::vec3 colorX(0.8f, 0.2f, 0.2f);
    glm::vec3 colorY(0.2f, 0.8f, 0.2f);
    glm::vec3 colorZ(0.2f, 0.2f, 0.8f);

    // X轴箭头，先生成Y轴方向圆柱和圆锥，最后旋转到X轴方向
    std::vector<Vertex> tempVertices;
    std::vector<unsigned int> tempIndices;

    // 圆柱 + 圆锥沿 Y 轴正方向生成（长度分配）
    GenerateCylinder(shaftRadius, shaftLength, segments, tempVertices, tempIndices, colorX);
    GenerateCone(shaftRadius * 2.0f, headLength, segments, tempVertices, tempIndices, colorX, glm::vec3(0, shaftLength, 0));

    // 旋转到 X 轴方向（绕 Z 轴负90度）
    glm::mat4 rot = glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(0, 0, 1));

    int baseIndex = m_verticesAxis.size();
    for (auto &v : tempVertices) {
        glm::vec3 p = glm::vec3(rot * glm::vec4(v.position, 1.0f));
        m_verticesAxis.push_back(Vertex{p, v.color});
    }
    for (auto idx : tempIndices) {
        m_indicesAxis.push_back(baseIndex + idx);
    }

    // 清空临时容器，生成 Y 轴箭头（不用旋转）
    tempVertices.clear();
    tempIndices.clear();
    GenerateCylinder(shaftRadius, shaftLength, segments, tempVertices, tempIndices, colorY);
    GenerateCone(shaftRadius * 2.0f, headLength, segments, tempVertices, tempIndices, colorY,
                 glm::vec3(0, shaftLength, 0));
    baseIndex = m_verticesAxis.size();
    for (auto &v : tempVertices) {
        m_verticesAxis.push_back(v);
    }
    for (auto idx : tempIndices) {
        m_indicesAxis.push_back(baseIndex + idx);
    }

    // 生成 Z 轴箭头（绕 X 轴正90度）
    tempVertices.clear();
    tempIndices.clear();
    GenerateCylinder(shaftRadius, shaftLength, segments, tempVertices, tempIndices, colorZ);
    GenerateCone(shaftRadius * 2.0f, headLength, segments, tempVertices, tempIndices, colorZ,
                 glm::vec3(0, shaftLength, 0));
    rot = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1, 0, 0));
    baseIndex = m_verticesAxis.size();
    for (auto &v : tempVertices) {
        glm::vec3 p = glm::vec3(rot * glm::vec4(v.position, 1.0f));
        m_verticesAxis.push_back(Vertex{p, v.color});
    }
    for (auto idx : tempIndices) {
        m_indicesAxis.push_back(baseIndex + idx);
    }

    m_axisRender = new Renderer(m_verticesAxis, m_indicesAxis, m_emptyTexture);
}

void SceneRobot::GenerateXYZ(float length)
{
    const float charWidth = length / 30.0f;
    const float charHeight = length / 20.0f;
    const float charShift = 1.04f * length;

    // Define colors for each axis
    glm::vec3 color(0.8f, 0.8f, 0.8f);

    // X-axis char
    m_verticesXYZ.push_back(Vertex{glm::vec3(charShift, charWidth, -charHeight), color});
    m_verticesXYZ.push_back(Vertex{glm::vec3(charShift, -charWidth, charHeight), color});
    m_verticesXYZ.push_back(Vertex{glm::vec3(charShift, -charWidth, -charHeight), color});
    m_verticesXYZ.push_back(Vertex{glm::vec3(charShift, charWidth, charHeight), color});

    // Y-axis char
    m_verticesXYZ.push_back(Vertex{glm::vec3(charWidth, charShift, charHeight), color});
    m_verticesXYZ.push_back(Vertex{glm::vec3(0.0f, charShift, 0.0f), color});
    m_verticesXYZ.push_back(Vertex{glm::vec3(-charWidth, charShift, charHeight), color});
    m_verticesXYZ.push_back(Vertex{glm::vec3(0.0f, charShift, 0.0f), color});
    m_verticesXYZ.push_back(Vertex{glm::vec3(0.0f, charShift, 0.0f), color});
    m_verticesXYZ.push_back(Vertex{glm::vec3(0.0f, charShift, -charHeight), color});

    // Z-axis char
    m_verticesXYZ.push_back(Vertex{glm::vec3(-charWidth, charHeight, charShift), color});
    m_verticesXYZ.push_back(Vertex{glm::vec3(charWidth, charHeight, charShift), color});
    m_verticesXYZ.push_back(Vertex{glm::vec3(charWidth, charHeight, charShift), color});
    m_verticesXYZ.push_back(Vertex{glm::vec3(-charWidth, -charHeight, charShift), color});
    m_verticesXYZ.push_back(Vertex{glm::vec3(-charWidth, -charHeight, charShift), color});
    m_verticesXYZ.push_back(Vertex{glm::vec3(charWidth, -charHeight, charShift), color});

    for (int i = 0; i < m_verticesXYZ.size(); i++) {
        m_indicesXYZ.push_back(i);
    }

    m_XYZRender = new Renderer(m_verticesXYZ, m_indicesXYZ, m_emptyTexture);
}

#if 0  // old axis and text
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
#endif