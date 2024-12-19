#ifndef SENCEROBOT_H
#define SENCEROBOT_H

#include "../Graphics/Objects/Robot.h"

class SceneRobot
{
public:
    SceneRobot(Robot *robot);
    ~SceneRobot();

    static SceneRobot *GetInstance();

    void UpdateStatus(Shader &shader, Camera &camera);
    void DrawGrid(Shader &shader, Camera &camera);
    void DrawAxis(Shader &shader, Camera &camera);

private:
    float m_platfromSize = 100.0f;
    int slices = m_platfromSize;
    glm::mat4 m_modelPlatform = glm::mat4(1.0f);
    
    std::vector<Texture> m_emptyTexture;
    std::vector<Vertex> m_verticesGird;
    std::vector<GLuint> m_indicesGird;
    Renderer *m_girdRender;
    void GenerateGrid(int rows, int cols, float spacing, glm::vec3 color);

    std::vector<Vertex> m_verticesAxis;
    std::vector<GLuint> m_indicesAxis;
    Renderer *m_axisRender;
    void GenerateAxis(float length);

    Robot *m_robot;
};

#endif