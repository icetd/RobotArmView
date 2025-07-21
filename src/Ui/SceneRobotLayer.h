#ifndef SCENEROBOT_LAYER_H
#define SCENEROBOT_LAYER_H

#include "../Core/Layer.h"
#include "../Core/SceneRobot.h"
#include "../Graphics/Buffers/FrameBuffer.h"
#include "../Graphics/Objects/Robot.h"
#include "../Graphics/Renderer/Camera.h"
#include "../Graphics/Renderer/Renderer.h"
#include "../Graphics/Renderer/Shader.h"
#include "Kinematics/KDLKinematics.h"
#include "Kinematics/kdl_parser.h"

class SceneRobotLayer : public Layer
{
public:
protected:
    virtual void OnAttach() override;
    virtual void OnUpdate(float ts) override;
    virtual void OnDetach() override;
    virtual void OnUIRender() override;

private:
    glm::mat4 m_proj = glm::mat4(1.0f);
    glm::mat4 m_view = glm::mat4(1.0f);

    Camera *m_camera;
    FrameBuffer *m_frameBuffer;

    Shader *m_TextureLightShader;
    Shader *m_TextureShader;
    Shader *m_ColorLightShader;
    Shader *m_ColorShader;

    float m_width;
    float m_height;
    int m_selected = 0;

    Robot *m_robot;
    SceneRobot *m_senceRobot;
    bool showGrid;
    bool showAxis;

    void ShowModelSence();
    void ShowModelLoad();
    void convertPath(char *path);
    
    const double deg2rad = 0.017453292519943295769236907684886127; // PI/180
    const double rad2deg = 57.29577951308232087679815481410517033; // 180/PI

    std::unique_ptr<KDLKinematics> m_kinematics;
    // IK 控制目标
    float targetX = 0, targetY = 0, targetZ = 0;
    float targetRoll = 0, targetPitch = 0, targetYaw = 0;
};

#endif