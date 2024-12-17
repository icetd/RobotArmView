#include "SceneRobot.h"

#include <stb_image.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

static SceneRobot *Instance = nullptr;

SceneRobot::SceneRobot(Robot *robot)
    : m_robot(robot)
{
    Instance = this;
}

SceneRobot::~SceneRobot()
{
}

SceneRobot *SceneRobot::GetInstance() { return Instance; }

void SceneRobot::UpdateStatus(Shader &shader, Camera &camera)
{
    m_robot->updateJointAngles(shader, camera);
}