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

private:
    Robot *m_robot;
};

#endif