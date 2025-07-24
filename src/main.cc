//#pragma comment(linker, "/subsystem:windows /ENTRY:mainCRTStartup")
#include <iostream>
#include <string>
#include <fstream>
#include "Core/Application.h"
#include "Core/log.h"
#include "Ui/MainLayer.h"
#include "Ui/SceneRobotLayer.h"
#include "Ui/LogLayer.h"
#include "Graphics/Objects/Robot.h"

#define VLD 0

#if VLD
#include <vld.h>
#endif

int main(int argc, char **argv)
{
    initLogger(ERRO);
    Application *app = new Application("RobotView", 1920, 1080);
    app->PushLayer<MainLayer>();
    app->PushLayer<SceneRobotLayer>();
    app->PushLayer<LogLayer>();
    app->Run();

    delete app;
    return 0;
}