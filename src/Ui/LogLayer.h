#ifndef LOG_LAYER_H
#define LOG_LAYER_H

#include "../Core/Layer.h"
#include "../Utils/Utils.h"

class LogLayer : public Layer
{
protected:
    virtual void OnAttach() override;
    virtual void OnUpdate(float ts) override;
    virtual void OnDetach() override;
    virtual void OnUIRender() override;

private:
    void ShowFeedbackLog();
    AppLog m_log;
};

#endif