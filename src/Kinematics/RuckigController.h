#ifndef RUCKIG_CONTROLLER_H
#define RUCKIG_CONTROLLER_H

#include <ruckig/ruckig.hpp>
#include <vector>
#include <functional>
#include <Core/MThread.h>

class RuckigController : public MThread
{
public:
    RuckigController(int dof, double controlCycle);

    void SetTarget(const std::vector<double> &targePosition);
    void SetLimits(const std::vector<double> &maxVel, const std::vector<double> &maxAcc, const std::vector<double> &maxJerk);

    void SetGetStateFunction(std::function<void(std::vector<double> &pos, std::vector<double> &vel)> getter);
    void SetSendCommandFunction(std::function<void(const std::vector<double> &pos)> sender);

    void run() override;

    bool IsFinished() const;

private:
    int m_dof;
    double m_dt;

    ruckig::InputParameter<6> m_input;
    ruckig::OutputParameter<6> m_output;
    ruckig::Ruckig<6> m_otg;

    bool m_active = false;

    std::function<void(std::vector<double> &pos, std::vector<double> &vel)> m_getState;
    std::function<void(const std::vector<double> &pos)> m_sendCommand;
};

#endif