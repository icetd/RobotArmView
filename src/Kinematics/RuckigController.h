#ifndef RUCKIG_CONTROLLER_H
#define RUCKIG_CONTROLLER_H

#include <ruckig/ruckig.hpp>
#include <vector>
#include <functional>
#include <Core/MThread.h>

class RuckigController : public MThread
{
public:
    RuckigController(size_t dof, double controlCycle);
    RuckigController(size_t dof, double controlCycle, size_t max_number_of_waypoints);

    void SetTarget(const std::vector<double> &targePosition);
    void SetLimits(const std::vector<double> &maxVel, const std::vector<double> &maxAcc, const std::vector<double> &maxJerk);

    void SetGetStateFunction(std::function<void(std::vector<double> &pos, std::vector<double> &vel)> getter);
    void SetSendCommandFunction(std::function<void(const std::vector<double> &pos)> sender);
    void SetDoneCallback(std::function<void()> cb);

    void run() override;

    bool IsFinished() const;

private:
    size_t m_dof;
    size_t m_max_number_of_waypoints;
    double m_dt;

    std::unique_ptr<ruckig::InputParameter<ruckig::DynamicDOFs>> m_input;
    std::unique_ptr<ruckig::OutputParameter<ruckig::DynamicDOFs>> m_output;
    std::unique_ptr<ruckig::Ruckig<ruckig::DynamicDOFs>> m_otg;

    bool m_active = false;

    std::function<void(std::vector<double> &pos, std::vector<double> &vel)> m_getState;
    std::function<void(const std::vector<double> &pos)> m_sendCommand;
    std::function<void()> m_doneCallback;
};

#endif