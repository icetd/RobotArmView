#include "RuckigController.h"
#include <iostream>

RuckigController::RuckigController(size_t dof, double controlCycle) :
    m_dof(dof), 
    m_dt(controlCycle)
{
    m_input = std::make_unique<ruckig::InputParameter<ruckig::DynamicDOFs>>(m_dof);
    m_output = std::make_unique<ruckig::OutputParameter<ruckig::DynamicDOFs>>(m_dof);
    m_otg = std::make_unique<ruckig::Ruckig<ruckig::DynamicDOFs>>(m_dof, m_dt);
    m_active = false;
}

// not yet implemented waypoints mode
RuckigController::RuckigController(size_t dof, double controlCycle, size_t max_number_of_waypoints) :
    m_dof(dof), 
    m_dt(controlCycle), 
    m_max_number_of_waypoints(max_number_of_waypoints)
{
    m_input = std::make_unique<ruckig::InputParameter<ruckig::DynamicDOFs>>(m_dof, m_max_number_of_waypoints);
    m_output = std::make_unique<ruckig::OutputParameter<ruckig::DynamicDOFs>>(m_dof, m_max_number_of_waypoints);
    m_otg = std::make_unique<ruckig::Ruckig<ruckig::DynamicDOFs>>(m_dof, m_dt, m_max_number_of_waypoints);
    m_active = false;
}

void RuckigController::SetLimits(const std::vector<double> &maxVel, const std::vector<double> &maxAcc, const std::vector<double> &maxJerk)
{
    for (int i = 0; i < m_dof; ++i) {
        m_input->max_velocity[i] = maxVel[i];
        m_input->max_acceleration[i] = maxAcc[i];
        m_input->max_jerk[i] = maxJerk[i];
    }
}

void RuckigController::SetTarget(const std::vector<double> &targetPosition)
{
    if (!m_getState) {
        std::cerr << "State getter not set.\n";
        return;
    }

    std::vector<double> pos(m_dof), vel(m_dof);
    m_getState(pos, vel);

    for (int i = 0; i < m_dof; ++i) {
        m_input->current_position[i] = pos[i];
        m_input->current_velocity[i] = vel[i];
        m_input->current_acceleration[i] = 0.0;

        m_input->target_position[i] = targetPosition[i];
        m_input->target_velocity[i] = 0.0;
        m_input->target_acceleration[i] = 0.0;
    }

    m_active = true;
}

void RuckigController::SetGetStateFunction(std::function<void(std::vector<double> &, std::vector<double> &)> getter)
{
    m_getState = getter;
}

void RuckigController::SetSendCommandFunction(std::function<void(const std::vector<double> &)> sender)
{
    m_sendCommand = sender;
}

void RuckigController::run()
{
    while (!this->isStoped()) {
        if (m_active && m_sendCommand) {
            auto result = m_otg->update(*m_input, *m_output);
            if (result == ruckig::Result::Error) {
                std::cerr << "[RuckigController] Ruckig Error\n";
                m_active = false;
                continue;
            }

            std::vector<double> newPos(m_dof);
            for (int i = 0; i < m_dof; ++i) {
                newPos[i] = m_output->new_position[i];
            }

            m_sendCommand(newPos);

            for (int i = 0; i < m_dof; ++i) {
                m_input->current_position[i] = m_output->new_position[i];
                m_input->current_velocity[i] = m_output->new_velocity[i];
                m_input->current_acceleration[i] = m_output->new_acceleration[i];
            }

            if (result == ruckig::Result::Finished) {
                m_active = false;
            }
        }

        // 控制周期 sleep
        std::this_thread::sleep_for(std::chrono::duration<double>(m_dt));
    }
}

bool RuckigController::IsFinished() const
{
    return !m_active;
}