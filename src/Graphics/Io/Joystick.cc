#include "Joystick.h"

// generate an instance for joystick with id i
Joystick::Joystick(int i)
{
    m_id = getId(i);
    update();
}

// update the joystick's states
void Joystick::update()
{
    m_present = glfwJoystickPresent(m_id);

    if (m_present) {
        m_name = glfwGetJoystickName(m_id);
        m_axes = glfwGetJoystickAxes(m_id, &m_axesCount);
        m_buttons = glfwGetJoystickButtons(m_id, &m_buttonCount);
    }
}

// get axis value
float Joystick::axesState(int axis)
{
    if (m_present) {
        return m_axes[axis];
    }

    return -1;
}

// get button state
unsigned char Joystick::buttonState(int button)
{
    if (m_present) {
        return m_buttons[button];
    }

    return GLFW_RELEASE;
}

// get number of axes
int Joystick::getAxesCount()
{
    return m_axesCount;
}

// get number of buttons
int Joystick::getButtonCount()
{
    return m_buttonCount;
}

// return if joystick present
bool Joystick::isPresent()
{
    return m_present;
}

// get name of joystick
const char *Joystick::getName()
{
    return m_name;
}

// static method to get enum value for joystick
int Joystick::getId(int i)
{
    return GLFW_JOYSTICK_1 + i;
}