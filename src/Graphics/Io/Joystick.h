#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <GLFW/glfw3.h>

// xbox
#define JOYSTICK_BUTTON_A 0
#define JOYSTICK_BUTTON_B 1
#define JOYSTICK_BUTTON_X 2
#define JOYSTICK_BUTTON_Y 3
#define JOYSTICK_BUTTON_LEFT_BUMPER 4
#define JOYSTICK_BUTTON_RIGHT_BUMPER 5
#define JOYSTICK_BUTTON_BACK 6
#define JOYSTICK_BUTTON_START 7
#define JOYSTICK_BUTTON_LEFT_THUMB 8
#define JOYSTICK_BUTTON_RIGHT_THUMB 9
#define JOYSTICK_BUTTON_DPAD_UP 10
#define JOYSTICK_BUTTON_DPAD_RIGHT 11
#define JOYSTICK_BUTTON_DPAD_DOWN 12
#define JOYSTICK_BUTTON_DPAD_LEFT 13

// axes
#define JOYSTICK_AXES_LEFT_STICK_X 0
#define JOYSTICK_AXES_LEFT_STICK_Y 1
#define JOYSTICK_AXES_RIGHT_STICK_X 2
#define JOYSTICK_AXES_RIGHT_STICK_Y 3
#define JOYSTICK_AXES_LEFT_TRIGGER 4
#define JOYSTICK_AXES_RIGHT_TRIGGER 5

class Joystick
{
public:
    // generate an instance for joystick with id i
    Joystick(int i);

    // update the joystick's states
    void update();

    // get axis value
    float axesState(int axis);
    // get button state
    unsigned char buttonState(int button);

    // get number of axes
    int getAxesCount();
    // get number of buttons
    int getButtonCount();

    // return if joystick present
    bool isPresent();
    // get name of joystick
    const char *getName();

    // static method to get enum value for joystick
    static int getId(int i);

private:
    // 1 if present, 0 if not
    int m_present;

    // joystick id
    int m_id;

    // joystick name
    const char *m_name;

    // number of axes on joystick
    int m_axesCount;
    // array of axes values
    const float *m_axes;

    // number of buttons
    int m_buttonCount;

    // array of button states
    const unsigned char *m_buttons;
};

#endif