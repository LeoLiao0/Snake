#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include <gui_generated/containers/JoystickBase.hpp>

class Joystick : public JoystickBase
{
public:
    Joystick();
    virtual ~Joystick() {}

    virtual void initialize();
    virtual void handleClickEvent(const touchgfx::ClickEvent& evt);
    virtual void handleDragEvent(const touchgfx::DragEvent& evt);

	void setJoyStickCahngeCallback(touchgfx::GenericCallback<const float&>& callback)
	{
		this->JoyStickCahngeCallback = &callback;
	}

protected:

    int16_t cx, cy; // center position
    float angle;
    float radius;
    bool isClicked;

    touchgfx::GenericCallback<const float&>*JoyStickCahngeCallback;

    Callback< Joystick, const Circle&, const ClickEvent& >CircleClickedCallback;
    void CircleClickedHandler ( const Circle &c, const ClickEvent &e );
};

#endif // JOYSTICK_HPP
