#ifndef SCREENVIEW_HPP
#define SCREENVIEW_HPP

#include <gui_generated/screen_screen/screenViewBase.hpp>
#include <gui/screen_screen/screenPresenter.hpp>
#include <gui/containers/Snake.hpp>
#include <gui/containers/Joystick.hpp>
#include <gui/containers/Food.hpp>

class screenView : public screenViewBase
{
public:
    screenView();
    virtual ~screenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

    float getDirectionAngle ( void );
protected:

    Snake snake;
    Joystick joystick;
    Food food;

    Callback< screenView >SnakeEatCallback;
    void SnakeEatHandler ( );

    Callback< screenView, const float& >JoystickChangeCallback;
    void JoystickChangeHandler ( const float &angle );

    Callback< screenView, int16_t&, int16_t& >FoodChangeCallback;
    void FoodChangeHandler ( int16_t &cx, int16_t &cy );
};

#endif // SCREENVIEW_HPP
