#ifndef SCREEN_SNAKEVIEW_HPP
#define SCREEN_SNAKEVIEW_HPP

#include <gui_generated/screen_snake_screen/screen_snakeViewBase.hpp>
#include <gui/screen_snake_screen/screen_snakePresenter.hpp>
#include <gui/containers/Snake.hpp>
#include <gui/containers/Joystick.hpp>
#include <gui/containers/Food.hpp>

class screen_snakeView : public screen_snakeViewBase
{
public:
    screen_snakeView();
    virtual ~screen_snakeView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

    float getDirectionAngle ( void );
protected:

    Snake snake;
    Joystick joystick;
    Food food;

    Callback< screen_snakeView >SnakeEatCallback;
    void SnakeEatHandler ( );

    Callback< screen_snakeView, const float& >JoystickChangeCallback;
    void JoystickChangeHandler ( const float &angle );

    Callback< screen_snakeView, int16_t&, int16_t& >FoodChangeCallback;
    void FoodChangeHandler ( int16_t &cx, int16_t &cy );
};

#endif // SCREEN_SNAKEVIEW_HPP
