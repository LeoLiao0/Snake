#include <gui/screen_snake_screen/screen_snakeView.hpp>

screen_snakeView::screen_snakeView():
SnakeEatCallback( this, &screen_snakeView::SnakeEatHandler ),
JoystickChangeCallback( this, &screen_snakeView::JoystickChangeHandler ),
FoodChangeCallback( this, &screen_snakeView::FoodChangeHandler )
{

}

void screen_snakeView::setupScreen()
{
    screen_snakeViewBase::setupScreen();

    snake.setMapPosition( food.getRect() );
    snake.setSnakeEatCallback( SnakeEatCallback );

    joystick.initialize( );
    joystick.setXY( 25, 395 );
    joystick.setJoyStickCahngeCallback( JoystickChangeCallback );

    food.initialize( );
    food.setNoDisplayArea( joystick.getRect() );
	food.setPositionCahngeCallback( FoodChangeCallback );
	food.setNewPosition( );

    add( snake );
    add( joystick );
    add( food );
}

void screen_snakeView::tearDownScreen()
{
    screen_snakeViewBase::tearDownScreen();

    remove( snake );
    remove( joystick );
    remove( food );
}

void screen_snakeView::SnakeEatHandler ( )
{
	food.setNewPosition( );
}

void screen_snakeView::JoystickChangeHandler ( const float &angle )
{
	snake.setDirectionAngle( (float)angle );
}

void screen_snakeView::FoodChangeHandler ( int16_t &cx, int16_t &cy )
{
	snake.setFoodPosition( cx, cy );
}
