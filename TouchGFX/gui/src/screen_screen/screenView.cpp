#include <gui/screen_screen/screenView.hpp>

screenView::screenView():
SnakeEatCallback( this, &screenView::SnakeEatHandler ),
JoystickChangeCallback( this, &screenView::JoystickChangeHandler ),
FoodChangeCallback( this, &screenView::FoodChangeHandler )
{

}

void screenView::setupScreen()
{
    screenViewBase::setupScreen();

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

void screenView::tearDownScreen()
{
    screenViewBase::tearDownScreen();

    remove( snake );
    remove( food );
}

void screenView::SnakeEatHandler ( )
{
	food.setNewPosition( );
}

void screenView::JoystickChangeHandler ( const float &angle )
{
	snake.setDirectionAngle( (float)angle );
}

void screenView::FoodChangeHandler ( int16_t &cx, int16_t &cy )
{
	snake.setFoodPosition( cx, cy );
}
