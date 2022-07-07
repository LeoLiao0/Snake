#include <gui/containers/Joystick.hpp>
#include <math.h>

Joystick::Joystick():
CircleClickedCallback( this, &Joystick::CircleClickedHandler )
{
	JoyStickCahngeCallback = NULL;
}

void Joystick::initialize()
{
    JoystickBase::initialize();

    circle_background.getRadius( radius );

    cx = circle_background.getX() + radius;
    cy = circle_background.getY() + radius;

    //circle_background.setClickAction( CircleClickedCallback );

    angle = 0;
    isClicked = false;

    setTouchable( true );
}

void Joystick::handleClickEvent(const touchgfx::ClickEvent& evt)
{
	if( evt.getType() == ClickEvent::RELEASED )
	{
		float r;
		angle = 0;
		isClicked = false;

		circle_controller.getRadius( r );
		circle_controller.moveTo( (cx - r), (cy - r) );
	}
}

void Joystick::handleDragEvent(const touchgfx::DragEvent& evt)
{
//	if( isClicked == false )
//		return;

	float r;
	float dist = sqrt( (evt.getNewX() - cx) * (evt.getNewX() - cx) + (evt.getNewY() - cy) * (evt.getNewY() - cy) );
	angle = atan2( (float)(evt.getNewY() - cy), (float)(evt.getNewX() - cx) );

	circle_controller.getRadius( r );

	if( dist > radius )
	{
		int16_t new_x = cx + radius * cos( angle );
		int16_t new_y = cy + radius * sin( angle );
		circle_controller.moveTo( (new_x - r), (new_y - r) );
	}
	else
	{
		circle_controller.moveTo( (evt.getNewX() - r), (evt.getNewY() - r) );
	}

	if( (JoyStickCahngeCallback != NULL) && (JoyStickCahngeCallback->isValid()) )
		JoyStickCahngeCallback->execute( angle );
}

void Joystick::CircleClickedHandler ( const Circle &c, const ClickEvent &e )
{
	if( &c == &circle_background )
	{
		if( e.getType() == ClickEvent::PRESSED )
			isClicked = true;
		else if( e.getType() == ClickEvent::RELEASED )
			isClicked = false;
	}
}

