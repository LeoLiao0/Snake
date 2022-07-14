#include <gui/containers/Snake.hpp>
#include <touchgfx/hal/HAL.hpp>
#include <BitmapDatabase.hpp>
#include <math.h>

using namespace touchgfx;

Snake::Snake():
new_Angle( 0 ),
previous_Angle( 0 ),
current_len( 0 ),
SnakeSpeed( 5 )
{
	snakeHead.init( 140, 40, BITMAP_SANKEHEAD_ID, 0 );
	snakeBodys[0].init( 100, 40, BITMAP_SNAKEBODY_ID );
	snakeBodys[1].init( 60, 40, BITMAP_SNAKEBODY_ID );

	snakeHead.update( SnakeSpeed );
	snakeBodys[0].update( SnakeSpeed );
	snakeBodys[1].update( SnakeSpeed );

	current_len = 2;

	Map = this->getRect( );

	Application::getInstance()->registerTimerWidget(this);
}

Snake::~Snake()
{
	Application::getInstance()->unregisterTimerWidget(this);
}

touchgfx::Rect Snake::getSolidRect() const
{
	return touchgfx::Rect();
}

void Snake::draw(const touchgfx::Rect& invalidatedArea) const
{
	Rect meAbs;
	translateRectToAbsolute( meAbs ); //To find our x and y coords in absolute.

	/* Paint Snake Head */
	Rect invalid = snakeHead.getRect() & invalidatedArea;
	if ( !invalid.isEmpty() )
	{
        invalid.x -= snakeHead.getX();
        invalid.y -= snakeHead.getY();
        snakeHead.draw( invalid );
	}

	/* Paint Snake Body */
	for( uint16_t count = 0 ; count < current_len ; count++ )
	{
		invalid = snakeBodys[count].getRect() & invalidatedArea;

		if ( !invalid.isEmpty() )
		{
            invalid.x -= snakeBodys[count].getX();
            invalid.y -= snakeBodys[count].getY();
            snakeBodys[count].draw( invalid );
		}
	}
}

void Snake::handleTickEvent()
{
	static int16_t previous_sx, previous_sy;

	this->invalidate();

	/* Update snake head position. */
	/* If head is move animation is active, do not set new position */
	if( snakeHead.isMoveAnimationRunning() == false )
	{
		float temp_Angle = snakeHead.directionAngle;
		snakeHead.directionAngle = new_Angle;
		previous_Angle = temp_Angle;

		/* Get next position */
		int16_t next_sx = snakeHead.sx + snakeHead.radius * 2 * cos( snakeHead.directionAngle );
		int16_t next_sy = snakeHead.sy + snakeHead.radius * 2 * sin( snakeHead.directionAngle );

		/* Detect if the snake head touches the border */
		if( detectBorder(snakeHead.sx, snakeHead.sy, snakeHead.radius ) == true )
			return;

		previous_sx = snakeHead.sx;
		previous_sy = snakeHead.sy;
		snakeHead.sx = next_sx;
		snakeHead.sy = next_sy;

		/* Update head Z angle */
		snakeHead.updateZAngle( snakeHead.directionAngle );

		/* to start move animation */
		snakeHead.update( SnakeSpeed );
	}

	/* Update snake body. */
	else
	{
		for( uint16_t count = 0 ; count < current_len ; count++ )
		{
			/* If body is move animation is active, do not set new position */
			if( snakeBodys[ count ].isMoveAnimationRunning() == true )
				continue;

			int16_t temp_sx = snakeBodys[ count ].sx;
			int16_t temp_sy = snakeBodys[ count ].sy;

			snakeBodys[ count ].sx = previous_sx;
			snakeBodys[ count ].sy = previous_sy;

			previous_sx = temp_sx;
			previous_sy = temp_sy;

			/* to start move animation */
			snakeBodys[ count ].update( SnakeSpeed );
		}
	}


	int16_t next_cx = snakeHead.sx + snakeHead.radius;
	int16_t next_cy = snakeHead.sy + snakeHead.radius;

	/* Check that the snake is touch the food, it means snake eat the food, and increase body length */
	if( isEatFood( next_cx, next_cy, food_cx, food_cy ) == true )
	{
		/* To notify the food widget to move to new position */
		if( (SnakeEatCallback != NULL) && (SnakeEatCallback->isValid()) )
		{
			SnakeEatCallback->execute( );
		}

		/* Check there is one more empty space in snake body buffer */
		if( current_len < MAX_NUMBER_OF_BODY )
		{
			/* To create and initialize new body */
			snakeBodys[ current_len ].init( previous_sx, previous_sy, BITMAP_SNAKEBODY_ID );
			snakeBodys[ current_len ].update( SnakeSpeed );
			current_len++;
		}
	}
}

bool Snake::detectBorder ( int16_t sx, int16_t sy, float radius  )
{
	int16_t Left_X = sx;
	int16_t Right_X = sy + radius * 2;
	int16_t Up_Y = sy;
	int16_t Down_Y = sy + radius * 2;

	if( (Left_X < Map.x) || (Right_X > (Map.x + Map.width)) || (Up_Y < Map.y) || (Down_Y > (Map.y + Map.height)) )
		return true;
	else
		return false;
}

void Snake::setMapPosition ( Rect rect )
{
	setPosition( rect.x, rect.y, rect.width, rect.height );
	Map = getRect();
}

void Snake::setDirectionAngle ( float angle )
{
	new_Angle = angle;
}

void Snake::setFoodPosition ( int16_t cx, int16_t cy )
{
	food_cx = cx;
	food_cy = cy;
}

bool Snake::isEatFood ( int16_t Snake_cx, int16_t Snake_cy, int16_t Food_cx, int16_t Food_cy )
{
	float dist = sqrt( (Snake_cx - Food_cx) * (Snake_cx - Food_cx) + (Snake_cy - Food_cy) * (Snake_cy - Food_cy) );
	dist = abs( dist );
	if( dist <= snakeBodys[0].radius * 2 )
		return true;
	else
		return false;
}


