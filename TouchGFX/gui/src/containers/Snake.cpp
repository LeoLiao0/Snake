#include <gui/containers/Snake.hpp>
#include <touchgfx/hal/HAL.hpp>
#include <BitmapDatabase.hpp>
#include <math.h>

using namespace touchgfx;

void SnakeBody::init( int cx, int cy, BitmapId bmp )
{
	//SnakeHead.setXY( sx, sy );
	setBitmap( touchgfx::Bitmap(bmp) );
	setWidth(touchgfx::Bitmap(bmp).getWidth());
	setHeight(touchgfx::Bitmap(bmp).getHeight());
	setBitmapPosition(0.000f, 0.000f);
	setScale(1.000f);
	setCameraDistance(1000.000f);
	setOrigo((touchgfx::Bitmap(bmp).getWidth() / 2), (touchgfx::Bitmap(bmp).getHeight() / 2), 1000.000f);
	setCamera((touchgfx::Bitmap(bmp).getWidth() / 2), (touchgfx::Bitmap(bmp).getHeight() / 2));
	updateAngles(0.000f, 0.000f, 0.000f);
	setRenderingAlgorithm( touchgfx::TextureMapper::BILINEAR_INTERPOLATION );
	setVisible( true );

	this->cx = cx;
	this->cy = cy;
	this->sx = cx - touchgfx::Bitmap(bmp).getWidth() / 2;
	this->sy = cy - touchgfx::Bitmap(bmp).getHeight() / 2;

	radius = touchgfx::Bitmap(bmp).getWidth() / 2;
	alpha = 255;

	setAlpha( alpha );
}

void SnakeBody::update( int16_t speed )
{
	//startMoveAnimation(sx, sy, speed, EasingEquations::elasticEaseInOut, EasingEquations::linearEaseNone);
	moveTo( sx, sy );
}

Snake::Snake():
directionAngle( 0 ),
current_len( 0 ),
speed( 150 )
{
	SnakeBodys[0].init( 120, 40, BITMAP_SANKEHEAD_ID );
	SnakeBodys[1].init( 80, 40, BITMAP_SNAKEBODY_ID );
	SnakeBodys[2].init( 40, 40, BITMAP_SNAKEBODY_ID );

	SnakeBodys[0].update( speed );
	SnakeBodys[1].update( speed );
	SnakeBodys[2].update( speed );

	current_len = 3;

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

	for( uint16_t count = 0 ; count < current_len ; count++ )
	{
		Rect invalid = SnakeBodys[count].getRect() & invalidatedArea;

		if ( !invalid.isEmpty() )
		{
            invalid.x -= SnakeBodys[count].getX();
            invalid.y -= SnakeBodys[count].getY();
			SnakeBodys[count].draw( invalid );
		}
	}
}

void Snake::handleTickEvent()
{
	bool EatFlag = false;
	int16_t previous_sx, previous_sy;
	int16_t previous_cx, previous_cy;
	int16_t next_sx, next_sy;

	/*if( ++SpeedCounter < speed * 60 / 1000 )
		return;*/
	if( ++SpeedCounter < 7 )
		return;
	else
		SpeedCounter = 0;

	this->invalidate();

	for( uint16_t count = 0 ; count < current_len ; count++ )
	{
		/* Head */
		if( count == 0 )
		{
			int16_t next_cx, next_cy;
			next_sx = SnakeBodys[ count ].sx + SnakeBodys[count].radius * 2 * cos( directionAngle );
			next_sy = SnakeBodys[ count ].sy + SnakeBodys[count].radius * 2 * sin( directionAngle );

			if( next_sx > 1024 )
				next_sx = -40;
			else if( next_sx < -40 )
				next_sx = 1023;

			if( next_sy > 600 )
				next_sy = -40;
			else if( next_sy < -40 )
				next_sy = 599;

			next_cx = next_sx + SnakeBodys[ count ].radius;
			next_cy = next_sy + SnakeBodys[ count ].radius;

			EatFlag = isEatFood( next_cx, next_cy, food_cx, food_cy );
			if( EatFlag == true )
			{
				if( (SnakeEatCallback != NULL) && (SnakeEatCallback->isValid()) )
				{
					SnakeEatCallback->execute( );
				}
			}

			/* Check if the next position does not intersect the boundary */
//			if( detectBorder(next_cx, next_cy, SnakeBodys[ count ].radius) == false )
//			{
				previous_sx = SnakeBodys[ count ].sx;
				previous_sy = SnakeBodys[ count ].sy;
				SnakeBodys[ count ].sx = next_sx;
				SnakeBodys[ count ].sy = next_sy;
				SnakeBodys[ count ].cx = next_cx;
				SnakeBodys[ count ].cy = next_cy;
//			}
//			else
//			{
//				return;
//			}
		}

		/* Body */
		else if( count < MAX_NUMBER_OF_BODY )
		{
			previous_cx = SnakeBodys[ count ].cx;
			previous_cy = SnakeBodys[ count ].cy;
			next_sx = SnakeBodys[ count ].sx;
			next_sy = SnakeBodys[ count ].sy;

			SnakeBodys[ count ].sx = previous_sx;
			SnakeBodys[ count ].sy = previous_sy;
			SnakeBodys[ count ].cx = previous_sx + SnakeBodys[ count ].radius;
			SnakeBodys[ count ].cy = previous_sy + SnakeBodys[ count ].radius;

			previous_sx = next_sx;
			previous_sy = next_sy;
		}
		else return;

		SnakeBodys[ count ].updateZAngle( directionAngle );
		SnakeBodys[ count ].update( speed * 60 / 1000 );
	}

	if( (EatFlag == true) && (current_len < MAX_NUMBER_OF_BODY) )
	{
		SnakeBodys[ current_len ].init( previous_cx, previous_cy, BITMAP_SNAKEBODY_ID );
		SnakeBodys[ current_len ].updateZAngle( directionAngle );
		SnakeBodys[ current_len ].update( speed * 60 / 1000 );
		current_len++;
	}
}

bool Snake::detectBorder ( int16_t cx, int16_t cy, float radius  )
{
	int16_t Left_X = cx - radius;
	int16_t Right_X = cx + radius;
	int16_t Up_Y = cy - radius;
	int16_t Down_Y = cy + radius;

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
	directionAngle = angle;
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
	if( dist <= SnakeBodys[0].radius * 2 )
		return true;
	else
		return false;
}


