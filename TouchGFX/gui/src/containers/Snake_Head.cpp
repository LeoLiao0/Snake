#include <gui/containers/Snake_Head.hpp>
#include <math.h>

using namespace touchgfx;

uint16_t SnakeSpeed = 5;

void SnakeHead::init( int sx, int sy, BitmapId bmp, float angle )
{
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

	this->sx = sx;
	this->sy = sy;

	radius = touchgfx::Bitmap(bmp).getWidth() / 2;
	alpha = 255;
    directionAngle = angle;

	setAlpha( alpha );
	moveTo( sx, sy );
}

void SnakeHead::update( uint16_t SnakeSpeed )
{
	startMoveAnimation(sx, sy, SnakeSpeed, EasingEquations::linearEaseNone, EasingEquations::linearEaseNone);
}
