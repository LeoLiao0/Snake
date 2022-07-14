#include <gui/containers/Snake_Body.hpp>
#include <math.h>

using namespace touchgfx;

void SnakeBody::init( int sx, int sy, BitmapId bmp )
{
	setBitmap( touchgfx::Bitmap(bmp) );
//	setWidth(touchgfx::Bitmap(bmp).getWidth());
//	setHeight(touchgfx::Bitmap(bmp).getHeight());
//	setBitmapPosition(0.000f, 0.000f);
//	setScale(1.000f);
//	setCameraDistance(1000.000f);
//	setOrigo((touchgfx::Bitmap(bmp).getWidth() / 2), (touchgfx::Bitmap(bmp).getHeight() / 2), 1000.000f);
//	setCamera((touchgfx::Bitmap(bmp).getWidth() / 2), (touchgfx::Bitmap(bmp).getHeight() / 2));
//	updateAngles(0.000f, 0.000f, 0.000f);
//	setRenderingAlgorithm( touchgfx::TextureMapper::BILINEAR_INTERPOLATION );
	setVisible( true );

	this->sx = sx;
	this->sy = sy;

	radius = touchgfx::Bitmap(bmp).getWidth() / 2;
	alpha = 255;

	setAlpha( alpha );
	moveTo( sx, sy );
}

void SnakeBody::update( uint16_t SnakeSpeed )
{
	startMoveAnimation(sx, sy, SnakeSpeed, EasingEquations::linearEaseNone, EasingEquations::linearEaseNone);
}


