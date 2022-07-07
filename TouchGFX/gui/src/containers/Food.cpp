#include <gui/containers/Food.hpp>
#include <stdlib.h>
#include <BitmapDatabase.hpp>
#include <math.h>

Food::Food()
{

}

void Food::initialize()
{
    FoodBase::initialize();
    noDisplayAreaRect = Rect( 0, 0, 0, 0 );
}

void Food::setNoDisplayArea ( Rect r )
{
	noDisplayAreaRect = r;
}

void Food::setNewPosition ( void )
{
	bool newPositionAllow = false;
	int16_t range_x_max, rang_x_min;
	int16_t range_y_max, rang_y_min;

	do{
		range_x_max = getX( ) + getWidth( ) - 1;
		rang_x_min = getX( );

		range_y_max = getY( ) + getHeight( ) - 1;
		rang_y_min = getY( );

		x = rand() % (range_x_max + 1 - rang_x_min) + rang_x_min;
		y = rand() % (range_y_max + 1 - rang_y_min) + rang_y_min;

		/* Check that the food is not out the border */
		if( ((x + circle_food.getWidth()) <= range_x_max) && ((y + circle_food.getHeight()) <= range_y_max) )
		{
			/* Check the new position is not in the no display area */
			if( (x > noDisplayAreaRect.x) && (x < (noDisplayAreaRect.x + noDisplayAreaRect.width - 1)) )
				newPositionAllow = false;
			else if( ((x + getWidth()) > noDisplayAreaRect.x) && ((x + getWidth()) < (noDisplayAreaRect.x + noDisplayAreaRect.width - 1)) )
				newPositionAllow = false;
			else if( (y > noDisplayAreaRect.y) && (y < (noDisplayAreaRect.y + noDisplayAreaRect.height - 1)) )
				newPositionAllow = false;
			else if( ((y + getHeight()) > noDisplayAreaRect.y) && ((y + getHeight()) < (noDisplayAreaRect.y + noDisplayAreaRect.height - 1)) )
				newPositionAllow = false;
			else
				newPositionAllow = true;
		}

	}while( newPositionAllow == false );

	/* Set food new position */
	circle_food.setXY( x, y );
	circle_food.invalidate( );

	if( (PositionCahngeCallback != NULL) && (PositionCahngeCallback->isValid()) )
	{
		int16_t cx = getFoodCenterX();
		int16_t cy = getFoodCenterY();
		PositionCahngeCallback->execute( cx, cy );
	}
}

int16_t Food::getFoodCenterX ( void )
{
	return x + circle_food.getWidth() / 2;
}

int16_t Food::getFoodCenterY ( void )
{
	return y + circle_food.getHeight() / 2;
}
