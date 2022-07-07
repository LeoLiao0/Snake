#ifndef SNAKE_HPP
#define SNAKE_HPP

#include <touchgfx/widgets/Image.hpp>
#include <touchgfx/widgets/Widget.hpp>
#include <touchgfx/widgets/TextureMapper.hpp>
#include <touchgfx/Application.hpp>
#include <touchgfx/EasingEquations.hpp>
#include <touchgfx/mixins/MoveAnimator.hpp>
#include <BitmapDatabase.hpp>

class SnakeBody : public touchgfx::TextureMapper
{
public:
	void init( int cx, int cy, touchgfx::BitmapId bmp );
	void update( int16_t speed );

    // Make sure that we do not JSMOC split
    virtual touchgfx::Rect getSolidRect() const
    {
        return touchgfx::Rect();
    }

    int16_t cx; // X coordinate of the center
    int16_t cy; // Y coordinate of the center
    int16_t sx; // X coordinate of upper left corner
    int16_t sy; // Y coordinate of upper left corner
    uint8_t alpha;
    float radius;
};

class Snake : public touchgfx::Widget
{
public:
	Snake();
	virtual ~Snake();

    virtual touchgfx::Rect getSolidRect() const;
    virtual void draw(const touchgfx::Rect& invalidatedArea) const;
    virtual void handleTickEvent();

    bool detectBorder ( int16_t cx, int16_t cy, float radius );
    void setMapPosition ( touchgfx::Rect rect );
    void setDirectionAngle ( float angle );
    void setFoodPosition ( int16_t cx, int16_t cy );
    bool isEatFood ( int16_t Snake_cx, int16_t Snake_cy, int16_t Food_cx, int16_t Food_cy );

	void setSnakeEatCallback(touchgfx::GenericCallback<>& callback)
	{
		this->SnakeEatCallback = &callback;
	}
private:
    static const uint16_t MAX_NUMBER_OF_BODY = 300;

    touchgfx::TextureMapper SnakeHead;
    SnakeBody SnakeBodys[ MAX_NUMBER_OF_BODY ];

    touchgfx::Rect Map;
    uint16_t current_len;
    float directionAngle;
    int16_t speed; // unit in ms
    int16_t SpeedCounter;
    int16_t food_cx, food_cy;

    touchgfx::GenericCallback<>*SnakeEatCallback;
};

#endif // SNAKE_HPP
