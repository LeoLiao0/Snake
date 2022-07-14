#ifndef SNAKE_BODY_HPP
#define SNAKE_BODY_HPP

#include <touchgfx/widgets/Image.hpp>
#include <touchgfx/EasingEquations.hpp>
#include <touchgfx/mixins/MoveAnimator.hpp>
#include <BitmapDatabase.hpp>

class SnakeBody : public touchgfx::MoveAnimator<touchgfx::Image>
{
public:
	void init( int sx, int sy, touchgfx::BitmapId bmp );
	void update( uint16_t SnakeSpeed );

    // Make sure that we do not JSMOC split
    virtual touchgfx::Rect getSolidRect() const
    {
        return touchgfx::Rect();
    }

    int16_t sx; // X coordinate of upper left corner
    int16_t sy; // Y coordinate of upper left corner
    uint8_t alpha;
    float radius;
};

#endif // SNAKE_BODY_HPP
