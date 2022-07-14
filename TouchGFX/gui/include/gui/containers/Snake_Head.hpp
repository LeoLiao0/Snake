#ifndef SNAKE_HEAD_HPP
#define SNAKE_HEAD_HPP

#include <touchgfx/widgets/TextureMapper.hpp>
#include <touchgfx/EasingEquations.hpp>
#include <touchgfx/mixins/MoveAnimator.hpp>
#include <BitmapDatabase.hpp>

class SnakeHead : public touchgfx::MoveAnimator<touchgfx::TextureMapper>
{
public:
	void init( int sx, int sy, touchgfx::BitmapId bmp, float angle );
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
    float directionAngle;
};

#endif // SNAKE_HEAD_HPP
