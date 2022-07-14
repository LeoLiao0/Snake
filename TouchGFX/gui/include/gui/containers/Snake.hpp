#ifndef SNAKE_HPP
#define SNAKE_HPP

#include <touchgfx/widgets/Widget.hpp>
#include <touchgfx/Application.hpp>
#include <gui/containers/Snake_Head.hpp>
#include <gui/containers/Snake_Body.hpp>
#include <BitmapDatabase.hpp>

class Snake : public touchgfx::Widget
{
public:
	Snake();
	virtual ~Snake();

    virtual touchgfx::Rect getSolidRect() const;
    virtual void draw(const touchgfx::Rect& invalidatedArea) const;
    virtual void handleTickEvent();

    bool detectBorder ( int16_t sx, int16_t sy, float radius );
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

    SnakeHead snakeHead;
    SnakeBody snakeBodys[ MAX_NUMBER_OF_BODY ];

    touchgfx::Rect Map;
    uint16_t current_len;
    float new_Angle;
    float previous_Angle;
    int16_t SpeedCounter;
    int16_t food_cx, food_cy;
    uint16_t SnakeSpeed;

    touchgfx::GenericCallback<>*SnakeEatCallback;
};

#endif // SNAKE_HPP
