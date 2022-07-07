#ifndef FOOD_HPP
#define FOOD_HPP

#include <gui_generated/containers/FoodBase.hpp>

class Food : public FoodBase
{
public:
    Food();
    virtual ~Food() {}

    virtual void initialize();

    void setNoDisplayArea ( Rect r );
    void setNewPosition ( void );
    int16_t getFoodCenterX ( void );
    int16_t getFoodCenterY ( void );

	void setPositionCahngeCallback(touchgfx::GenericCallback<int16_t&, int16_t&>& callback)
	{
		this->PositionCahngeCallback = &callback;
	}
protected:

	Rect noDisplayAreaRect;
    int16_t x; // position y of food
    int16_t y; // position y of food

    touchgfx::GenericCallback<int16_t&, int16_t&>*PositionCahngeCallback;
};

#endif // FOOD_HPP
