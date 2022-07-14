#ifndef SCREEN_SNAKEPRESENTER_HPP
#define SCREEN_SNAKEPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class screen_snakeView;

class screen_snakePresenter : public touchgfx::Presenter, public ModelListener
{
public:
    screen_snakePresenter(screen_snakeView& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~screen_snakePresenter() {};

private:
    screen_snakePresenter();

    screen_snakeView& view;
};

#endif // SCREEN_SNAKEPRESENTER_HPP
