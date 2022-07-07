/**
  ******************************************************************************
  * File Name          : TouchGFXGPIO.cpp
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include <touchgfx/hal/GPIO.hpp>

/**
 * GPIO_ID Enum, these are used bt TouchGFX framework to signal events.
 *
 * VSYNC_FREQ,  /// Pin is toggled at each VSYNC
 * RENDER_TIME, /// Pin is high when frame rendering begins, low when finished
 * FRAME_RATE,  /// Pin is toggled when the frame buffers are swapped.
 * MCU_ACTIVE   /// Pin is high when framework is utilizing the MCU.
 *
 */

/* USER CODE BEGIN TouchGFXGPIO.cpp */

using namespace touchgfx;

/*
 * Perform configuration of IO pins.
 */
void GPIO::init()
{

}

/*
 * Sets a pin high.
 */
void GPIO::set(GPIO_ID id)
{

}

/*
 * Sets a pin low.
 */
void GPIO::clear(GPIO_ID id)
{

}

/*
 * Toggles a pin.
 */
void GPIO::toggle(GPIO_ID id)
{

}

/*
 * Gets the state of a pin.
 */
bool GPIO::get(GPIO_ID id)
{
  return false;
}

/* USER CODE END TouchGFXGPIO.cpp */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
