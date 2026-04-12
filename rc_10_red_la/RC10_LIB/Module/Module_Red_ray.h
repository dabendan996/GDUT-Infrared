#ifndef MODULE_RED_RAY_H
#define MODULE_RED_RAY_H

#pragma once

#include "stm32h7xx_hal.h"

class Red_ray
{
	public:
		
  static Red_ray* GetInstance(TIM_HandleTypeDef *htim, uint32_t Channel);
    
  void InitUART();

  Red_ray(const Red_ray&) = delete;
  Red_ray& operator=(const Red_ray&) = delete;



private:
  Red_ray(TIM_HandleTypeDef *htim, uint32_t Channel); // ??????
  Red_ray() = default;
};




#endif