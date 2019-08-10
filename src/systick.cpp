/*
 * intrrupt.cpp
 *
 *  Created on: 2019/04/06
 *      Author: User
 */

#include <systick.hpp>

std::map<SysTick_Interrupt* const,const std::function<void(void)>> SysTick_Interrupt::callFunctions_;

extern "C"
{
	void SysTick_Handler()
	{
		SysTick_Interrupt::update();
	}
}
