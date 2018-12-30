/*
 * timer.h
 *
 *  Created on: Dec 10, 2017
 *      Author: danny
 */

#ifndef TIMER_H_
#define TIMER_H_
#include <stdint.h>
namespace Timer
{
	struct TimerWaitValue
	{
		uint16_t startValue;
		uint16_t endValue;
	};

	uint16_t GetCurrent();
	bool HasPassed( const TimerWaitValue &val);
	bool HasPassedOnce( TimerWaitValue &val);
	TimerWaitValue After( uint16_t ticks);
	TimerWaitValue After( const TimerWaitValue &passedValue, uint16_t ticks);

	constexpr uint16_t ticksPerSecond = 7812;
	constexpr TimerWaitValue always = {0,0};
}

#endif /* TIMER_H_ */
