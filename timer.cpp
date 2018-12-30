//
//  Copyright (C) 2017 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//


#include "timer.h"
#include <avr/io.h>
#include <avr_utilities/pin_definitions.hpp>


struct InitTimer
{
	InitTimer()
	{
		TCCR1A = 0;
		TCCR1B = 5; // clk/1024
	}
} timerstarter;

namespace Timer
{
	uint16_t GetCurrent()
	{
		return TCNT1;
	}

	/**
	 * Test if the timer has passed the wait value.
	 *
	 * Since the wait value is not changed, this function will become
	 * false after a timer overflow and then become true again.
	 * call HasPassedOnce() to make sure that the timer value is set to
	 * always after the time point has passed.
	 *
	 * HasPassed() will always return true for the special value Timer::always.
	 */
	bool HasPassed( const TimerWaitValue &val)
	{
		auto currentTimer = GetCurrent();
		if (val.startValue <= val.endValue)
		{
			return currentTimer >= val.endValue or currentTimer < val.startValue;
		}
		else
		{
			return currentTimer < val.startValue and currentTimer >= val.endValue;
		}
	}

	/**
	 * Test if the timer has passed the timer wait value and set the
	 * given value to Timer::always if it has.
	 */
	bool HasPassedOnce( TimerWaitValue &val)
	{
		if (HasPassed( val))
		{
			val = always;
			return true;
		}
		else
		{
			return false;
		}
	}

	/**
	 * Return a timer wait value that is a given amount of ticks in the future.
	 */
	TimerWaitValue After( uint16_t ticks)
	{
		auto currentTimer = GetCurrent();
		return { currentTimer, currentTimer + ticks};
	}

    /**
     * Return a timer wait value that is a given amount of ticks
     * after another value, which is expected to have passed already.
     */
    TimerWaitValue After( const TimerWaitValue &passedValue, uint16_t ticks)
    {
        return { passedValue.endValue + ticks, passedValue.endValue};
    }

}
