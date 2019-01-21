//
//  Copyright (C) 2019 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef FLARE_HPP_
#define FLARE_HPP_
#include <ws2811/rgb.h>
#include <ws2811/rgb_operators.hpp>

class flare
{
public:
    enum Mode
    {
        Off = 0,
        OneShot,
        BackAndForthForward,
        BackAndForthBackward,
        ModeCount // end of sequence
    };

    template< size_t size>
    bool render( ws2811::rgb (&leds)[size]) const
    {
        if (m_mode == Off or size <= m_led_index) return false;

        leds[m_led_index] = ws2811::fade( m_scale, m_color_from, m_color_to);

        return true;
    }

    void stop()
    {
        m_mode = Off;
    }

    bool is_active() const
    {
        return m_mode != Off;
    }

    uint8_t led_index() const
    {
        return m_led_index;
    }

    void setup(
            uint8_t new_index,
            Mode new_mode,
            const ws2811::rgb &from,
            const ws2811::rgb &to,
            uint8_t new_speed)
    {
        m_led_index = new_index;
        m_mode = new_mode;
        m_color_from = from;
        m_color_to = to;
        m_speed = new_speed;
        m_accumulator = 0;
        m_scale = 0;
    }



    void step()
    {
        m_accumulator += m_speed;
        while (m_accumulator >= threshold)
        {
            m_accumulator -= threshold;
            if (m_mode == BackAndForthForward)
            {
                if (m_scale != 255)
                {
                    if (++m_scale == 255)
                    {
                        m_mode = BackAndForthBackward;
                    }
                }
            }
            else if (m_mode == OneShot)
            {
                if (m_scale != 255)
                {
                    ++m_scale;
                }
                else
                {
                    m_mode = Off;
                }
            }
            else if (m_mode == BackAndForthBackward)
            {
                if (m_scale != 0)
                {
                    if (--m_scale == 0)
                    {
                        m_mode = BackAndForthForward;
                    }
                }
            }
        }
    }

private:
    Mode        m_mode = Off;
    ws2811::rgb m_color_from{0,0,0};
    ws2811::rgb m_color_to{0,0,0};
    uint8_t     m_scale = 0;
    uint8_t     m_led_index = 0;
    uint16_t    m_accumulator;
    uint8_t     m_speed;
    static constexpr uint16_t threshold = 16;
};




#endif /* FLARE_HPP_ */
