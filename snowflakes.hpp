//
//  Copyright (C) 2019 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef SNOWFLAKES_HPP_
#define SNOWFLAKES_HPP_
#include "simple_random.hpp"

/**
 * This class animates a number of "snow flakes" across a matrix display.
 *
 * Snowflakes have individual vertical speed. Their horizontal speed
 * is determined by one of two "wind" velocities. Each wind velocity
 * is determined by a limited random walk. Wether a flake is influenced
 * by wind1 or wind2 is determined by a threshold that also randomly
 * moves up and down. The first n flakes are influenced by wind1 while
 * the next count - n flakes are influenced by wind2.
 *
 * All positions and speeds are kept in fixed point numbers.
 */
template<typename display_type>
class snowflakes_type
{
public:

    bool render( display_type &display, bool create_new = true)
    {
        update_wind();

        bool active = false;
        for ( uint8_t i = 0; i < count; ++i)
        {
            auto &flake = flakes[i];

            flake.step();
            if (flake.at_end())
            {
                if (create_new)
                {
                    flake = random_snowflake();
                    active = true;
                }
            }
            else
            {
                active = true;
            }
            if (i < threshold)
            {
                flake.offset_x( wind1);
            }
            else
            {
                flake.offset_x( wind2);
            }
            flake.render( display);
        }
        return active;
    }

private:

    class snowflake
    {
    public:
        snowflake()
        :x{0}, y{y_end}, vy{0}
        {

        }
        snowflake( int16_t x, uint8_t vy)
        :x{x}, y{0}, vy{vy}
        {
        }

        void step( )
        {
            if (not at_end())
            {
                y += vy;
            }
        }

        void offset_x( int8_t offset)
        {
            x += offset;
            if (x < 0 or x >= static_cast<int16_t>(display_type::column_count * x_scale))
            {
                x = 0;
                y = y_end;
            }
        }

        void render( display_type &display)
        {
            if (not at_end())
            {
                display.set_pixel( x / x_scale, y / y_scale);
            }
        }

        bool at_end() const
        {
            return y >= y_end;
        }

    private:
        static constexpr uint8_t x_scale = 16;
        static constexpr uint8_t y_scale = 16;
        static constexpr uint8_t y_end = 8 * y_scale;

        int16_t x; // in 10.6 fixed point
        uint8_t y;  // in 4.4 fixed point
        uint8_t vy; // in 4.4 fixed point
    };

    static snowflake random_snowflake()
    {
        return
            {
                static_cast<int16_t>( my_rand()%(display_type::column_count *16)),
                static_cast<uint8_t>( 1 + my_rand()%4)
            };
    }

    void update_wind()
    {
        constexpr int8_t wind_limit = 3;
        if (wind1 > -wind_limit and (my_rand() & 0x18) == 0) --wind1;
        if (wind1 < wind_limit  and (my_rand() & 0x18) == 0) ++wind1;
        if (wind2 > -wind_limit and (my_rand() & 0x18) == 0) --wind2;
        if (wind2 < wind_limit  and (my_rand() & 0x18) == 0) ++wind2;

        if (threshold > count/3 and (my_rand() & 0x10)) --threshold;
        if (threshold < (2 * count) / 3  and (my_rand() & 0x10)) ++threshold;
    }

    /// amount of snowflakes
    static constexpr uint8_t count = 20;

    /// determine how many snowflakes are influenced by
    /// wind1, resp. wind2
    int8_t threshold = count/2;

    snowflake flakes[20];

    int8_t wind1 = 0;
    int8_t wind2 = -3;
};


#endif /* SNOWFLAKES_HPP_ */
