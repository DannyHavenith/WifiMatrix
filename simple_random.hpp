//
//  Copyright (C) 2019 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef SIMPLE_RANDOM_HPP_
#define SIMPLE_RANDOM_HPP_

/// very crude pseudo random generator
inline uint16_t my_rand()
{
    static uint16_t state;
    return state += 13331; // adding a prime number
}

inline int16_t plusminus( int16_t range)
{
    return my_rand() % (2 * range + 1) - range;
}



inline uint16_t no_more_than( uint16_t range)
{
    return my_rand() % range;
}

inline int16_t plusminus( int16_t offset, int16_t range)
{
    int16_t val = no_more_than( range) + offset;
    if (my_rand() & 0x80)
    {
        val = -val;
    }
    return val;
}


#endif /* SIMPLE_RANDOM_HPP_ */
