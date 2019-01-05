//
//  Copyright (C) 2018 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#include <avr_utilities/esp-link/client.hpp>
#include <avr_utilities/pin_definitions.hpp>
#include <avr_utilities/esp-link/command.hpp>
#include <avr_utilities/devices/max7219_matrix_display.hpp>
#include <avr_utilities/devices/bitbanged_spi.h>
#include <avr_utilities/font5x8.hpp>
#include <avr_utilities/simple_text_parsing.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <util/delay.h>
#include "timer.h"
#include "snowflakes.hpp"
#include "simple_random.hpp"

#define MQTT_BASE_NAME "matrix/"

namespace {

template< typename T>
struct identity
{
    using type = T;
};

template<typename T>
T min( const T &a, const typename identity<T>::type &b)
{
    if (a<b) return a;
    return b;
}

// setup to talk to a max7219 display array.
struct spi_pins
{
    PIN_TYPE( B, 3) mosi;
    PIN_TYPE( B, 5) clk;
    pin_definitions::null_pin_type miso;
};

// this display has 9 matrices, talks through bit-banged spi and uses B4 as cs pin.
using csk_type = PIN_TYPE( B, 4);
using spi_type = bitbanged_spi< spi_pins>;
constexpr uint8_t matrix_count = 9;
using display_type = max7219::display_buffer<matrix_count, spi_type, csk_type>;
display_type display;


/**
 * Global state that describes the behaviour of this device.
 */
struct global_state_type
{
    uint8_t flashSpeed   = 25;
    uint8_t flashCounter = 0;
    bool    displayIsOn = true;
    char    text_buffer[256] = {0};
    int16_t text_offset = 0;

    void set_speed( uint8_t speed)
    {
        if (speed > wait_threshold) speed = wait_threshold;
        wait_step = speed;
    }

    // determine globally the speed of all animations
    bool do_scroll = false;
    uint8_t  wait_step = 48;
    uint8_t  wait_accumulator = 0;
    static constexpr uint8_t wait_threshold = 128;

    bool do_snowflakes = false;
    bool snowflakes_active = false;

    bool do_fireworks = false;
    bool fireworks_active = false;
} g;

// communication with esp-link
esp_link::client::uart_type uart{19200};
IMPLEMENT_UART_INTERRUPT( uart);
esp_link::client esp{ uart};

PIN_TYPE( B, 6) led;


/**
 * Object to translate an ascii string into bits to be rendered
 * on the matrix display.
 */
class string_bits
{
public:
    string_bits( const char *string)
    :next_character{string}, next_column{nullptr}
    {}

    /**
     * Get the next column of bits to be rendered.
     */
    uint8_t next()
    {
        if (not next_column and not fetch_next_character())
        {
            return 0;
        }
        if (next_column < reinterpret_cast<const uint8_t *>(4))
        {
            --next_column;
            return 0;
        }
        uint8_t value = pgm_read_byte(next_column++);
        if (not value)
        {
            next_column = nullptr;
        }
        return value;
    }

    bool at_end() const
    {
        return (not next_column and not *next_character);
    }

private:
    bool fetch_next_character()
    {
        const auto character = *next_character;
        if (!character) return false;
        ++next_character;

        if (character == ' ')
        {
            next_column = reinterpret_cast< const uint8_t *>(2);
        }
        else
        {
            next_column = font5x8::find_character( character);
        }
        return true;
    }

    const char    *next_character; ///< pointer to next character
    const uint8_t *next_column;    ///< pointer in pgm space to next character pixel column
};

/**
 * Render a string to the display at its current cursor position.
 *
 * Parameter 'offset' moves the string to the right by inserting
 * empty columns or, if offset is negative, to the left by not rendering
 * the first columns of the text.
 *
 * This function returns the number of columns it tried to render, which
 * could be more than the actual amount of columns on the display.
 *
 */
uint16_t render_string( const char *str, int16_t offset = 0)
{
    uint16_t columns = 0;
    string_bits bits{ str};

    // "render" columns to the left of the physical display
    while (offset < 0)
    {
        ++offset;
        bits.next();
    }

    // render empty columns to move the text to the right
    while (offset > 0)
    {
        display.push_column(0);
        ++columns;
        --offset;
    }

    // render columns, potentially
    // rendering more than fit on the display.
    while (not bits.at_end())
    {
        display.push_column( bits.next());
        ++columns;
    }

    return columns;
}

char *my_strcpy( char *dest, const char *src, uint16_t len)
{
    while (len && *src)
    {
        *dest++ = *src++;
        --len;
    }
    *dest = 0;
    return dest;
}

template< typename IntegerType>
bool assign_if_topic(
        IntegerType &value,
        const char *expected,
        const char *(&topic_start),
        const char *topic_end,
        esp_link::string_ref &message)
{
    using namespace text_parsing;
    if (consume(topic_start, topic_end, expected))
    {
        value = parse_uint16( message.buffer, message.buffer + message.len);
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * This function is called when an update is received on the subscribed MQTT topic.
 */
void update( const esp_link::packet *p, uint16_t size)
{
    using namespace esp_link;
    using namespace text_parsing;

    packet_parser parser{ p};
    toggle(led);

    string_ref topic;
    string_ref message;
    parser.get( topic);
    parser.get( message);

    const char *topic_ptr = topic.buffer;
    const char *topic_end = topic_ptr + topic.len;

    // if the topic is indeed the expected one...
    if (consume(topic_ptr, topic_end, MQTT_BASE_NAME))
    {
        if (consume(topic_ptr, topic_end, "text"))
        {
            display.clear();
            my_strcpy( g.text_buffer, message.buffer, message.len);
            g.do_scroll = render_string( g.text_buffer) > display_type::column_count;
            g.wait_accumulator = 0;
            if (not g.do_scroll)
            {
                display.transmit();
                g.text_offset = 0;
            }
        }
        else if (consume( topic_ptr, topic_end, "flash"))
        {
            if (consume( topic_ptr, topic_end, "Speed"))
            {
                g.flashSpeed = parse_uint16( message.buffer, message.buffer + message.len);
            }
            else
            {
                uint8_t value = parse_uint16( message.buffer, message.buffer + message.len);
                if (value)
                {
                    g.flashCounter = g.flashSpeed;
                }
                else
                {
                    display.enable( true);
                    g.displayIsOn = true;
                    g.flashCounter = 0;
                }
            }
        }
        else if (consume( topic_ptr, topic_end, "scrollSpeed"))
        {
            g.set_speed( parse_uint16( message.buffer, message.buffer + message.len));
        }
        else if (consume( topic_ptr, topic_end, "snow"))
        {
            g.do_snowflakes = parse_uint16( message.buffer, message.buffer + message.len) != 0;
            if (g.do_snowflakes)
            {
                g.snowflakes_active = true;
            }
        }
        else if (consume( topic_ptr, topic_end, "fireworks"))
        {
            g.do_fireworks = parse_uint16( message.buffer, message.buffer + message.len) != 0;
            if (g.do_fireworks)
            {
                g.fireworks_active = true;
            }
        }
    }
}

void connected( const esp_link::packet *p, uint16_t size)
{
    set(led);
    using esp_link::mqtt::subscribe;
    using esp_link::mqtt::publish;
    //esp.send("connected\n");
    esp.execute( subscribe, MQTT_BASE_NAME "+", 0);
    esp.execute( publish, MQTT_BASE_NAME "version", "0.2", 0, false);
    clear(led);
}

// fireworks related classes

/**
 * This class represents a dot rendered on a matrix display.
 *
 * Each dot has an x,y-position where x and y have a higher resolution
 * than the actual dots on the matrix.
 *
 * A dot can be "at_end", which means that the dot will not be
 * displayed.
 */
class dot
{
public:

    dot()
    :x{0}, y{y_end}
    {
    }

    dot( int16_t x, uint8_t y)
    :x{x},y{y}
    {
    }

    void render( display_type &display) const
    {
        if (
            not at_end() and
            x >= 0 and x < 8*x_scale*matrix_count and
            y >= 0)
        {
            display.set_pixel( x / x_scale, y / y_scale);
        }
    }

    bool at_end() const
    {
        return y >= y_end;
    }


    static constexpr uint8_t x_scale = 16;
    static constexpr uint8_t y_scale = 16;
    static constexpr uint8_t y_end = 8 * y_scale;

    int16_t x; // in 12.4 fixed point
    int16_t y;  // in12.4 fixed point
};

/**
 * a moving dot
 */
class velocity_dot : public dot
{
public:
    velocity_dot()
    :dot{},
     vx{0},
     vy{0}
    {

    }

    velocity_dot( int16_t x, uint8_t y, int16_t vx, int8_t vy)
    :dot{x,y}, vx{vx}, vy{vy}
    {
    }

    void step(int8_t gravity)
    {
        if (not at_end())
        {
            x += vx;
            y += vy;
            vy += gravity;
        }
    }


    int16_t vx;
    int8_t vy;
};

class rocket
{
public:
    rocket()
    :fuse{0}, trigger{127}
    {
    }

    rocket(int16_t x, int16_t vx, int8_t vy, uint8_t fuse, int8_t trigger = 0)
    :
     fuse{fuse},
     trigger{trigger}
    {
        dots[0] = {x, 127, vx, vy};
    }

    bool step(int8_t gravity)
    {
        // rocket on the ground, fuse burning
        if (fuse)
        {
            --fuse;
            return true;
        }

        // in flight, before burst
        if (not dots[0].at_end() and dots[0].vy < trigger)
        {
            // create a trail of at most 4 dots
            for (uint8_t dot = min(dot_count, 4) - 1; dot > 0; --dot)
            {
                dots[dot] = dots[dot-1];
            }

            dots[0].step( gravity);

            // do we burst now?
            if (dots[0].vy >= trigger)
            {
                constexpr static int8_t v_offset = 6;
                // make sure we are always in the burst state from here on
                trigger = -128;
                for (auto &dot : dots)
                {
                    dot.x = dots[0].x;
                    dot.y = dots[0].y;
                    dot.vx = dots[0].vx + plusminus( v_offset, v_offset);
                    dot.vy = dots[0].vy + plusminus( 1, v_offset);
                }
            }
            return true;
        }

        // after burst
        bool active_dot = false;
        for ( auto &dot : dots)
        {
            if (!dot.at_end())
            {
                active_dot = true;
                dot.step( gravity);
            }
        }

        return active_dot;
    }

    void render( display_type &display) const
    {
        if (not fuse)
        {
            for (const auto &dot: dots)
            {
                dot.render( display);
            }
        }
    }

private:
    static constexpr uint8_t dot_count = 8;
    velocity_dot dots[dot_count];
    uint8_t fuse;
    int8_t trigger;
};

class rockets_type
{
public:
    rockets_type()
    {
        for (auto & rocket: rockets)
        {
            rocket = random_rocket();
        }
    }

    bool render( display_type &display, bool make_new)
    {
        constexpr int8_t gravity = 1;
        bool active = false;
        for (auto &rocket: rockets)
        {
            if (rocket.step( gravity))
            {
                active = true;
                rocket.render( display);
            }
            else if (make_new)
            {
                rocket = random_rocket();
                active = true;
            }
        }

        return active;
    }

private:
    static rocket random_rocket()
    {
        constexpr static int16_t vx_range = 8;
        constexpr static int8_t vy_range = 10;
        constexpr static uint8_t fuse_range = 110;
        //constexpr static uint8_t trigger_range = 6;
        return
            {
                static_cast<int16_t>( no_more_than( 8*matrix_count*16)), // x
                static_cast<int16_t>( plusminus( vx_range)), // vx
                static_cast<int8_t>(-(no_more_than( vy_range) + 8)), // vy, negative is up
                static_cast<uint8_t>(no_more_than( fuse_range)), // fuse
                //static_cast<int8_t>( no_more_than( trigger_range) - 2)
                0
            };

    }
    constexpr static uint8_t rocket_count = 5;
    rocket rockets[rocket_count];
} rockets;

}

int main()
{
    using esp_link::mqtt::setup;

    snowflakes_type<display_type> snowflakes;

    make_output(led);
    display.auto_shift( false);


    display.clear();
    for (int16_t offset = 0; offset > -150; --offset)
    {
        display.clear();
        render_string("wait wait wait wait wait wait wait", offset);
        display.transmit();
        _delay_ms( 20);
    }

    display.clear();
    render_string("Connecting...");
    display.transmit();

    while (not esp.sync()) toggle( led);
    display.clear();
    display.transmit();

    esp.execute( setup, &connected, nullptr, nullptr, &update);
    auto next = Timer::After( Timer::ticksPerSecond/50);
    connected(nullptr, 0);
    for (;;)
    {
        while (not HasPassed(next))
        {
            esp.try_receive();
        }
        next = Timer::After( Timer::ticksPerSecond/50);

        // implement flash
        if (g.flashCounter)
        {
            if (not --g.flashCounter)
            {
                g.flashCounter = g.flashSpeed;
                display.enable( g.displayIsOn);
                g.displayIsOn = not g.displayIsOn;
            }
        }

        // if one of the animations is active, render text.
        bool do_render = g.snowflakes_active or g.fireworks_active;

        // implement scroll
        if (g.do_scroll)
        {
            g.wait_accumulator += g.wait_step;
            if (g.wait_accumulator >= g.wait_threshold)
            {
                g.wait_accumulator -= g.wait_threshold;
                --g.text_offset;
                do_render = true;
            }
        }

        if (do_render)
        {
            // render text to display.
            display.clear();
            auto columns_rendered = render_string( g.text_buffer, g.text_offset);

            // as the string is scrolling off to the left, we need to draw the start
            // of the string on the right again.

            // add some space between the end of the string and the start of the
            // repeated string.
            static constexpr auto repeat_space = 6;
            if (g.do_scroll and columns_rendered < display_type::column_count + repeat_space)
            {
                for (uint8_t count = repeat_space; count; --count)
                {
                    display.push_column( 0);
                }
                render_string( g.text_buffer, 0);
                if (columns_rendered == 0)
                {
                    g.text_offset = repeat_space;
                }
            }
        }

        if (g.snowflakes_active)
        {
            g.snowflakes_active = snowflakes.render( display, g.do_snowflakes);
        }

        if (g.fireworks_active)
        {
            g.fireworks_active = rockets.render( display, g.do_fireworks);
        }

        if (do_render)
        {
            display.transmit();
        }
    }
}
