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

#define MQTT_BASE_NAME "matrix/"

namespace {

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


/// very crude pseudo random generator
uint16_t my_rand()
{
    static uint16_t state;
    return state += 13331; // adding a prime number
}


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
    bool snowflakes_falling = false;
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
            g.do_scroll = render_string( g.text_buffer) > 8 * matrix_count;
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
                g.snowflakes_falling = true;
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
            if (x < 0 or x >= (8 * matrix_count * x_scale))
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
                static_cast<int16_t>( my_rand()%(8*matrix_count*16)),
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

    static constexpr uint8_t count = 20;
    int8_t threshold = count/2;
    snowflake flakes[20];
    int8_t wind1 = 0;
    int8_t wind2 = -3;
} snowflakes;

}

int main()
{
    using esp_link::mqtt::setup;
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

        bool do_render = false;
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

        if (do_render or g.snowflakes_falling)
        {
            // render text to display.
            display.clear();
            auto columns_rendered = render_string( g.text_buffer, g.text_offset);
            if (g.do_scroll and columns_rendered < 8 * matrix_count)
            {
                render_string( g.text_buffer, 0);
                if (columns_rendered == 0)
                {
                    g.text_offset = 0;
                }
            }
        }

        if (g.snowflakes_falling)
        {
            snowflakes.render( display, g.do_snowflakes);
        }

        if (do_render or g.snowflakes_falling)
        {
            display.transmit();
        }
    }
}
