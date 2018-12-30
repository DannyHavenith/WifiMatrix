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
} g;

// communication with esp-link
esp_link::client::uart_type uart{19200};
IMPLEMENT_UART_INTERRUPT( uart);
esp_link::client esp{ uart};

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
        render_string("wait wait wait wait wait wait wait wait wait wait", offset);
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

        // implement scroll
        if (g.do_scroll)
        {
            set(led);
            g.wait_accumulator += g.wait_step;
            if (g.wait_accumulator >= g.wait_threshold)
            {
                g.wait_accumulator -= g.wait_threshold;
                --g.text_offset;

                // render text to display.
                display.clear();
                auto columns_rendered = render_string( g.text_buffer, g.text_offset);
                if (columns_rendered < 8 * matrix_count)
                {
                    render_string( g.text_buffer, 0);
                    if (columns_rendered == 0)
                    {
                        g.text_offset = 0;
                    }
                }
                display.transmit();
            }
            clear(led);
        }
    }
}
