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
#include <avr/pgmspace.h>
#include <util/delay.h>

#define MQTT_BASE_NAME "matrix/"

namespace {
esp_link::client::uart_type uart{19200};
esp_link::client esp{ uart};
using csk_type = PIN_TYPE( B, 4);
struct spi_pins
{
    PIN_TYPE( B, 3) mosi;
    PIN_TYPE( B, 5) clk;
    pin_definitions::null_pin_type miso;
};
using spi_type = bitbanged_spi< spi_pins>;

max7219::display_buffer<9, spi_type, csk_type> display;

IMPLEMENT_UART_INTERRUPT( uart);
PIN_TYPE( B, 6) led;

}


class string_bits
{
public:
    string_bits( const char *string, uint8_t length)
    :next_character{string}, characters_left{length}, next_column{nullptr}
    {}

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

    bool fetch_next_character()
    {
        if (not characters_left)
        {
            return false;
        }
        else
        {
            const auto character = *next_character++;
            --characters_left;

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
    }

    bool at_end() const
    {
        return (not next_column and not characters_left);
    }

private:
    const char    *next_character; ///< pointer to next character
    uint8_t       characters_left; ///< how many characters still left
    const uint8_t *next_column;    ///< pointer in pgm space to next character pixel column
};

uint16_t render_string( const char *str, uint8_t length, int16_t offset = 0)
{
    uint16_t columns = 0;
    string_bits bits{ str, length};
    while (offset < 0)
    {
        ++offset;
        bits.next();
    }

    while (offset > 0)
    {
        display.push_column(0);
        ++columns;
        --offset;
    }

    while (not bits.at_end())
    {
        display.push_column( bits.next());
        ++columns;
    }

    return columns;
}

/**
 * Consume as many characters from the character array pointed to by "input" as
 * match the expectation.
 *
 * This function will increase "input" and stop consuming characters at the
 * first character that doesn't match the expectation string or when the
 * expectation string is exhausted.
 */
bool consume( const char *(&input), const char *end, const char *expectation)
{
    while (*expectation and input < end and *input++ == *expectation++) /* continue */;
    return *expectation == 0;
}

/**
 * This function is called when an update is received on the subscribed MQTT topic.
 */
void update( const esp_link::packet *p, uint16_t size)
{
    using namespace esp_link;
    packet_parser parser{ p};
    toggle(led);

    string_ref topic;
    string_ref message;
    parser.get( topic);
    parser.get( message);

    const char *topic_ptr = topic.buffer;
    const char *topic_end = topic_ptr + topic.len;

    display.clear();
    render_string( "r", 1);

    toggle(led);
    // if the topic is indeed the expected one...
    if (consume(topic_ptr, topic_end, MQTT_BASE_NAME "text"))
    {
        display.clear();
        render_string( message.buffer, message.len);
        display.transmit();
    }
}

void connected( const esp_link::packet *p, uint16_t size)
{
    set(led);
    using esp_link::mqtt::subscribe;
    using esp_link::mqtt::publish;
    //esp.send("connected\n");
    esp.execute( subscribe, MQTT_BASE_NAME "+", 0);
    esp.execute( publish, MQTT_BASE_NAME "version", "0.1", 0, false);
    clear(led);
}

int main()
{
    using esp_link::mqtt::setup;
    make_output(led);

    display.clear();
    render_string( "wait", 4);
    display.transmit();

    _delay_ms(3000);

    display.clear();
    render_string("Connecting...", 13);
    display.transmit();

    while (not esp.sync()) toggle( led);
    display.clear();
    display.transmit();


    esp.execute( setup, &connected, nullptr, nullptr, &update);
    connected(nullptr, 0);
    for (;;)
    {
        esp.try_receive();
    }

}
