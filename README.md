# WifiMatrix
Led matrix display controlled by AVR, connected to esp-link, listening to MQTT topics for control.

This project contains the code for an AVR (atmega328) connected to an esp-link esp8266. The 
AVR listens to MQTT events and will display text published to "matrix/text" on a daisy chain of
8x8 matrices, each one controlled by a single max7219.

This project also serves as an example of how to use the the "raw AVR" [esp-link client code](https://github.com/DannyHavenith/avr_utilities/tree/master/avr_utilities/esp-link) that
is in the [avr_utilities](https://github.com/DannyHavenith/avr_utilities) library.

