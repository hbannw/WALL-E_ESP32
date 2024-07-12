# WALL-E_ESP32
esp32 code for Wall-e 3d printed robot

First thanks to Simon Bluett (Chillibasket) for the original work

This is the code for my modified buil of the Wall-E robot
https://www.thingiverse.com/thing:6583998/


the following changes were made :
- use the ESP32 Wroom chip to control and communicate
- user a DFPlayer card to have better sound and volume control
- shrink the web page code to fit in the ESP32 SPIFFS filesystem
- add the eyebrow control to manual moves and animations

It is necessary to create a file named : settings.h containing the following lines
#define WIFI_SSID "your wifi SSID"
#define WIFI_PASSWORD "your wifi password"

The files in the data directory have to be copied to the SPIFFS filesystem on the ESP32
IIs is possible with Arduino IDE 1.8 and the following tool https://randomnerdtutorials.com/install-esp32-filesystem-uploader-arduino-ide/

you can use you home wifi settings or the shared connection setting from your phone

the files in the sound directory has to be copied to the MicroSD card
this card has to be inserted in the DFPlayer card
