/*
 Note:The module needs to be jumpered to an SPI interface. R19,R21 Short and R18,R20 Open  
 Unused signal pin Recommended to connect to GND 
  == Hardware connection ==
    OLED   =>    Arduino
  *1. GND    ->    GND
  *2. VCC    ->    3.3
  *4. SCL    ->    SCK
  *5. SDI    ->    MOSI
  *14. DC     ->    9
  *15. RES    ->    8  
  *16. CS     ->    10
*/

#include <SPI.h>
#include <Wire.h>

#include "oled.h"

#define SCREEN_BUFFER_SIZE 8192
#define SERIAL_BUFFER_SIZE 4

static uint8_t screen_buffer[SCREEN_BUFFER_SIZE] = {};
static char serial_buffer[SERIAL_BUFFER_SIZE] = {};

void write_screen();
void serial_communication();

void setup() {
    oled_begin();
    oled_clear();
    oled_bitmap_gray(PIC2);
    delay(10);
    
    unsigned long ulngStart = millis();
    Serial.begin(4608000);
    while (!Serial && ((millis () - ulngStart) <= 666));
    // Serial.println("Starty");
}

void loop() {
    // oled_clear();

    // oled_bitmap_gray(PIC1);
    // delay(10);

    // oled_clear();
    // oled_bitmap_gray(PIC2);
    // delay(1000);

    // oled_clear();
    // er_oled_bitmap_mono(PIC3);
    // delay(10);

    // oled_clear();
    // er_oled_string(0, 0, "********************************", 0);
    // er_oled_string(32, 16, "EastRising Technology", 0);
    // er_oled_string(40, 32, "www.buydisplay.com", 1);
    // er_oled_string(0, 48, "********************************", 0);
    // uint8_t i;
    // for (i = 0; i <= 48; i++) {
    //     command(0xa1);  //start line
    //     data(i);
    //     delay(100);
    // }
    // for (i = 48; i > 0; i--) {
    //     command(0xa1);  //start line
    //     data(i);
    //     delay(100);
    // }

    serial_communication();
    delay(10);
}

void write_screen() {
    uint16_t bytes_written = 0;
    Serial.println("Ok");
    uint8_t bytes_read = Serial.readBytes(serial_buffer, 8192);

    oled_bitmap_gray(screen_buffer);
    
    // while (bytes_written < 8192) {
    //     //Serial.println("Ok");
    //     uint8_t bytes_read = Serial.readBytes(serial_buffer, 8193);
    //     bytes_written += bytes_read;
    //     memcpy(tile + bytes_written, buf, bytes_read);
    // }
    // displayTextDebug(tft_2, String(bytes_written));
}


void serial_communication(){
    int bytes_read;
    if (Serial.available() > 0) {
        bytes_read = Serial.readBytes(serial_buffer, SERIAL_BUFFER_SIZE);
    }
    Serial.print(serial_buffer);
    // Serial.print("@64############################################################!@128###########################################################!@192##########################################################!@256###########################################################!");
    // Serial.flush();
}
