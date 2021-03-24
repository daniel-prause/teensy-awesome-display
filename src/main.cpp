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
#define SERIAL_BUFFER_OP_SIZE 16
#define SERIAL_BUFFER_SIZE 64

#include <stdio.h>
#include <stdlib.h>

static uint8_t screen_buffer[SCREEN_BUFFER_SIZE] = {};
static char serial_buffer[SERIAL_BUFFER_OP_SIZE] = {};

void print_fps();
void serial_communication();
byte readRegisterValueFromSerial();

elapsedMillis elapsed;
static unsigned int frame_counter = 0;
int* fps_ptr;
char fps_str[32];

void setup() {
    for (int i=0;i<2;i++){
        delay(100);
        oled_begin();
        delay(100);
        oled_clear();
        delay(1000);
        oled_clear();
        // oled_bitmap_gray(PIC2);
        // delay(1000);
    }

    fps_ptr = (int*) malloc(sizeof(int));

    unsigned long ulngStart = millis();
    Serial.begin(4608000);
    while (!Serial && ((millis() - ulngStart) <= 666));
}

void loop() {
    serial_communication();
    // print_fps();
}

uint8_t buffer[8192];
uint8_t maxBuffer[8];
void serial_communication() {
    int operation = readRegisterValueFromSerial();
    switch (operation) {
        case 0xE4:
            int bytes_read = 0;
            int total_bytes = 0;
            while (total_bytes < 8192){
                if (Serial.available() > 0) {
                    bytes_read = Serial.readBytes(serial_buffer, SERIAL_BUFFER_SIZE);
                    memcpy(screen_buffer + total_bytes, serial_buffer, bytes_read);
                    total_bytes += bytes_read;
                }
            }

            oled_bitmap_gray(screen_buffer);
            frame_counter += 1;
            break;
    }
}

byte readRegisterValueFromSerial() {
    while (Serial.available() <= 0);
    return Serial.read();
}

void print_fps(){
    if (elapsed >= 1000){
        *fps_ptr = frame_counter;
        frame_counter = 0;
        elapsed = 0;
    }
    memset(fps_str, 0, 32);
    sprintf(fps_str, "%u FPS", *fps_ptr);
    er_oled_string(0, 0, fps_str, 0);
}
