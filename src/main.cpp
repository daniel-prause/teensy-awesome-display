/*
 * Hardware connection (SPI):
 *     OLED   =>    Teensy
 *  1. GND    ->    GND
 *  2. VCC    ->    3.3
 *4. SCL    ->    SCK (13)
 *5. SDI    ->    MOSI (11)
 * 14. DC     ->    9
 * 15. RES    ->    8
 * 16. CS     ->    10
 */

#include <Adafruit_BME280.h>
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <Wire.h>

#include "oled.h"

// #define SEALEVELPRESSURE_HPA (1013.25)
#define SCREEN_BUFFER_SIZE 8192
#define SERIAL_BUFFER_SIZE 64

void print_fps();
void serial_communication();
byte readRegisterValueFromSerial();

float temperature;
float humidity;
float altitude;
float pressure;
char sensor_str[14];

Adafruit_BME280 bme;

void setup()
{
    delay(100);
    oled_begin();
    delay(1000);
    oled_clear();

    unsigned long ulngStart = millis();
    Serial.begin(9600);
    while (!Serial && ((millis() - ulngStart) <= 666))
        ;

    while (!bme.begin(0x76))
    {
        er_oled_string(0, 0, "BME error!", 0);
        delay(1000);
    }
}

void loop()
{
    serial_communication();
<<<<<<< src/main.cpp
=======
    print_fps(); // this keeps the SPI connection alive? i don't know ¯\_(ツ)_/¯
>>>>>>> src/main.cpp
}

uint8_t buffer[SCREEN_BUFFER_SIZE];
void serial_communication()
{
    int operation = readRegisterValueFromSerial();
    switch (operation)
    {
    case 0x11:
    case 0x12:
    {
        memset(buffer, 0, sizeof(buffer));
        oled_bitmap_gray(buffer);
        break;
    }

    case 0xCD:
    {
        temperature = bme.readTemperature();
        humidity = bme.readHumidity();

        memset(sensor_str, 0, sizeof(sensor_str));
        snprintf(sensor_str, sizeof(sensor_str), "%.02f %.02f", temperature, humidity);
        Serial.write(sensor_str, 14);
        break;
    }

    case 0xE4:
    {
        int bytes_read = 0;
        int total_bytes = 0;
        char serial_buffer[SCREEN_BUFFER_SIZE];
        while (total_bytes < SCREEN_BUFFER_SIZE)
        {
            if (Serial.available() > 0)
            {
                bytes_read = Serial.readBytes(serial_buffer, SERIAL_BUFFER_SIZE);
                memcpy(buffer + total_bytes, serial_buffer, bytes_read);
                total_bytes += bytes_read;
            }
        }
        oled_bitmap_gray(buffer);
        break;
    }
    }
}

byte readRegisterValueFromSerial()
{
    while (Serial.available() <= 0)
        ;
    return Serial.read();
}
