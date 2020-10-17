#include <ILI9341_t3.h>
#include <SPI.h>
#include <Wire.h>
#include <XPT2046_Touchscreen.h>
#include <lvgl.h>

#include "lv_examples.h"

#define TFT_CS 10
#define TFT_DC 9
ILI9341_t3 display = ILI9341_t3(TFT_CS, TFT_DC);

#define CS_PIN 8
#define TIRQ_PIN 2
XPT2046_Touchscreen touch = XPT2046_Touchscreen(CS_PIN);

#define LVGL_TICK_PERIOD 60
int screenWidth = 320;
int screenHeight = 240;

int oldTouchX = 0;
int oldTouchY = 0;

/*A static or global variable to store the buffers*/
static lv_disp_buf_t disp_buf;

/*Static or global buffer(s). The second buffer is optional*/
static lv_color_t buf_1[LV_HOR_RES_MAX * 10];
static lv_color_t buf_2[LV_HOR_RES_MAX * 10];

IntervalTimer tick;
static void lv_tick_handler(void) {
    lv_tick_inc(LVGL_TICK_PERIOD);
}

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint16_t width = (area->x2 - area->x1 + 1);
    uint16_t height = (area->y2 - area->y1 + 1);

    display.writeRect(area->x1, area->y1, width, height, (uint16_t *)color_p);

    lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

bool my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
    uint16_t touchX, touchY;

    if (touch.touched()) {
        TS_Point p = touch.getPoint();  // Retrieve a point

        touchX = p.x;  // Rotate the co-ordinates
        touchY = p.y;
        //touchY = 240-touchY;

        if ((touchX != oldTouchX) || (touchY != oldTouchY)) {
            Serial.print("x= ");
            Serial.print(touchX, DEC);
            Serial.print(" y= ");
            Serial.println(touchY, DEC);

            oldTouchY = touchY;
            oldTouchX = touchX;
            data->state = LV_INDEV_STATE_PR;
            data->point.x = touchX;
            data->point.y = touchY;
        }
    } else {
        data->point.x = oldTouchX;
        data->point.y = oldTouchY;
        data->state = LV_INDEV_STATE_REL;
    }
    return 0;
}

void setup() {
    display.begin();
    display.fillScreen(ILI9341_BLACK);
    display.setRotation(1);

    touch.begin();
    touch.setRotation(1);

    lv_init();
    lv_disp_buf_init(&disp_buf, buf_1, buf_2, LV_HOR_RES_MAX * 10);

    /*Initialize the display*/
    lv_disp_t *disp;
    lv_disp_drv_t disp_drv;                 /*A variable to hold the drivers. Can be local variable*/
    lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
    disp_drv.buffer = &disp_buf;            /*Set an initialized buffer*/
    disp_drv.flush_cb = my_disp_flush;      /*Set a flush callback to draw to the display*/
    disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/

    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);          /*Descriptor of a input device driver*/
    indev_drv.type = LV_INDEV_TYPE_POINTER; /*Touch pad is a pointer-like device*/
    indev_drv.read_cb = my_touchpad_read;   /*Set your driver function*/
    lv_indev_drv_register(&indev_drv);      /*Finally register the driver*/

    lv_demo_widgets();  // Start demo

    Serial.println("tick.begin");
    tick.begin(lv_tick_handler, LVGL_TICK_PERIOD * 1000);  // Start ticker
}

void loop() {
    lv_task_handler(); /* let the GUI do its work */
    delay(5);
}
