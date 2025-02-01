/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "pico/multicore.h"

#include "usb_descriptors.h"

// Waveshare includes
#include "DEV_Config.h"
#include "GUI_Paint.h"
#include "LCD_1in28.h"
#include "QMI8658.h"
#include "fonts.h"


//--------------------------------------------------------------------+
// Waveshare helper functions
//--------------------------------------------------------------------+

UWORD *BlackImage;


float acceleration_magnitude() {
    // Fetch acceleration info
    float acc[3], gyro[3];
    unsigned int tim_count = 0;
    QMI8658_read_xyz(gyro, acc, &tim_count);

    return (float) sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
}

void waveshare_init() {
  DEV_Module_Init();
  adc_init();
  adc_gpio_init(29);
  adc_select_input(3);

  // Initialize LCD
  LCD_1IN28_Init(HORIZONTAL);
  LCD_1IN28_Clear(WHITE);
  DEV_SET_PWM(60);

  // Create image
  UDOUBLE Imagesize = LCD_1IN28_HEIGHT * LCD_1IN28_WIDTH * 2;
  if ((BlackImage = (UWORD *)malloc(Imagesize)) == NULL)
  {
      printf("Failed to apply for black memory...\r\n");
      exit(0);
  }
  Paint_NewImage((UBYTE *)BlackImage, LCD_1IN28.WIDTH, LCD_1IN28.HEIGHT, 0, WHITE);
  Paint_SetScale(65);
  Paint_Clear(WHITE);

  // Initialize accelerometer
  QMI8658_init();

  // Seed RNG using noise from accelerometer
  srand((int) (acceleration_magnitude() * 1000));
}


//--------------------------------------------------------------------+
// State machine
//--------------------------------------------------------------------+
typedef enum {
  STATE_LOW_KEYLESS, // Low acceleration, no key selected
  STATE_HIGH_KEYLESS, // High acceleration, no key selected
  STATE_LOW_KEY, // Low acceleration, key selected
  STATE_HIGH_KEY, // High acceleration, key selected
} MagicState;



// typedef enum {
//   EVENT_LOW_TO_HIGH,
//   EVENT_HIGH_TO_LOW
// } MagicEvent;

void enter_state_low_keyless();
void enter_state_high_keyless();
void enter_state_low_key();
void enter_state_high_key();
void exit_state_low_keyless();
void exit_state_high_keyless();
void exit_state_low_key();
void exit_state_high_key();
void update_state_low_keyless();
void update_state_high_keyless();
void update_state_low_key();
void update_state_high_key();


// Range of ASCII characters to include, inclusive
const int ASCII_LOWER_BOUND = 65;
const int ASCII_UPPER_BOUND = 90;
const float LOW_TO_HIGH_THRESHOLD = 100;
const float HIGH_TO_LOW_THRESHOLD = 30;
const float TIMEOUT_KEY_ENTER_TIME = 2 * 1000000; // Idle time in seconds spent in STATE_LOW_KEY before key is entered

char getRandomChar() {
    int rn = (rand() % (ASCII_UPPER_BOUND - ASCII_LOWER_BOUND + 1)) + ASCII_LOWER_BOUND;
    return (char) rn;
}

MagicState state = STATE_LOW_KEYLESS;
char selectedCharacter;
uint64_t stateStartTime;

void magic_update() {
  waveshare_init();

  while (true) {
    float magnitude = acceleration_magnitude();

    switch (state) {
      case STATE_LOW_KEYLESS: 
      {
        
        Paint_DrawNum(120-41, 120-64-24, magnitude, &Font16, 2, BLACK, WHITE);
        LCD_1IN28_Display(BlackImage);

        if (magnitude > LOW_TO_HIGH_THRESHOLD) {
          state = STATE_HIGH_KEY;
          selectedCharacter = getRandomChar();
        }
      }
      break;

      case STATE_HIGH_KEY:
      {
        // Prepare cstring for printing
        char selectedCharacterString[2];
        selectedCharacterString[0] = selectedCharacter;
        selectedCharacterString[1] = 0;

        // Print selected character
        Paint_DrawString_EN(120-41, 120-64-24, selectedCharacterString, &FontLibertinusMono128, BLACK, WHITE);
        Paint_DrawNum(120-41, 120-64-24, magnitude, &Font16, 2, BLACK, WHITE);
        LCD_1IN28_Display(BlackImage);

        if (magnitude < HIGH_TO_LOW_THRESHOLD) {
            state = STATE_LOW_KEY;
            stateStartTime = time_us_64();
        }
      }
      break;

      case STATE_LOW_KEY:
      {
        // Prepare cstring for printing
        char selectedCharacterString[2];
        selectedCharacterString[0] = selectedCharacter;
        selectedCharacterString[1] = 0;
        // Print selected character
        Paint_DrawString_EN(120-41, 120-64-24, selectedCharacterString, &FontLibertinusMono128, BLACK, WHITE);
        Paint_DrawNum(120-41, 120-64-24, magnitude, &Font16, 2, BLACK, WHITE);
        Paint_DrawNum(120-41, 120-64-24+30, time_us_64(), &Font16, 2, BLACK, WHITE);
        Paint_DrawNum(120-41, 120-64-24+50, stateStartTime + TIMEOUT_KEY_ENTER_TIME, &Font16, 2, BLACK, WHITE);
        LCD_1IN28_Display(BlackImage);

        if (magnitude > LOW_TO_HIGH_THRESHOLD) {
          state = STATE_HIGH_KEY;
          selectedCharacter = getRandomChar();
        }
        else if (time_us_64() > stateStartTime + TIMEOUT_KEY_ENTER_TIME) {
          Paint_Clear(WHITE);
          
          multicore_fifo_push_blocking(selectedCharacter);

          state = STATE_LOW_KEYLESS;
        }
      }
      break;
    }
  }


}

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void hid_task(void);

/*------------- MAIN -------------*/
int main(void)
{
  
  multicore_launch_core1(magic_update);

  board_init();

  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }

  while (1)
  {
    tud_task();
    hid_task();
    sleep_ms(10);
  }

}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

static void send_hid_report(uint8_t report_id, uint32_t btn)
{
  // skip if hid is not ready yet
  if ( !tud_hid_ready() ) return;

  switch(report_id)
  {
    case REPORT_ID_KEYBOARD:
    {
      // use to avoid send multiple consecutive zero report for keyboard
      static bool has_keyboard_key = false;

      if ( btn )
      {
        uint8_t keycode[6] = { 0 };
        keycode[0] = ((uint8_t) multicore_fifo_pop_blocking()) - 61;
        tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, keycode);

        has_keyboard_key = true;
      }else
      {
        // send empty key report if previously has key pressed
        if (has_keyboard_key) tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, NULL);
        has_keyboard_key = false;
      }
    }
    break;

    default: break;
  }
}

// Every 10ms, we will sent 1 report for each HID profile (keyboard, mouse etc ..)
// tud_hid_report_complete_cb() is used to send the next report after previous one is complete
void hid_task(void)
{
  // Poll every 10ms
  const uint32_t interval_ms = 10;
  static uint32_t start_ms = 0;

  if ( board_millis() - start_ms < interval_ms) return; // not enough time
  start_ms += interval_ms;

  uint32_t const btn = multicore_fifo_rvalid(); // board_button_read();

  // Remote wakeup
  if ( tud_suspended() && btn )
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  }else
  {
    // Send the 1st of report chain, the rest will be sent by tud_hid_report_complete_cb()
    send_hid_report(REPORT_ID_KEYBOARD, btn);
  }
}

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
{
  (void) instance;
  (void) len;

  uint8_t next_report_id = report[0] + 1u;

  if (next_report_id < REPORT_ID_COUNT)
  {
    send_hid_report(next_report_id, 1);// board_button_read());
  }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  (void) instance;

  // if (report_type == HID_REPORT_TYPE_OUTPUT)
  // {
  //   // Set keyboard LED e.g Capslock, Numlock etc...
  //   if (report_id == REPORT_ID_KEYBOARD)
  //   {
  //     // bufsize should be (at least) 1
  //     if ( bufsize < 1 ) return;

  //     uint8_t const kbd_leds = buffer[0];

  //     if (kbd_leds & KEYBOARD_LED_CAPSLOCK)
  //     {
  //       // Capslock On: disable blink, turn led on
  //       blink_interval_ms = 0;
  //       // board_led_write(true);
  //     }else
  //     {
  //       // Caplocks Off: back to normal blink
  //       // board_led_write(false);
  //       blink_interval_ms = BLINK_MOUNTED;
  //     }
  //   }
  // }
}
