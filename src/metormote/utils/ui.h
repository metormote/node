/*
 * ui.h
 *
 * Created: 10/6/2011 4:16:38 PM
 *  Author: Administrator
 */ 


#ifndef UI_H_
#define UI_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "atom.h"
#include "gpio.h"
#include "board.h"
#include "tc.h"

#define UI_GREEN_LED      0
#define UI_RED_LED        1
#define UI_YELLOW_LED     2
#define UI_BLUE_LED       3

#define UI_PUSH_BUTTON_DOWN     1
#define UI_PUSH_BUTTON_UP       2

#define UI_EVENT_BUTTON_DOWN     1
#define UI_EVENT_BUTTON_UP       2



#ifdef __cplusplus
extern "C" { 
#endif

struct BLINK {
  uint16_t off;
  uint16_t period;
  uint16_t timeout;
};



void ui_init(void);
bool ui_is_led_on(uint8_t led);
void ui_led_on(uint8_t led);
void ui_led_off(uint8_t led);
void ui_led_blink(uint8_t led, uint16_t on_period, uint16_t off_period, uint16_t timeout);
void ui_led_blink_off(uint8_t led);

void ui_set_ui_listener(void (*listener)(uint8_t event));


#ifdef __cplusplus
}
#endif



#endif /* UI_H_ */