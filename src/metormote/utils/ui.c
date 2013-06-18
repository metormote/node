/*
 * ui.c
 *
 * Created: 10/6/2011 4:16:22 PM
 *  Author: Administrator
 */ 
#include "ui.h"
#include <util/atomic.h>

static volatile struct BLINK blink[LED_COUNT];

static void (*ui_listener)(uint8_t);

static void ovf_interrupt_callback(void);
//static void led_toggle(uint8_t led);


/* Interrupt handler for button events */
ISR(GPIO_SWITCH_INT_VECTOR) {
  atomIntEnter();
  if(ui_listener!=NULL) {
    if(!ioport_get_pin_level(GPIO_SWITCH)) {
      ui_listener(UI_EVENT_BUTTON_DOWN);
    }
    else {
      ui_listener(UI_EVENT_BUTTON_UP);
    }
  }
  atomIntExit(FALSE, TRUE);
}


void ui_set_ui_listener(void (*listener)(uint8_t event)) {
  ui_listener=listener;
}

void ui_init() {
  
  //init the interrupt for the push button
  //use INT0
  GPIO_SWITCH_PORT->INT0MASK = GPIO_SWITCH_PORT->INT0MASK | ioport_pin_to_mask(GPIO_SWITCH);
  GPIO_SWITCH_PORT->INTCTRL = (GPIO_SWITCH_PORT->INTCTRL & ~PORT_INT0LVL_gm)  | PORT_INT0LVL_LO_gc;
  GPIO_SWITCH_PORT->PIN4CTRL = IOPORT_SENSE_BOTHEDGES;
  
  /*
  * Unmask clock for TCD0
  */
  tc_enable(&TCD0);
  
  /*
  * Configure interrupts callback functions for overflow interrupt
  */
  tc_set_overflow_interrupt_callback(&TCD0, ovf_interrupt_callback);
      
  /*
  * Configure TC in normal mode, configure period 100ms
  */
  tc_set_wgm(&TCD0, TC_WG_NORMAL);
  tc_write_period(&TCD0, F_CPU / 1024 / 10);
  
  
  /*
  * Run TCD0 at AVR_CPU_HZ / 1024
  */
  tc_write_clock_source(&TCD0, TC_CLKSEL_DIV1024_gc);
  
}

bool ui_is_led_on(uint8_t led) {
  switch(led) {
    case UI_GREEN_LED:
      return !ioport_get_pin_level(GPIO_LED_GREEN);
    case UI_RED_LED:
      return !ioport_get_pin_level(GPIO_LED_RED);
    case UI_YELLOW_LED:
      return !ioport_get_pin_level(GPIO_LED_YELLOW);
    case UI_BLUE_LED:
      return !ioport_get_pin_level(GPIO_LED_BLUE);
    default:
      return false;
  }
}

void ui_led_on(uint8_t led) {
  switch(led) {
    case UI_GREEN_LED:
      ioport_set_pin_level(GPIO_LED_GREEN, false);
      break;
    case UI_RED_LED:
      ioport_set_pin_level(GPIO_LED_RED, false);
      break;
    case UI_YELLOW_LED:
      ioport_set_pin_level(GPIO_LED_YELLOW, false);
      break;
    case UI_BLUE_LED:
      ioport_set_pin_level(GPIO_LED_BLUE, false);
      break;
    default:
      break;
  }
}


void ui_led_off(uint8_t led){
  switch(led) {
    case UI_GREEN_LED:
      ioport_set_pin_level(GPIO_LED_GREEN, true);
      break;
    case UI_RED_LED:
      ioport_set_pin_level(GPIO_LED_RED, true);
      break;
    case UI_YELLOW_LED:
      ioport_set_pin_level(GPIO_LED_YELLOW, true);
      break;
    case UI_BLUE_LED:
      ioport_set_pin_level(GPIO_LED_BLUE, true);
      break;
    default:
      break;
  }
}


void ui_led_blink(uint8_t led, uint16_t on_period, uint16_t off_period, uint16_t timeout){
  
  blink[led].off=off_period;
  blink[led].period=on_period+off_period;
  blink[led].timeout=timeout;
  
  
  /*
  * Enable TC interrupt (overflow)
  */
  cpu_irq_disable();
  tc_set_overflow_interrupt_level(&TCD0, TC_OVFINTLVL_LO_gc);
  cpu_irq_enable();
  
  //turn led on
  ui_led_on(led);
  
}


void ui_led_blink_off(uint8_t led){
  blink[led].timeout=0;
  ui_led_off(led);
}


static void ovf_interrupt_callback() {
  uint8_t i, off=0;
  uint16_t v;
  
  atomIntEnter();
  
  for(i=0;i<LED_COUNT;i++) {
    if(blink[i].timeout>1) {
      v=blink[i].timeout % blink[i].period;
      if(v==0) {
        ui_led_on(i);
      }     
      else if(v==blink[i].off) {
        ui_led_off(i);
      } 
      blink[i].timeout--;
    }
    else if(blink[i].timeout==1) {
      ui_led_off(i);
      blink[i].timeout--;
      off++;
    }
    else {
      off++;
    }
  }
  
  if(off==LED_COUNT) {
    /*
    * Disable TC interrupt
    */
    cpu_irq_disable();
    tc_set_overflow_interrupt_level(&TCD0, TC_OVFINTLVL_OFF_gc);
    cpu_irq_enable();
    
    /*
    * Mask clock for TCD0
    */
    //tc_disable(&TCD0);
  }  
  atomIntExit(FALSE, FALSE);
}

/*
static void led_toggle(uint8_t led){
  switch(led) {
    case UI_GREEN_LED:
      gpio_toggle_pin(GPIO_LED_GREEN);
      break;
    case UI_RED_LED:
      gpio_toggle_pin(GPIO_LED_RED);
      break;
    case UI_YELLOW_LED:
      gpio_toggle_pin(GPIO_LED_YELLOW);
      break;
    case UI_BLUE_LED:
      gpio_toggle_pin(GPIO_LED_BLUE);
      break;
    default:
      break;
  }
}
*/



