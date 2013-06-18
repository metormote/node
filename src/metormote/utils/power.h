/*
 * power.h
 *
 * Created: 9/4/2012 7:00:02 PM
 *  Author: Administrator
 */ 
#ifndef POWER_H_
#define POWER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <gpio.h>
#include <board.h>
#include <status_codes.h>
#include "asf.h"
#include "atom.h"

#define POWER_ADC_CHANNEL_TEMP 0
#define POWER_ADC_CHANNEL_TEMP_MASK ADC_CH0
#define POWER_ADC_CHANNEL_BATT 1
#define POWER_ADC_CHANNEL_BATT_MASK ADC_CH1

enum POWER_EVENT {
  POWER_GOOD_ON,
  POWER_GOOD_OFF
};


void power_init(void);
void power_good_init(void);
void power_set_pg_listener(void (*listener)(enum POWER_EVENT));
bool power_is_good(void);
void power_enable_dcdc(bool enable);
int8_t power_read_batt_lvl(uint32_t *batt_lvl);
int8_t power_read_temperature(int32_t *temperature);

#endif /* POWER_H_ */