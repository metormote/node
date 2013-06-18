/*
 * seed.h
 *
 * Created: 2/17/2013 5:00:59 PM
 *  Author: Administrator
 */ 


#ifndef SEED_H_
#define SEED_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <gpio.h>
#include <board.h>
#include <status_codes.h>
#include "asf.h"

#define SEED_ADC_CHANNEL 0
#define SEED_ADC_CHANNEL_MASK ADC_CH0
#define SEED_ADCCH_POS ADCCH_POS_PIN7

void seed(void);


#endif /* SEED_H_ */