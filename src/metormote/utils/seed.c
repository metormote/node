/*
 * seed.c
 *
 * Created: 2/17/2013 5:00:50 PM
 *  Author: Administrator
 */ 
#include "seed.h"


static int16_t read_adc(void) {
  adc_start_conversion(SEED_ADC, SEED_ADC_CHANNEL_MASK);
  adc_wait_for_interrupt_flag(SEED_ADC, SEED_ADC_CHANNEL_MASK);
  return adc_get_signed_result(SEED_ADC, SEED_ADC_CHANNEL_MASK);
}


static uint8_t vnbox(void) {
  uint8_t b1, b2;
  for(;;) {
    b1=(read_adc() & 1) ^ (( read_adc()>>1) & 1);
    b2=(read_adc() & 1) ^ (( read_adc()>>1) & 1);
    if(b1 && !b2) return 1;
    else if(!b1 && b2) return 0;
  }
}


/**
 * \b seed
 *
 * Seed random generator.
 *
 * @return None
 */
void seed()
{ 
  uint8_t i;
  unsigned long seed;
  struct adc_config         adc_conf;
  struct adc_channel_config adcch_conf;
  
  // Clear the configuration structures.
  memset(&adc_conf, 0, sizeof(struct adc_config));
  memset(&adcch_conf, 0, sizeof(struct adc_channel_config));
  
  /* Configure the ADC module:
   * - signed, 12-bit results
   * - arefa reference
   * - 63 kHz maximum clock rate
   * - manual conversion triggering
   */
  adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, SEED_ADC_REF);
  adc_set_clock_rate(&adc_conf, 63000);
  adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
  adc_write_configuration(ADC, &adc_conf);
  
  /* Configure ADC channel:
   * - single-ended measurement from configured input pin
   * - interrupt flag set on completed conversion
   */
  adcch_set_interrupt_mode(&adcch_conf, ADCCH_MODE_COMPLETE);
  adcch_disable_interrupt(&adcch_conf);
  
  adcch_set_input(&adcch_conf, ADCCH_POS_PIN7, ADCCH_NEG_NONE, 1);
  adcch_write_configuration(SEED_ADC, SEED_ADC_CHANNEL_MASK, &adcch_conf);
  
  adc_enable(SEED_ADC);
  
  // Do one dummy conversion.
  adc_start_conversion(SEED_ADC, SEED_ADC_CHANNEL_MASK);
  adc_wait_for_interrupt_flag(SEED_ADC, SEED_ADC_CHANNEL_MASK);
  
  seed=0;
  for(i=0;i<sizeof(unsigned long)*8;i++) {
    if(vnbox()) seed |= (1L << i);
  }
  srandom(seed);
  
  adc_disable(SEED_ADC);
  
  return;
}   
