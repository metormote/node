/*
 * power.c
 *
 * Created: 9/4/2012 6:59:49 PM
 *  Author: Administrator
 */ 
#include "power.h"

#ifdef GPIO_POWER_GOOD

static void (*power_listener)(enum POWER_EVENT);

/* Interrupt handler for power good events */
ISR(PORTA_INT0_vect) {
  atomIntEnter();
  if (ioport_get_pin_level(GPIO_POWER_GOOD)) {
    power_listener(POWER_GOOD_ON);
  }
  else {
    power_listener(POWER_GOOD_OFF);
  }
  atomIntExit(FALSE, TRUE);
}


void power_good_init() {
  GPIO_POWER_PORT->INT0MASK = GPIO_POWER_PORT->INT0MASK | ioport_pin_to_mask(GPIO_POWER_GOOD);
  GPIO_POWER_PORT->INTCTRL = (GPIO_POWER_PORT->INTCTRL & ~ PORT_INT0LVL_gm) | PORT_INT0LVL_LO_gc;
  GPIO_POWER_PORT->PIN2CTRL = PORT_ISC_BOTHEDGES_gc;
}

void power_set_pg_listener(void (*listener)(enum POWER_EVENT)) {
  power_listener=listener;
}

bool power_is_good() {
  return ioport_get_pin_level(GPIO_POWER_GOOD);
}

void power_enable_dcdc(bool enable) {
  ioport_set_pin_level(GPIO_POWER_GOOD, enable);
}

#endif

/**
 * \b power_init
 *
 * Init ADC application.
 *
 * @return None
 */
void power_init()
{ 
  struct adc_config         adc_conf;
  struct adc_channel_config adcch_conf;
  
  // Clear the configuration structures.
  memset(&adc_conf, 0, sizeof(struct adc_config));
  memset(&adcch_conf, 0, sizeof(struct adc_channel_config));
  
  /* Configure the ADC module:
   * - unsigned, 12-bit results
   * - bandgap (1 V) voltage reference
   * - 1 kHz maximum clock rate
   * - manual conversion triggering
   */
  adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_BANDGAP);
  adc_set_clock_rate(&adc_conf, 1000);
  adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
  
  adc_write_configuration(ADC, &adc_conf);
  //adc_set_callback(ADC, adc_handler);
  
  /* Configure ADC channel:
   * - single-ended measurement from configured input pin
   * - interrupt flag set on completed conversion
   */
  adcch_set_interrupt_mode(&adcch_conf, ADCCH_MODE_COMPLETE);
  adcch_disable_interrupt(&adcch_conf);
  
  adcch_set_input(&adcch_conf, ADCCH_POS_PIN4, ADCCH_NEG_NONE, 1);
  adcch_write_configuration(ADC, POWER_ADC_CHANNEL_TEMP_MASK, &adcch_conf);
  
  adcch_set_input(&adcch_conf, ADCCH_POS_PIN0, ADCCH_NEG_NONE, 1);
  adcch_write_configuration(ADC, POWER_ADC_CHANNEL_BATT_MASK, &adcch_conf);
  
  adc_enable(ADC);
  
  // Do one dummy conversion.
  adc_start_conversion(ADC, POWER_ADC_CHANNEL_TEMP_MASK);
  adc_wait_for_interrupt_flag(ADC, POWER_ADC_CHANNEL_TEMP_MASK);
  
  adc_start_conversion(ADC, POWER_ADC_CHANNEL_BATT_MASK);
  adc_wait_for_interrupt_flag(ADC, POWER_ADC_CHANNEL_BATT_MASK);
  
  return;
}   


int8_t power_read_batt_lvl(uint32_t *batt_lvl) {
  uint32_t adc_val;
  int32_t vin;
  
  // Start next conversion.
  adc_start_conversion(ADC, POWER_ADC_CHANNEL_BATT_MASK);
    
  adc_wait_for_interrupt_flag(ADC, POWER_ADC_CHANNEL_BATT_MASK);
  adc_val=adc_get_unsigned_result(ADC, POWER_ADC_CHANNEL_BATT_MASK);
  
  vin=(1000*adc_val)/4095-50;
  if(vin<0) vin=0;
  *batt_lvl=(5700*vin)/1000;
  
  return STATUS_OK;
}


int8_t power_read_temperature(int32_t *temperature) {
  uint32_t adc_val;
  int32_t vin;
  
  // Start next conversion.
  adc_start_conversion(ADC, POWER_ADC_CHANNEL_TEMP_MASK);
  
  adc_wait_for_interrupt_flag(ADC, POWER_ADC_CHANNEL_TEMP_MASK);
  adc_val=adc_get_unsigned_result(ADC, POWER_ADC_CHANNEL_TEMP_MASK);
  
  vin=(1000*adc_val)/4095-32;
  if(vin<0) vin=0;
  *temperature=159624-488*vin;
  
  return STATUS_OK;
}