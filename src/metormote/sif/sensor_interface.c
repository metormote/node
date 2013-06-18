/*
 * sensor_interface.c
 *
 * Created: 10/6/2012 11:42:12 PM
 *  Author: Administrator
 */ 
# include "sensor_interface.h"


void sif_init() {
  
}

int8_t sif_process_gpio_message(pb_istream_t *msg_stream, pb_ostream_t *resp_stream) {
  gpio_State msg;
  gpio_State reply;
  
  //decode message
  if (!pb_decode(msg_stream, gpio_State_fields, &msg)) {
    return ERR_IO_ERROR;
  }
  
  if(msg.has_IO_DAC0) {
    ioport_set_pin_level(SENSOR_DAC0, msg.IO_DAC0);
  }
  
  if(msg.has_IO_DAC1) {
    ioport_set_pin_level(SENSOR_DAC0, msg.IO_DAC1);
  }
  
  reply.has_IO_DAC0=true;
  reply.IO_DAC0=ioport_get_pin_level(SENSOR_DAC0);
  
  reply.has_IO_DAC1=true;
  reply.IO_DAC1=ioport_get_pin_level(SENSOR_DAC1);
  
  
  //encode reply
  if (!pb_encode(resp_stream, gpio_State_fields, &reply)) {
    return ERR_IO_ERROR;
  }
  
  return STATUS_OK;
}