/*
 * potentiometer.c
 *
 * Created: 6/17/2012 8:11:30 PM
 *  Author: Administrator
 */ 
#include "potentiometer.h"


struct spi_device POTENTIOMETER_DEVICE = {
  .id = SENSOR_CS
};

/**
@brief  This function is for init the potentiometer spi. 
*/ 
void potentiometer_init()
{  
  ioport_set_pin_dir(SENSOR_SCK, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(SENSOR_SCK, false);
  ioport_set_pin_dir(SENSOR_MOSI, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(SENSOR_MOSI, false);
  ioport_set_pin_dir(SENSOR_MISO, IOPORT_DIR_INPUT);
  ioport_set_pin_dir(SENSOR_CS, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(SENSOR_CS, true);
  
  spi_master_init(SENSOR_SPI);
  spi_master_setup_device(SENSOR_SPI, &POTENTIOMETER_DEVICE, SPI_MODE_0, POTENTIOMETER_SPI_MASTER_SPEED, 0);
  spi_enable(SENSOR_SPI);  
  
  //(SENSOR_SPI)->INTCTRL=SPI_INTLVL_LO_gc;
}

int8_t potentiometer_set(uint8_t value) {
  int8_t status=STATUS_OK;
  uint8_t bytes[2];
  
  bytes[0]=0;
  bytes[1]=value;
  
  spi_select_device(SENSOR_SPI, &POTENTIOMETER_DEVICE);
  
  spi_write_single(SENSOR_SPI, bytes[0]);
  while(!spi_is_tx_ok(SENSOR_SPI));
  spi_write_single(SENSOR_SPI, bytes[1]);
  while(!spi_is_tx_ok(SENSOR_SPI));
  
  spi_deselect_device(SENSOR_SPI, &POTENTIOMETER_DEVICE);
  
  return status;
}


int8_t potentiometer_process_message(pb_istream_t *msg_stream, pb_ostream_t *resp_stream) {
  potentiometer_In msg;
  potentiometer_Out reply;
  
  //decode message
  if (!pb_decode(msg_stream, potentiometer_In_fields, &msg)) {
    return ERR_IO_ERROR;
  }
  
  if(potentiometer_set((uint8_t)msg.value)!=STATUS_OK) {
    return ERR_IO_ERROR;
  }
  
  reply.value=msg.value;
  
  //encode reply
  if (!pb_encode(resp_stream, potentiometer_Out_fields, &reply)) {
    return ERR_IO_ERROR;
  }
  
  return STATUS_OK;
}