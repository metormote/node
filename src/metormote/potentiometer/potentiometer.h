/*
 * potentiometer.h
 *
 * Created: 6/17/2012 8:11:11 PM
 *  Author: Administrator
 */ 
#ifndef POTENTIOMETER_H_
#define POTENTIOMETER_H_

#include <stdio.h>
#include <string.h>
#include <spi.h>
#include <spi_master.h>
#include <status_codes.h>
#include <interrupt.h>
#include "board.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "potentiometer.pb.h"

#define POTENTIOMETER_OUT_MSG_CODE              67
#define POTENTIOMETER_IN_MSG_CODE               68

#define POTENTIOMETER_SPI_MASTER_SPEED  500000

void potentiometer_init(void);
int8_t potentiometer_set(uint8_t value);
int8_t potentiometer_process_message(pb_istream_t *msg_stream, pb_ostream_t *resp_stream);

#endif /* POTENTIOMETER_H_ */