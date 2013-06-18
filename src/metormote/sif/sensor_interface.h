/*
 * sensor_interface.h
 *
 * Created: 10/6/2012 11:42:32 PM
 *  Author: Administrator
 */ 


#ifndef SENSOR_INTERFACE_H_
#define SENSOR_INTERFACE_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include "gpio.h"
#include "status_codes.h"
#include "board.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "gpio.pb.h"


#define GPIO_STATE_MSG_CODE              75

#ifdef __cplusplus
extern "C" { 
#endif

/* Pins
1.  VLD0 3.3V OUT
2.  GND
3.  IO_ADC0
4.  IO_ADC1
5.  IO_DAC0
6.  IO_DAC1
7.  SPI_MISO
8.  SPI_MOSI
9.  SPI_SCK
10. SPI_CS
11. TWI_SDA
12. TWI_SCL
13. UART_TX
14. UART_RX
15. POWER_IN
16. GND
17. PDI_DATA
18. 3.3V MCU
19. RESET/PDI_CLK
20. GND
*/

void sif_init(void);
int8_t sif_process_gpio_message(pb_istream_t *msg_stream, pb_ostream_t *resp_stream);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_INTERFACE_H_ */
