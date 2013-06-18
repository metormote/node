/*
 * metormote_node_v2.h
 *
 * Created: 2/11/2012 4:54:03 PM
 *  Author: Administrator
 */ 


#ifndef METORMOTE_NODE_V2_H_
#define METORMOTE_NODE_V2_H_


#include "compiler.h"
#include "ioport.h"


/*! \name External oscillator
 */
//@{
//#define BOARD_XOSC_HZ          16000000
//#define BOARD_XOSC_TYPE        XOSC_TYPE_XTAL
//#define BOARD_XOSC_STARTUP_US  10
//@}

//#define F_CPU   32000000UL



/*! \name Power functions pin mapping
 */
//! @{
#define GPIO_POWER_PORT           (&PORTA)
//Battery power
#define GPIO_POWER_VBAT_SENSE     IOPORT_CREATE_PIN(PORTA,0)
//External power
#define GPIO_POWER_VUSB_SENSE     IOPORT_CREATE_PIN(PORTA,1)
//Power good
#define GPIO_POWER_GOOD           IOPORT_CREATE_PIN(PORTA,5)
#define GPIO_POWER_DCDC_ENABLE    IOPORT_CREATE_PIN(PORTA,6)

//! @}


/*! \name Temp sensor ADC pin number
 */
//! @{
// Use ADC0.
#define ADC                   &ADCA
#define GPIO_SENSOR_TEMP      IOPORT_CREATE_PIN(PORTA, 4)
//! @}


/*! \name TWI Connections of the sensor interface
 */
//! @{
#define SENSOR_TWI            &TWIC
#define SENSOR_TWI_SCL        IOPORT_CREATE_PIN(PORTC,1)
#define SENSOR_TWI_SDA        IOPORT_CREATE_PIN(PORTC,0)
//! @}


/*! \name SPI Connections of the sensor interface
 */
//! @{
#define SENSOR_SPI            &SPIE
#define SENSOR_CS             IOPORT_CREATE_PIN(PORTE,4)
#define SENSOR_MOSI           IOPORT_CREATE_PIN(PORTE,5)
#define SENSOR_MISO           IOPORT_CREATE_PIN(PORTE,6)
#define SENSOR_SCK            IOPORT_CREATE_PIN(PORTE,7)
//! @}


/*! \name UART Connections of the sensor interface
 */
//! @{
#define SENSOR_UART_RX        IOPORT_CREATE_PIN(PORTF,2)
#define SENSOR_UART_TX        IOPORT_CREATE_PIN(PORTF,3)
//! @}

/*! \name Sensor interface ad/da pin mapping
 */
//! @{
#define SENSOR_ADC0           IOPORT_CREATE_PIN(PORTA,2)
#define SENSOR_ADC1           IOPORT_CREATE_PIN(PORTA,3)
#define SENSOR_DAC0           IOPORT_CREATE_PIN(PORTB,1)
#define SENSOR_DAC1           IOPORT_CREATE_PIN(PORTB,2)
//! @}


/*! \name GPIO Connections of LEDs
 */
//! @{
//! Number of LEDs.
#define LED_COUNT   4

#define GPIO_LED_GREEN        IOPORT_CREATE_PIN(PORTB,4)
#define GPIO_LED_RED          IOPORT_CREATE_PIN(PORTB,5)
#define GPIO_LED_YELLOW       IOPORT_CREATE_PIN(PORTB,6)
#define GPIO_LED_BLUE         IOPORT_CREATE_PIN(PORTB,7)
//! @}


/*! \name GPIO Connections of Push Buttons
 */
//! @{
#define GPIO_SWITCH_PORT          (&PORTB)
#define GPIO_SWITCH               IOPORT_CREATE_PIN(PORTB,3)
//#define GPIO_SWITCH_INTMSK      0x08
#define GPIO_SWITCH_INT_VECTOR    PORTB_INT0_vect
//! @}


/*! \name SPI Connections of the AT45DBX Data Flash Memory
 */
//! @{
#define AT45DBX_SPI           &SPIC
#define AT45DBX_SPI_CLK_MASK  SYSCLK_PORT_C
#define AT45DBX_CS            IOPORT_CREATE_PIN(PORTD,4)  // CS as output
#define AT45DBX_MASTER_MOSI   IOPORT_CREATE_PIN(PORTD,5)  // MOSI as output
#define AT45DBX_MASTER_MISO   IOPORT_CREATE_PIN(PORTD,6)  // MISO as input
#define AT45DBX_MASTER_SCK    IOPORT_CREATE_PIN(PORTD,7)  // SCK as output
#define AT45DBX_RESET         IOPORT_CREATE_PIN(PORTC,4)  // RESET as output

//! @}


/*! \name GPIO Connections of ANT
 */
//! @{
#define CONF_BOARD_ENABLE_ANT
#define ANT_USART            (&USARTD0)
#define ANT_RX_PORT          (&PORTD)
#define ANT_RTS_PORT         (&PORTF)
#define ANT_RX_INT_VECTOR    PORTD_INT0_vect
#define ANT_RTS_INT_VECTOR   PORTF_INT0_vect
#define nRF24AP2_nSUSPEND    IOPORT_CREATE_PIN(PORTF, 4)
#define nRF24AP2_SLEEP       IOPORT_CREATE_PIN(PORTF, 5)
#define nRF24AP2_RTS         IOPORT_CREATE_PIN(PORTF, 6)
#define nRF24AP2_nRESET      IOPORT_CREATE_PIN(PORTF, 7)
#define nRF24AP2_TX          IOPORT_CREATE_PIN(PORTD, 2)
#define nRF24AP2_RX          IOPORT_CREATE_PIN(PORTD, 3)
//! @}


/*! \name GPIO Connections of the nRF8001 Bluetooth Low Energy chipset
 */
//! @{
#define CONF_BOARD_ENABLE_BLE
#define BLE_SPI             &SPIC
#define BLE_SPI_CLK_MASK    SYSCLK_PORT_C
#define BLE_RDYN_PORT       (&PORTE)
#define BLE_MASTER_MOSI     IOPORT_CREATE_PIN(PORTC,5)  // MOSI as output
#define BLE_MASTER_MISO     IOPORT_CREATE_PIN(PORTC,6)  // MISO as input
#define BLE_MASTER_SCK      IOPORT_CREATE_PIN(PORTC,7)  // SCK as output
#define BLE_ACTIVE          IOPORT_CREATE_PIN(PORTD,0)  // RDYN as input
#define BLE_RESET           IOPORT_CREATE_PIN(PORTD,1)  // RDYN as input
#define BLE_RDYN            IOPORT_CREATE_PIN(PORTE,0)  // RDYN as input
#define BLE_REQN            IOPORT_CREATE_PIN(PORTE,1)  // REQN as output
#define BLE_RDYN_INT_VECTOR PORTE_INT0_vect
#define BLE_SPI_INT_VECTOR  SPIC_INT_vect

//uart ble test interface
#define BLE_USART            &USARTE0
#define BLE_TXD             IOPORT_CREATE_PIN(PORTE,2)  // RDYN as input
#define BLE_RXD             IOPORT_CREATE_PIN(PORTE,3)  // RDYN as input
//! @}




static inline void board_init(void)
{
  ioport_set_pin_dir(GPIO_POWER_VBAT_SENSE, IOPORT_DIR_INPUT);
  ioport_set_pin_dir(GPIO_POWER_VUSB_SENSE, IOPORT_DIR_INPUT);
  ioport_set_pin_dir(GPIO_POWER_GOOD, IOPORT_DIR_INPUT);
  ioport_set_pin_dir(GPIO_POWER_DCDC_ENABLE, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(GPIO_POWER_DCDC_ENABLE, false);
  
  ioport_set_pin_dir(GPIO_LED_RED, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(GPIO_LED_RED, true);
  ioport_set_pin_dir(GPIO_LED_GREEN, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(GPIO_LED_GREEN, true);
  ioport_set_pin_dir(GPIO_LED_YELLOW, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(GPIO_LED_YELLOW, true);
  ioport_set_pin_dir(GPIO_LED_BLUE, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(GPIO_LED_BLUE, true);
  
  ioport_set_pin_dir(GPIO_SWITCH, IOPORT_DIR_INPUT);
  ioport_set_pin_dir(GPIO_SENSOR_TEMP, IOPORT_DIR_INPUT);
  
  ioport_set_pin_dir(SENSOR_ADC0, IOPORT_DIR_INPUT);
  ioport_set_pin_dir(SENSOR_ADC1, IOPORT_DIR_INPUT);
  ioport_set_pin_dir(SENSOR_DAC1, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(SENSOR_DAC1, false);
  ioport_set_pin_dir(SENSOR_DAC0, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(SENSOR_DAC0, false);
  
// RF
#ifdef CONF_BOARD_ENABLE_ANT
  ioport_set_pin_dir(nRF24AP2_TX, IOPORT_DIR_INPUT);
  ioport_set_pin_dir(nRF24AP2_RX, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(nRF24AP2_RX, true);
  // configure nRESET for rf...
  ioport_set_pin_dir(nRF24AP2_nRESET, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(nRF24AP2_nRESET, true);
  // configure nSUSPEND for rf...
  ioport_set_pin_dir(nRF24AP2_nSUSPEND, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(nRF24AP2_nSUSPEND, true);
  // configure SLEEP for rf...
  ioport_set_pin_dir(nRF24AP2_SLEEP, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(nRF24AP2_SLEEP, false);
  // configure RTS for rf...
  ioport_set_pin_dir(nRF24AP2_RTS, IOPORT_DIR_INPUT);
#endif


#ifdef CONF_BOARD_ENABLE_BLE
  ioport_set_pin_dir(BLE_RESET, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(BLE_RESET, true);
  ioport_set_pin_dir(BLE_ACTIVE, IOPORT_DIR_INPUT);
  ioport_set_pin_dir(BLE_RDYN, IOPORT_DIR_INPUT);
  ioport_set_pin_dir(BLE_REQN, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(BLE_REQN, true);
  ioport_set_pin_dir(BLE_MASTER_SCK, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(BLE_MASTER_SCK, true);
  ioport_set_pin_dir(BLE_MASTER_MOSI, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(BLE_MASTER_MOSI, true);
  ioport_set_pin_dir(BLE_MASTER_MISO, IOPORT_DIR_INPUT);
  ioport_set_pin_dir(BLE_TXD, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(BLE_TXD, true);
  ioport_set_pin_dir(BLE_RXD, IOPORT_DIR_INPUT);
#endif

}

#endif /* METORMOTE_NODE_V2_H_ */