/*
 * ble.h
 *
 * Created: 10/14/2012 12:54:18 PM
 *  Author: Administrator
 */ 
#ifndef BLE_H_
#define BLE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "ble/ble_core.h"
#include "utils/mem.h"

#define BLE_BLOCK_LEN   20

#define BLE_PIPE_PROTOBUF_IN      1
#define BLE_PIPE_PROTOBUF_OUT     2
#define BLE_PIPE_DEVICE_ID        3

typedef enum {
  BLE_EVENT_CONNECTING = 1,
  BLE_EVENT_CONNECTED = 2,
  BLE_EVENT_DISCONNECT = 3,
  BLE_EVENT_DATA = 4
} ble_events_t;

#ifdef __cplusplus
extern "C" { 
#endif

void ble_init(void);
int8_t ble_reset(void);
void ble_standby(void);
void ble_resume(void);
int8_t ble_start(void (*callback)(ble_events_t event, uint8_t *data, uint16_t));
bool ble_is_connected(void);
int8_t ble_send(uint8_t *data, uint16_t len);
int8_t ble_set_device_id(uint64_t device_id);

#ifdef __cplusplus
}
#endif

#endif /* BLE_H_ */