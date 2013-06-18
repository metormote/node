#ifndef ANT_H
#define ANT_H

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include "mem.h"
#include "atom.h"
#include "atomsem.h"
#include "atommutex.h"
#include "ant_core.h"
#include "ant_state.h"
#include "ant_scan.h"
#include "ant_route.h"
#include "ant_master.h"

#ifdef __cplusplus
extern "C" { 
#endif

typedef enum {
  ANT_EVENT_SLAVE_DATA = 1,
  ANT_EVENT_NETWORK_DATA = 2,
  ANT_EVENT_MASTER_DATA = 3,
  ANT_EVENT_POLL_BEGIN = 4,
  ANT_EVENT_POLL_END = 5,
  ANT_EVENT_SCAN_START = 6,
  ANT_EVENT_CONNECTED=7,
  ANT_EVENT_WAKEUP=8,
  ANT_EVENT_START=9
} ant_event_t;


void ant_init(uint8_t device_type, uint32_t device_id, uint32_t network_id);
void ant_start(void (*callback)(ant_event_t event, uint8_t *data, uint16_t));
void ant_reset(void);
void ant_standby(void);
void ant_resume(void);
int8_t ant_send_to_base(uint8_t *data, uint16_t len);
int8_t ant_send_to_device(uint32_t device_id, uint8_t *data, uint16_t len);
int8_t ant_scan_on(uint8_t channel);
int8_t ant_emit_on(uint8_t channel);
int8_t ant_emit_off(uint8_t channel);
uint32_t ant_get_network_id(void);
void ant_set_network_id(uint32_t network_id);
void ant_set_slave_control(uint16_t polling_interval, uint16_t timeout, uint8_t no_of_polls);

#ifdef __cplusplus
}
#endif


#endif /* ANT_H */
