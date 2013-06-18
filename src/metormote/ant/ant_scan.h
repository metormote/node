/*
 * ant_scan.h
 *
 * Created: 2/25/2013 10:00:19 PM
 *  Author: Administrator
 */ 
#ifndef ANT_SCAN_H_
#define ANT_SCAN_H_

#include <stdint.h>
#include <stddef.h>
#include "ant_core.h"
#include "ant_msg.h"
#include "ant_route.h"

#define ANT_SCAN_CHANNEL_TIMEOUT_HIGH   2
#define ANT_SCAN_CHANNEL_TIMEOUT_LOW    2
#define ANT_SCAN_TX_POWER               3

#ifdef __cplusplus
extern "C" { 
#endif

typedef enum {
  ANT_SCAN_EVENT_CLOSED=0,
  ANT_SCAN_EVENT_NETWORK_FOUND=1,
  ANT_SCAN_EVENT_NODE_FOUND=2,
  ANT_SCAN_EVENT_MASTER_CONNECTED=3,
  ANT_SCAN_EVENT_NODE_CONNECTED=4,
  ANT_SCAN_EVENT_ERROR=9,
} ant_scan_event_type;

typedef enum {
  ANT_SCAN_CONNECT_MASTER=0,
  ANT_SCAN_CONNECT_NODE=1
} ant_scan_connect_target;

typedef enum {
  ANT_SCAN_CONNECT_STATE_CLOSED=0,
  ANT_SCAN_CONNECT_STATE_INITIAL=1,
  ANT_SCAN_CONNECT_STATE_NETWORK_FOUND=2,
  ANT_SCAN_CONNECT_STATE_REQUESTING=3,
  ANT_SCAN_CONNECT_STATE_CONNECTED=4,
  ANT_SCAN_CONNECT_STATE_TRANSITION_TO_ROUTE=5
} ant_scan_connect_state;


struct ant_scan_event {
  ant_scan_event_type type;
  uint8_t             node_id;
  uint32_t            network_id;
};

struct ant_scan_connect_state_t {
  ant_scan_connect_target     target;
  uint8_t                     state;
  uint32_t                    network_id;
  struct ant_node connecting_node;
  struct ant_slave_t connecting_slave;
};


void ant_scan_init(void (*callback)(uint8_t channel, struct ant_scan_event *evt));
void ant_scan_reset(void);
int8_t ant_scan_start(uint8_t channel, uint8_t frequency, uint16_t period);
int8_t ant_scan_connect(uint8_t channel, ant_scan_connect_target target);
void ant_scan_stop(uint8_t channel);

#ifdef __cplusplus
}
#endif




#endif /* ANT_SCAN_H_ */