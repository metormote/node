#ifndef ANT_MASTER_H
#define ANT_MASTER_H

#include <stdint.h>
#include <stddef.h>
#include <rtc.h>

#include "mem.h"
#include "ant_core.h"
#include "ant_state.h"
#include "ant_msg.h"
#include "ant_slave.h"
#include "ant_slave.h"

#define ANT_MASTER_CHANNEL_TIMEOUT_LOW      2
#define ANT_MASTER_CHANNEL_TIMEOUT_HIGH     2

#define ANT_MASTER_SEARCH_TIMEOUT           16
#define ANT_MASTER_SENSOR_TIMEOUT           300
#define ANT_MASTER_SENSOR_POLL_INTERVAL     10
#define ANT_MASTER_SENSOR_NO_OF_POLLS       8

#ifdef __cplusplus
extern "C" { 
#endif

typedef enum {
  ANT_MASTER_STATE_AVAILABLE=0,
  ANT_MASTER_STATE_ASSIGNING_ADDRESS=1,
  ANT_MASTER_STATE_ASSIGNING_CHANNEL=2,
  ANT_MASTER_STATE_CHANNEL_ASSIGNED=3,
  ANT_MASTER_STATE_CHANNEL_CONFIRMED=4
} ant_master_state;

struct ant_slave_list_entry {
  struct ant_slave_t slave;
  struct ant_slave_list_entry *next;
  //next time we poll the slave
  uint32_t          next_poll;
  //no of sent poll messages
  uint8_t           poll_count;
};

struct ant_node {
  //uint8_t     channel;
  uint8_t     node_id;
  uint32_t    device_id;
  uint8_t     freq[3];
  uint16_t    period;
};

struct ant_master_t {
  ant_master_state state;
  uint8_t tx_counter;
  struct ant_node connecting_node;
  struct ant_slave_t connecting_slave;
  struct ant_slave_list_entry *current;
  struct ant_slave_list_entry *slave_list;
};

typedef enum {
  ANT_MASTER_EVENT_CLOSED=0,
  ANT_MASTER_EVENT_DATA=1,
  ANT_MASTER_EVENT_NEW_SLAVE=2,
  ANT_MASTER_EVENT_NEW_NODE=3
} ant_master_event_type;

struct ant_master_event {
  ant_master_event_type type;
  struct ant_slave_t *slave;
  struct ant_node *node;
  struct ant_slave_msg *msg;
};

extern struct ant_master_t ant_master;

void ant_master_init(void (*callback)(uint8_t channel, struct ant_master_event *evt));
void ant_master_reset(void);
uint8_t ant_master_has_slave(uint32_t device_id);
int8_t ant_master_open_channel(uint8_t channel, uint8_t frequency, uint16_t period);
int8_t ant_master_send(uint8_t channel, uint32_t device_id, uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif


#endif /* ANT_MASTER_H */
