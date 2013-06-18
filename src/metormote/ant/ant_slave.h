#ifndef ANT_SLAVE_H
#define ANT_SLAVE_H

#include <stdint.h>
#include <stddef.h>
#include "rtc.h"
#include "mem.h"
#include "ant_core.h"
#include "ant_state.h"
#include "ant_msg.h"

//seconds
#define ANT_SLAVE_CHANNEL_TIMEOUT_HIGH   2
#define ANT_SLAVE_CHANNEL_TIMEOUT_LOW    2

#ifdef __cplusplus
extern "C" { 
#endif

typedef enum {
  ANT_SLAVE_EVENT_CLOSED=0,
  ANT_SLAVE_EVENT_DATA=1,
  ANT_SLAVE_EVENT_POLL_BEGIN=2,
  ANT_SLAVE_EVENT_POLL_END=3,
  ANT_SLAVE_EVENT_OTHER=4
} ant_slave_event_type;


struct ant_slave_msg {
  uint16_t shared_address;
  uint8_t  len;
  uint8_t  data[5];
  uint8_t  *burst;
} __attribute__((packed));


struct ant_slave_event_t {
  ant_slave_event_type type;
  struct ant_slave_msg *msg;
};

struct ant_slave_t {
  uint16_t          shared_address;
  uint8_t           device_type;
  uint32_t          device_id;
  //slave timeout in seconds
  uint16_t          timeout;
  //polling interval
  uint16_t          poll_interval;
  //no of consecutive poll messages sent
  uint8_t           no_of_polls;
  //last time received transmission
  uint32_t          last_rx;
} __attribute__((packed));


extern struct ant_slave_t ant_slave;

void ant_slave_init(void (*callback)(uint8_t, struct ant_slave_event_t *evt));
void ant_slave_reset(void);
int8_t ant_slave_open_channel(uint8_t channel, uint8_t frequency, uint16_t period, uint8_t node_id);
void ant_slave_start(uint8_t channel, uint16_t shared_address);
int8_t ant_slave_send(uint8_t channel, uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif


#endif /* ANT_SLAVE_H */
