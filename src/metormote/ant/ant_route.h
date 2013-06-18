#ifndef ANT_ROUTE_H
#define ANT_ROUTE_H

#include <stdint.h>
#include <stddef.h>
#include "rtc.h"

#include "mem.h"
#include "ant_core.h"
#include "ant_master.h"
#include "ant_state.h"
#include "olsr.h"


#define ANT_ROUTE_CHANNEL_PERIOD          0x1000
#define ANT_ROUTE_CHANNEL_TIMEOUT_HIGH    2
#define ANT_ROUTE_CHANNEL_TIMEOUT_LOW     2

//if no broadcast message has been received from 
//the remote end after ANT_ROUTE_MAX_TX_COUNT packets
//then close the channel
#define ANT_ROUTE_MAX_TX_COUNT            128
#define ANT_ROUTE_TIMEOUT                 10


#ifdef __cplusplus
extern "C" { 
#endif

typedef enum {
  ANT_ROUTE_EVENT_CLOSED              =0,
  ANT_ROUTE_EVENT_DATA                =1,
  ANT_ROUTE_EVENT_NODE_ID_COLLISION   =2
} ant_route_event_type;

struct ant_route_event {
  ant_route_event_type  type;
  struct ant_data_msg          *msg;
} __attribute__((packed));

struct ant_node_list_entry {
  uint8_t channel;
  uint8_t tx_counter;
  uint32_t last_hello;
  struct ant_node node;
  struct ant_node_list_entry *next;
};

struct ant_route_t {
  //sequence number for tc messages
  uint8_t tc_seq_no;
  //sequence number for mid messages
  uint8_t mid_seq_no;
  //sequence number for hna messages
  uint8_t hna_seq_no;
  //sequence number for topology control
  uint8_t ansn;
  //used to send topology messages with the right interval
  uint32_t last_emit;
  //used to send topology messages in right order
  uint8_t emit_state;
  //index to keep track of sensors on a shared channel
  uint8_t mid_index;
  //node list
  struct ant_node_list_entry *node_list;
};

//ant message
struct ant_data_msg {
  uint8_t   ttl:4, type:4;
  uint8_t   src;
  uint8_t   dst;
  uint8_t   len;
  uint8_t   payload[4];
  uint8_t   *burst;
} __attribute__((packed));


void ant_route_init(void (*callback)(uint8_t, struct ant_route_event *));
void ant_route_reset(void);
int8_t ant_route_open_channel(uint8_t channel, struct ant_node *node, bool master);
int8_t ant_route_send(uint8_t destination, uint8_t *data, uint16_t len);
uint8_t ant_route_suggest_node_id(void);
bool ant_route_has_node(uint8_t node_id);
struct ant_node *ant_route_get_node(uint32_t device_id);
bool ant_route_check(uint8_t node_id);

#ifdef __cplusplus
}
#endif


#endif /* ANT_ROUTE_H */
