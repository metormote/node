#ifndef OLSR_H
#define OLSR_H

#include <stdint.h>
#include <rtc.h>
#include "atom.h"
#include "atommutex.h"

#define OLSR_MAX_LINK              4
#define OLSR_MAX_NEIGHBOUR         4
#define OLSR_MAX_MPR               4
#define OLSR_MAX_2HOP              16
#define OLSR_MAX_TOPOLOGY          16
#define OLSR_MAX_HNA               4
#define OLSR_MAX_MID               32
#define OLSR_MAX_ROUTE_TABLE       16
#define OLSR_MAX_DUPLICATES        16

#define OLSR_MAX_HOPS              16

#define OLSR_HELLO_MESSAGE         1
#define OLSR_TC_MESSAGE            2
#define OLSR_MID_MESSAGE           3
#define OLSR_HNA_MESSAGE           4

//link type constants
#define OLSR_UNSPEC_LINK           0
#define OLSR_ASYM_LINK             1
#define OLSR_SYM_LINK              2
#define OLSR_LOST_LINK             3

//neigbour type constants
#define OLSR_NOT_NEIGH             0
#define OLSR_SYM_NEIGH             1
#define OLSR_MPR_NEIGH             2

//hysteresis constants
#define OLSR_HYST_THRESHOLD_HIGH   0.8
#define OLSR_HYST_THRESHOLD_LOW    0.3
#define OLSR_HYST_SCALING          0.5

//willingness constants
#define OLSR_WILL_NEVER            0
#define OLSR_WILL_LOW              1
#define OLSR_WILL_DEFAULT          3
#define OLSR_WILL_HIGH             6
#define OLSR_WILL_ALWAYS           7

//neighbour status constants
#define OLSR_NOT_SYM               0
#define OLSR_SYM                   1

#define OLSR_HELLO_INTERVAL        2 //seconds
#define OLSR_REFRESH_INTERVAL      2 //seconds
#define OLSR_TC_INTERVAL           8 //seconds
#define OLSR_MID_INTERVAL          8 //TC_INTERVAL
#define OLSR_HNA_INTERVAL          8 //TC_INTERVAL

#define OLSR_NEIGHB_HOLD_TIME      6 //3 x REFRESH_INTERVAL
#define OLSR_TOP_HOLD_TIME         15 //3 x TC_INTERVAL
#define OLSR_DUP_HOLD_TIME         30 //seconds
#define OLSR_MID_HOLD_TIME         15 //3 x MID_INTERVAL
#define OLSR_HNA_HOLD_TIME         15 //3 x HNA_INTERVAL

#define EXTRACT_BITS(val, start, len) ( val >> start ) & ~(0xff << len)


#ifdef __cplusplus
extern "C" { 
#endif


//link set entry
struct olsr_link {
  uint8_t   neighbour;
  uint8_t   channel;
  uint32_t  sym_time;
  uint32_t  asym_time;
  uint32_t  time; 
};

//neighbour set entry
struct olsr_neighbour {
  uint8_t  neighbour;
  uint8_t  mpr:1, status:4, willingness:4;
};

//2-hop neighbour set entry
struct olsr_2hop {
  uint8_t    neighbour;
  uint8_t    two_hop;
  uint32_t   time; 
};

//mpr selector set entry
struct olsr_mpr_select {
  uint8_t    neighbour;
  uint32_t   time;
};

//topology set entry
struct olsr_topology {
  uint8_t   dest;
  uint8_t   last;
  uint8_t   ansn; 
  uint32_t  time;
};

//mid set entry
struct olsr_mid {
  uint8_t   origin;
  uint8_t   device_type;
  uint32_t  device_id;
  uint32_t  time;
};

//hna set entry
struct olsr_hna {
  uint8_t   gateway;
  uint8_t   network_type;
  uint32_t  network_id;
  uint32_t  time;
};

//routing table entry
struct olsr_route {
  uint8_t  dest;
  uint8_t  next;
  uint8_t  hops;
};


//duplicate set entry
struct olsr_duplicate {
  uint8_t   origin;
  uint8_t   seq_no;
  uint8_t   retransmitted;
  uint8_t   source;
  uint32_t  time;
};

/******messages********/

//hello message
struct olsr_hello_msg {
  uint8_t   willingness:4, type:4; //HELLO_MESSAGE
  //uint8_t   vtime_h:4, vtime_l:4;
  uint8_t   origin;
 //four bitfields of length 2 each
  uint8_t   link_type;
  uint8_t   neighbour_type;
  uint8_t   neighbour[4];
} __attribute__((packed));

//topology message
struct olsr_tc_msg {
  uint8_t   ttl:4, type:4; //TC_MESSAGE
  //uint8_t   vtime_h:4, vtime_l:4;
  uint8_t   origin;
  uint8_t   seq_no;
  uint8_t   ansn;
  uint8_t   neighbour[4]; 
} __attribute__((packed));

//mid message
struct olsr_mid_msg {
  uint8_t   ttl:4, type:4; //MID_MESSAGE
  //uint8_t   vtime_h:4, vtime_l:4;
  uint8_t   origin;
  uint8_t   seq_no;
  uint8_t   device_type;
  uint32_t  device_id; 
} __attribute__((packed));

//hna message
struct olsr_hna_msg {
  uint8_t   ttl:4, type:4; //HNA_MESSAGE
  //uint8_t   vtime_h:4, vtime_l:4;
  uint8_t   origin;
  uint8_t   seq_no;
  uint8_t   network_type;
  uint32_t  network_id;
} __attribute__((packed));



//declare link set array
extern struct olsr_link olsr_link_set[OLSR_MAX_LINK];

//declare neighbour set array
extern struct olsr_neighbour olsr_neighbour_set[OLSR_MAX_NEIGHBOUR];

//declare 2-hop neighbour set array
extern struct olsr_2hop olsr_2hop_set[OLSR_MAX_2HOP];

//declare mpr selector set array
extern struct olsr_mpr_select olsr_mpr_select_set[OLSR_MAX_NEIGHBOUR];

//declare topology set array
extern struct olsr_topology olsr_topology_set[OLSR_MAX_TOPOLOGY];

//declare hna set array
extern struct olsr_hna olsr_hna_set[OLSR_MAX_HNA];

//declare mid set array
extern struct olsr_mid olsr_mid_set[OLSR_MAX_MID];

//declare routing table array
extern struct olsr_route olsr_route_table[OLSR_MAX_ROUTE_TABLE];

void olsr_remove_link(uint8_t channel);
void olsr_update(void);
void olsr_process_hello_msg(struct olsr_hello_msg *msg, uint8_t channel);
uint8_t olsr_process_tc_msg(struct olsr_tc_msg *msg, uint8_t channel);
uint8_t olsr_process_hna_msg(struct olsr_hna_msg *msg, uint8_t channel);
uint8_t olsr_process_mid_msg(struct olsr_mid_msg *msg, uint8_t channel);

uint8_t olsr_route_next_node(uint8_t dst);
uint8_t olsr_is_selected_mpr(void);
uint8_t olrs_find_node_id(uint8_t device_type, uint32_t device_id);

void olsr_init(uint8_t *n);


#ifdef __cplusplus
}
#endif


#endif /* OLSR_H */
