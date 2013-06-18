/*
 * ant_state.h
 *
 * Created: 8/23/2011 1:30:46 PM
 *  Author: Administrator
 */ 

#ifndef ANT_STATE_H_
#define ANT_STATE_H_

#include <stdint.h>
#include <stddef.h>

#define ANT_MSG_TYPE_DATA                 1

#define ANT_NO_NETWORK                    0x0

#define ANT_MASTER_CHANNEL_NO             0
#define ANT_SLAVE_CHANNEL_NO              1

#define ANT_SCAN_CHANNEL_LOW              1
#define ANT_SCAN_CHANNEL_HIGH             5

#define ANT_TUNNEL_CHANNEL_NO             7

#define ANT_MASTER_CHANNEL_FREQ           66
#define ANT_MASTER_CHANNEL_PERIOD         0x1000
#define ANT_ROUTE_CHANNEL_PERIOD          0x1000

#define ANT_RECEIVE_TIMEOUT               60*SYSTEM_TICKS_PER_SEC
#define ANT_SEND_TIMEOUT                  60*SYSTEM_TICKS_PER_SEC
#define ANT_TX_POWER                      3

#define ANT_CONNECT_TIMEOUT               10*SYSTEM_TICKS_PER_SEC

#ifdef __cplusplus
extern "C" { 
#endif

typedef enum {
  ANT_DEVICE_TYPE_BASE            =1,
  ANT_DEVICE_TYPE_NODE            =2,
  ANT_DEVICE_TYPE_SLAVE           =3
} ant_device_type;

struct ant_state_t {
  ant_device_type   device_type;
  uint32_t          device_id;
  uint32_t          network_id;
  uint8_t           node_id;
};

extern struct ant_state_t ant_state;

#ifdef __cplusplus
}
#endif


#endif /* ANT_STATE_H_ */