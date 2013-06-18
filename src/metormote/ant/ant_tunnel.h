#ifndef ANT_TUNNEL_H
#define ANT_TUNNEL_H

#include <stdint.h>
#include <stddef.h>
#include "ant_core.h"
#include "ant_state.h"

#define ANT_TUNNEL_CHANNEL_PERIOD          0x1000
#define ANT_TUNNEL_CHANNEL_FREQ            88
#define ANT_TUNNEL_CHANNEL_TIMEOUT_HIGH    4
#define ANT_TUNNEL_CHANNEL_TIMEOUT_LOW     0


#ifdef __cplusplus
extern "C" { 
#endif

typedef enum {
  ANT_TUNNEL_EVENT_CLOSED              =0,
  ANT_TUNNEL_EVENT_DATA                =1
} ant_tunnel_event_type;

struct ant_tunnel_event {
  ant_tunnel_event_type     type;
} __attribute__((packed));


struct ant_tunnel_header {
  uint8_t      counter:7, fin:1;
  uint16_t     len;
  uint8_t      reserved[5];
};


void ant_tunnel_init(void (*callback)(uint8_t, struct ant_tunnel_event *));
int8_t ant_tunnel_open_channel(uint8_t channel, uint32_t device_id, bool master);
int8_t ant_tunnel_close_channel(uint8_t channel);
int8_t ant_tunnel_transfer(uint8_t channel, io_input_stream_t stream);


#ifdef __cplusplus
}
#endif


#endif /* ANT_TUNNEL_H */
