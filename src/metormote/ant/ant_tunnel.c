#include  "ant_tunnel.h"

static void tunnel_event(struct ant_event *evt);

//callback for tunnel events
static void (*tunnel_callback)(uint8_t channel, struct ant_tunnel_event *);


void ant_tunnel_init(void (*callback)(uint8_t, struct ant_tunnel_event *)) {
  tunnel_callback=callback;
}


int8_t ant_tunnel_open_channel(uint8_t channel, uint32_t device_id, bool master) {
  int8_t status;
  struct ant_config config;
  
  memset(&config, 0, sizeof(struct ant_config));
  
  config.master=master;
  config.shared=0;
  config.bg_scan=0;
  config.freq_agility=0;
  config.pairing=(device_id >> 31);
  config.device_type=(uint8_t)((device_id >> 24) & 0x7F);
  config.tx_type=(uint8_t)(device_id >> 16);
  config.device_no=(uint16_t)device_id;
  config.frequency=ANT_TUNNEL_CHANNEL_FREQ;
  config.tx_power=ANT_TX_POWER;
  config.period=ANT_TUNNEL_CHANNEL_PERIOD;
  config.timeout_hp=ANT_TUNNEL_CHANNEL_TIMEOUT_HIGH;
  config.timeout_lp=ANT_TUNNEL_CHANNEL_TIMEOUT_LOW;
  config.proximity=0;
  config.search_priority=16;
  
  status=ant_core_config(channel, &config);
  if(status!=STATUS_OK) {
    return ERR_IO_ERROR;
  }
  
  status=ant_core_open_channel(channel, &tunnel_event);
  
  return status;
}


int8_t ant_tunnel_close_channel(uint8_t channel) {
  return ant_core_close_channel(channel);
}



int8_t ant_tunnel_transfer(uint8_t channel, io_input_stream_t stream) {
  int8_t status;
  uint8_t i;
  struct ant_tunnel_header header;
  uint8_t *buf;
  
  memset(&header, 0, sizeof(struct ant_tunnel_header));
  
  for(i=0;i<5;i++) {
    status=ant_core_send_acknowledged_data(channel, (uint8_t *)&header);
    if(status==ANT_EVENT_TRANSFER_TX_COMPLETED) {
      status=STATUS_OK;
      break;
    }
    atomTimerDelay(SYSTEM_TICKS_PER_SEC);
  }
  if(status!=STATUS_OK) {
    return ERR_TIMEOUT;
  }
  
  buf=mem_safe_malloc(256);
  if(buf==NULL) {
    return ERR_NO_MEMORY;
  }
  
  while(stream.bytes_left>0) {
    header.len = stream.bytes_left>256 ? 256 : (uint16_t)stream.bytes_left;
    stream.read(&stream, buf, header.len);
    if(!stream.bytes_left) {
      header.fin=1;
    }
    for(i=0;i<10;i++) {
      status=ant_core_send_burst_transfer(channel, NULL, (uint8_t *)&header, buf, header.len);
      if(status==ANT_EVENT_TRANSFER_TX_COMPLETED) {
        status=STATUS_OK;
        break;
      }
      atomTimerDelay(10);
    }
    if(status!=STATUS_OK) {
      break;
    }
    header.counter++;
    atomTimerDelay(1);
  }
  
  mem_safe_free(buf);
  
  return status;
}



static void tunnel_event(struct ant_event *evt) {
  struct ant_tunnel_event tunnel_event;
  
  switch(evt->event_code) {
    case ANT_DATA_BROADCAST:
    case ANT_DATA_ACKNOWLEDGE:
    case ANT_DATA_BURST_TRANSFER:
      break;
    case ANT_EVENT_TX:
      break;
    case ANT_EVENT_CHANNEL_CLOSED:
      ant_core_unassign_channel(evt->channel);
      tunnel_event.type=ANT_TUNNEL_EVENT_CLOSED;
      if(tunnel_callback) {
        tunnel_callback(evt->channel, &tunnel_event);
      }
      break;
    case ANT_EVENT_RX_FAIL_GO_TO_SEARCH:
      ant_core_close_channel(evt->channel);
      break;
  }
  return;
}

