#include  "ant_slave.h"

struct ant_slave_t ant_slave;

static void ant_slave_event(struct ant_event *evt);

static void (*slave_callback)(uint8_t channel, struct ant_slave_event_t *evt);

void ant_slave_init(void (*callback)(uint8_t, struct ant_slave_event_t *evt)) {
  slave_callback=callback;
  ant_slave_reset();
}

void ant_slave_reset() {
}


int8_t ant_slave_open_channel(uint8_t channel, uint8_t frequency, uint16_t period, uint8_t node_id) {
  struct ant_config config;
  
  memset(&config, 0, sizeof(struct ant_config));
  
  config.master=0;
  config.shared=1;
  config.bg_scan=0;
  config.freq_agility=0;
  config.tx_type=3;
  config.pairing=0;
  config.device_type=1;
  config.device_no=node_id;
  config.frequency=frequency;
  config.period=period;
  config.tx_power=ANT_TX_POWER;
  config.timeout_hp=ANT_SLAVE_CHANNEL_TIMEOUT_HIGH;
  config.timeout_lp=ANT_SLAVE_CHANNEL_TIMEOUT_LOW;
  config.proximity=0;
  config.search_priority=128;
  
  if(ant_core_config(channel, &config)!=STATUS_OK) {
    return ERR_IO_ERROR;
  }
  
  return ant_core_open_channel(channel, &ant_slave_event);
}

void ant_slave_start(uint8_t channel, uint16_t shared_address) {
  ant_slave.shared_address=shared_address;
  ant_slave.last_rx=rtc_get_time();
  ant_core_channels[channel].callback=&ant_slave_event;
}

static void ant_slave_event(struct ant_event *evt) {
  struct ant_slave_event_t slave_evt;
  struct ant_slave_msg msg;
  
  switch(evt->event_code) {
    case ANT_DATA_BROADCAST:
    case ANT_DATA_ACKNOWLEDGE:
    case ANT_DATA_BURST_TRANSFER:
      msg.shared_address=evt->shared_address;
      evt->stream->read(evt->stream, &msg.len, 1);
      evt->stream->read(evt->stream, msg.data, 5);
      evt->data_len-=6;
      ant_slave.last_rx=rtc_get_time();
          
      switch(evt->event_code) {
        case ANT_DATA_BROADCAST:
          switch(msg.len) {
            case ANT_SLAVE_POLL_BEGIN_MESSAGE:
              slave_evt.type=ANT_SLAVE_EVENT_POLL_BEGIN;
              slave_evt.msg=&msg;
              slave_callback(evt->channel, &slave_evt);
              break;
            case ANT_SLAVE_POLL_MESSAGE:
              break;
            case ANT_SLAVE_POLL_END_MESSAGE:
              slave_evt.type=ANT_SLAVE_EVENT_POLL_END;
              slave_evt.msg=&msg;
              slave_callback(evt->channel, &slave_evt);
              break;
            default:
              slave_evt.type=ANT_SLAVE_EVENT_OTHER;
              slave_evt.msg=&msg;
              slave_callback(evt->channel, &slave_evt);
              break;
          }
          break;
        case ANT_DATA_ACKNOWLEDGE:
          slave_evt.type=ANT_SLAVE_EVENT_DATA;
          slave_evt.msg=&msg;
          slave_callback(evt->channel, &slave_evt);
          break;
        case ANT_DATA_BURST_TRANSFER:
          msg.burst=mem_safe_malloc(evt->data_len);
          if(msg.burst!=NULL) {
            evt->stream->read(evt->stream, msg.burst, evt->data_len);
            slave_evt.type=ANT_SLAVE_EVENT_DATA;
            slave_evt.msg=&msg;
            slave_callback(evt->channel, &slave_evt);
          }
          else {
            ant_core_clear_rx_stream(evt->data_len);
          }
          mem_safe_free(msg.burst);
          break;
      }
      break;
    case ANT_EVENT_RX_FAIL_GO_TO_SEARCH:
      ant_core_close_channel(evt->channel);
      break;
    case ANT_EVENT_CHANNEL_CLOSED:
      ant_core_unassign_channel(evt->channel);
      slave_evt.type=ANT_SLAVE_EVENT_CLOSED;
      slave_callback(evt->channel, &slave_evt);
      break;
    case ANT_EVENT_RX_FAIL:
    case ANT_EVENT_TRANSFER_TX_FAILED:
      slave_evt.type=ANT_SLAVE_EVENT_OTHER;
      slave_callback(evt->channel, &slave_evt);
    default:
      break;
  }
  return;
}



int8_t ant_slave_send(uint8_t channel, uint8_t *data, uint16_t len) {
  int8_t status;
  uint8_t *ptr;
  struct ant_slave_msg msg;
  if(!ant_slave.shared_address){
    return ERR_BAD_ADDRESS;
  }
  
  msg.shared_address=ant_slave.shared_address;
  msg.len=len;
  
  if(len<=5) {
    memcpy(msg.data, data, len);
    status=ant_core_send_acknowledged_data(channel, (uint8_t *)&msg);
  }
  else {
    ptr=(uint8_t *)&msg;
    memset(msg.data, 0, 5);
    status=ant_core_send_burst_transfer(channel, &msg.shared_address, ptr+2, data, len);
  }
  
  return status;
}

