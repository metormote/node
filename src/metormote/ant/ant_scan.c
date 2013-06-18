/*
 * ant_scan.c
 *
 * Created: 2/25/2013 10:00:06 PM
 *  Author: Administrator
 */ 
#include "ant_scan.h"

static struct ant_scan_connect_state_t connect_state;

static void (*scan_callback)(uint8_t channel, struct ant_scan_event *event);

static int8_t connect(uint8_t channel);

static void scan_event_callback(struct ant_event *evt);
static void connect_event_callback(struct ant_event *evt);
static void ant_connect_data(struct ant_event *evt);

void ant_scan_init(void (*callback)(uint8_t channel, struct ant_scan_event *evt)) {
  scan_callback=callback;
  ant_scan_reset();
}


void ant_scan_reset() {
}


int8_t ant_scan_start(uint8_t channel, uint8_t frequency, uint16_t period) {
  struct ant_config config;
  int8_t status;
  
  memset(&config, 0, sizeof(struct ant_config));
  memset(&connect_state, 0, sizeof(struct ant_scan_connect_state_t));
  connect_state.state=ANT_SCAN_CONNECT_STATE_CLOSED;
  connect_state.connecting_node.freq[0]=frequency;
  connect_state.connecting_node.period=period;
  
  config.master=0;
  config.shared=0;
  config.bg_scan=1;
  config.freq_agility=0;
  config.tx_type=0;
  config.pairing=0;
  config.device_type=0;
  config.device_no=0; 
  config.timeout_hp=255;
  config.timeout_lp=255;
  config.tx_power=ANT_SCAN_TX_POWER;
  config.frequency=frequency;
  config.period=period;
  config.proximity=0;
  config.search_priority=1;
  
  status=ant_core_config(channel, &config);
  if(status!=STATUS_OK) {
    return status;
  }
  
  status=ant_core_open_channel(channel, &scan_event_callback);
  
  return status;
}


int8_t ant_scan_connect(uint8_t channel, ant_scan_connect_target target) {
  connect_state.target=target;
  connect_state.state=ANT_SCAN_CONNECT_STATE_INITIAL;
  return ant_core_close_channel(channel);
}


void ant_scan_stop(uint8_t channel) {
  ant_core_close_channel(channel);
}


static void scan_event_callback(struct ant_event *evt) {
  uint8_t data[8];
  uint8_t msg_type;
  struct ant_scan_event scan_evt;
  
  if(evt->data_len>8) {
    ant_core_clear_rx_stream(evt->data_len);
    return;
  }
  
  switch(evt->event_code) {
    case ANT_DATA_BROADCAST:
      evt->stream->read(evt->stream, data, evt->data_len);
      evt->shared_address=data[0];
      evt->shared_address|=((0xFFFF & data[1]) << 8);
      if(evt->shared_address!=0xFFFF) {
        break;
      }
      msg_type=data[2];
      
      switch(msg_type) {
        case ANT_NETWORK_INFO_MESSAGE:
        case ANT_CHANNEL_AVAILABLE_MESSAGE:
          scan_evt.node_id=data[3];
          scan_evt.network_id = data[4];
          scan_evt.network_id |= ((0xFFFFFFFF & data[5]) << 8);
          scan_evt.network_id |= ((0xFFFFFFFF & data[6]) << 16);
          scan_evt.network_id |= ((0xFFFFFFFF & data[7]) << 24);
          scan_evt.type=msg_type==ANT_NETWORK_INFO_MESSAGE ? ANT_SCAN_EVENT_NETWORK_FOUND : ANT_SCAN_EVENT_NODE_FOUND;
          connect_state.connecting_node.node_id=scan_evt.node_id;
          connect_state.network_id=scan_evt.network_id;
          scan_callback(evt->channel, &scan_evt);
      }
      break;
    case ANT_EVENT_CHANNEL_CLOSED:
      ant_core_unassign_channel(evt->channel);
      switch(connect_state.state) {
        case ANT_SCAN_CONNECT_STATE_INITIAL:
          if(connect(evt->channel)!=STATUS_OK) {
            scan_evt.type=ANT_SCAN_EVENT_ERROR;
            ant_core_channels[evt->channel].callback=NULL;
            scan_callback(evt->channel, &scan_evt);
          }
          break;
        default:
          scan_evt.type=ANT_SCAN_EVENT_CLOSED;
          ant_core_channels[evt->channel].callback=NULL;
          scan_callback(evt->channel, &scan_evt);
      }
      break;
  }
  return;
}


static int8_t connect(uint8_t channel) {
  int8_t status;
  struct ant_config config;
  
  memset(&config, 0, sizeof(config));
  
  config.master=0;
  config.shared=1;
  config.bg_scan=0;
  config.freq_agility=0;
  config.tx_type=0;
  config.pairing=0;
  config.device_type=0;
  config.device_no=connect_state.connecting_node.node_id;
  config.frequency=connect_state.connecting_node.freq[0];
  config.period=connect_state.connecting_node.period;
  config.timeout_hp=ANT_SCAN_CHANNEL_TIMEOUT_HIGH;
  config.timeout_lp=ANT_SCAN_CHANNEL_TIMEOUT_LOW;
  config.tx_power=ANT_SCAN_TX_POWER;
  config.proximity=0;
  config.search_priority=128;
  
  status=ant_core_config(channel, &config);
  if(status!=STATUS_OK) {
    return status;
  }
  
  status=ant_core_open_channel(channel, &connect_event_callback);
  if(status==STATUS_OK) {
    ant_core_set_shared_address_msg(channel, 0xFFFF);
  }
  
  return status;
}


static void connect_event_callback(struct ant_event *evt) {
  struct ant_scan_event scan_evt;
  
  switch(evt->event_code) {
    case ANT_DATA_BROADCAST:
    case ANT_DATA_ACKNOWLEDGE:
      ant_connect_data(evt);
      break;
    case ANT_EVENT_RX_FAIL_GO_TO_SEARCH:
      ant_core_close_channel(evt->channel);
      break;
    case ANT_EVENT_CHANNEL_CLOSED:
      ant_core_unassign_channel(evt->channel);
      ant_core_channels[evt->channel].callback=NULL;
      if(connect_state.state==ANT_SCAN_CONNECT_STATE_TRANSITION_TO_ROUTE) {
        if(ant_route_open_channel(evt->channel, &connect_state.connecting_node, false)==STATUS_OK) {
          scan_evt.type=ANT_SCAN_EVENT_NODE_CONNECTED;
          scan_evt.network_id=connect_state.network_id;
          scan_evt.node_id=connect_state.connecting_node.node_id;
          connect_state.state=ANT_SCAN_CONNECT_STATE_CONNECTED;
        }
        else {
          scan_evt.type=ANT_SCAN_EVENT_ERROR;
          connect_state.state=ANT_SCAN_CONNECT_STATE_CLOSED;
        }
      }
      else {
        scan_evt.type=ANT_SCAN_EVENT_CLOSED;
        connect_state.state=ANT_SCAN_CONNECT_STATE_CLOSED;
      }
      scan_callback(evt->channel, &scan_evt);
      break;
  }
  return;
}


static void ant_connect_data(struct ant_event *evt) {
  uint8_t data[6];
  uint8_t msg_type, channel, node_id;
  uint32_t network_id;
  struct ant_scan_event scan_evt;
  
  if(evt->data_len>6) {
    ant_core_clear_rx_stream(evt->data_len);
    return;
  }
  channel=evt->channel;
  evt->stream->read(evt->stream, data, evt->data_len);
  msg_type=data[0];
  
  switch(connect_state.state) {
    case ANT_SCAN_CONNECT_STATE_INITIAL:
      switch(msg_type) {
        case ANT_NETWORK_INFO_MESSAGE:
          if(connect_state.target!=ANT_SCAN_CONNECT_MASTER) {
            break;
          }
        case ANT_CHANNEL_AVAILABLE_MESSAGE:
          node_id=data[1];
          network_id = data[2];
          network_id |= ((0xFFFFFFFF & data[3]) << 8);
          network_id |= ((0xFFFFFFFF & data[4]) << 16);
          network_id |= ((0xFFFFFFFF & data[5]) << 24);
          
          if((connect_state.connecting_node.node_id!=node_id) ||
            (connect_state.network_id!=ANT_NO_NETWORK && connect_state.network_id!=network_id)) {
            break;
          }
          
          connect_state.connecting_node.node_id=node_id;
          connect_state.network_id=network_id;
          
          if(connect_state.target==ANT_SCAN_CONNECT_NODE) {
            if(send_request_channel_msg(channel, ant_state.node_id, ant_state.device_id)==ANT_EVENT_TRANSFER_TX_COMPLETED) {
              connect_state.state=ANT_SCAN_CONNECT_STATE_REQUESTING;
              ant_core_set_shared_address_msg(evt->channel, 0xFFFE);
            }                
          }
          else if(connect_state.target==ANT_SCAN_CONNECT_MASTER) {
            connect_state.state=ANT_SCAN_CONNECT_STATE_NETWORK_FOUND;
          }
      }
      break;
    case ANT_SCAN_CONNECT_STATE_NETWORK_FOUND:
      if(msg_type==ANT_ADDRESS_AVAILABLE_MESSAGE) {
        if(send_request_address_msg(channel, ant_state.device_id)==ANT_EVENT_TRANSFER_TX_COMPLETED) {
          connect_state.state=ANT_SCAN_CONNECT_STATE_REQUESTING;
          connect_state.connecting_slave.shared_address = data[4];
          connect_state.connecting_slave.shared_address |= ((0xFFFF & data[5]) << 8);
          ant_core_set_shared_address_msg(evt->channel, 0xFFFE);
        }
        else {
          connect_state.state=ANT_SCAN_CONNECT_STATE_INITIAL;
        }          
      }
      break;
    case ANT_SCAN_CONNECT_STATE_REQUESTING:
      switch(msg_type) {
        case ANT_BUSY_AQUIRING_MESSAGE:
          if(data[2]==((uint8_t)ant_state.device_id) && 
              data[3]==((uint8_t)(ant_state.device_id >> 8)) && 
                  data[4]==((uint8_t)(ant_state.device_id >> 16)) && 
                      data[5]==((uint8_t)(ant_state.device_id >> 24))) {
                      
            if(send_confirm_aquire_msg(channel, ant_state.device_id)==ANT_EVENT_TRANSFER_TX_COMPLETED) {
              if(connect_state.target==ANT_SCAN_CONNECT_MASTER) {
                ant_core_set_shared_address_msg(evt->channel, connect_state.connecting_slave.shared_address);
                connect_state.state=ANT_SCAN_CONNECT_STATE_CONNECTED;
                ant_slave_start(evt->channel, connect_state.connecting_slave.shared_address);
                scan_evt.type=ANT_SCAN_EVENT_MASTER_CONNECTED;
                scan_evt.network_id=connect_state.network_id;
                scan_callback(evt->channel, &scan_evt);
              }              
            }            
          }
          break;
        case ANT_CHANNEL_INFO_MESSAGE:
          //frequency and period of the available channel
          connect_state.connecting_node.freq[0]=data[1];
          connect_state.connecting_node.freq[1]=data[2];
          connect_state.connecting_node.freq[2]=data[3];
          connect_state.connecting_node.period = data[4];
          connect_state.connecting_node.period |= ((0xFFFF & data[5]) << 8);
          
          if(send_confirm_aquire_msg(channel, ant_state.device_id)==ANT_EVENT_TRANSFER_TX_COMPLETED) {
            connect_state.state=ANT_SCAN_CONNECT_STATE_TRANSITION_TO_ROUTE;
          }
          ant_core_close_channel(channel);
      }
      break;      
  }
}


