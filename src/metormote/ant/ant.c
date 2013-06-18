#include "ant.h"

static void start_callback(void);
static void scan_callback(uint8_t channel, struct ant_scan_event *evt);
static void master_callback(uint8_t channel, struct ant_master_event *evt);
static void slave_callback(uint8_t channel, struct ant_slave_event_t *evt);
static void route_callback(uint8_t channel, struct ant_route_event *evt);


//callback for data
static void (*ant_callback)(ant_event_t event, uint8_t *data, uint16_t len);

uint32_t ant_get_network_id(void) {
  return ant_state.network_id;
}

void ant_set_network_id(uint32_t nid) {
  ant_state.network_id=nid;
}

void ant_init(uint8_t device_type, uint32_t device_id, uint32_t network_id) {
  ant_state.device_type=device_type;
  ant_state.device_id=device_id;
  ant_state.network_id=network_id;
  
  ant_core_init();
  ant_scan_init(&scan_callback);
  ant_master_init(&master_callback);
  ant_slave_init(&slave_callback);
  ant_route_init(&route_callback);
  
  //we assign a random node id and handle collisions later
  ant_state.node_id=ant_route_suggest_node_id();
}

void ant_reset() {
  ant_scan_reset();
  ant_master_reset();
  ant_slave_reset();
  ant_route_reset();
  ant_core_reset();
}

void ant_standby() {
  ant_core_sleep(TRUE);
}

void ant_resume() {
  ant_core_sleep(FALSE);
}

void ant_start(void (*callback)(ant_event_t event, uint8_t *data, uint16_t len)) {
  
  ant_callback=callback;
  
  for(;;) {
    ant_core_start(&start_callback);
  }
}

static void start_callback() {
  
  switch(ant_state.device_type) {
    case ANT_DEVICE_TYPE_BASE:
      ant_emit_on(ANT_MASTER_CHANNEL_NO);
      break;
    case ANT_DEVICE_TYPE_NODE:
    case ANT_DEVICE_TYPE_SLAVE:
    default:
      ant_scan_on(ANT_SLAVE_CHANNEL_NO);
  }
  
  ant_callback(ANT_EVENT_START, NULL, 0);
}

int8_t ant_emit_on(uint8_t channel) {
  int8_t status;
  
  if(ant_core_channel_is_open(channel)) {
    return ERR_BUSY;
  }
  
  status=ant_master_open_channel(channel, ANT_MASTER_CHANNEL_FREQ, ANT_MASTER_CHANNEL_PERIOD);
  
  return status;
}

int8_t ant_emit_off(uint8_t channel) {
  
  if(ant_core_channel_is_open(channel)) {
    return ant_core_close_channel(channel);
  }
  
  return STATUS_OK;
}


int8_t ant_scan_on(uint8_t channel) {
  int8_t status;
  //search->tx_type=7;
  //search->device_type=1;
  //search->device_no=49;
  if(ant_core_channel_is_open(channel)) {
    return ERR_BUSY;
  }
  
  ant_callback(ANT_EVENT_SCAN_START, NULL, 0);
  
  status=ant_scan_start(channel, ANT_MASTER_CHANNEL_FREQ, ANT_MASTER_CHANNEL_PERIOD);
    
  if(status!=STATUS_OK) {
    ant_core_cleanup_channels();
  }
  return status;
}


int8_t ant_send_to_base(uint8_t *data, uint16_t len) {
  uint8_t dst;
  int8_t status;
  switch(ant_state.device_type) {
    case ANT_DEVICE_TYPE_SLAVE:
      status=ant_slave_send(ANT_SLAVE_CHANNEL_NO, data, len);
      break;
    default:
      dst=olrs_find_node_id(ANT_DEVICE_TYPE_BASE, 0);
      if(dst==0) status=ERR_IO_ERROR;
      //send packet to basestation node
      else status=ant_route_send(dst, data, len);
      break;
  }
  return status;
}

int8_t ant_send_to_device(uint32_t device_id, uint8_t *data, uint16_t len) {
  int8_t status;
  uint8_t dst;
  switch(ant_state.device_type) {
    case ANT_DEVICE_TYPE_SLAVE:
      status=ant_slave_send(ANT_SLAVE_CHANNEL_NO, data, len);
      break;
    default:
      if(ant_master_has_slave(device_id)) {
        status=ant_master_send(ANT_MASTER_CHANNEL_NO, device_id, data, len);
      }
      else {
        dst=olrs_find_node_id(0, device_id);
        if(dst==0) status=ERR_IO_ERROR;
        //send packet to node
        else status=ant_route_send(dst, data, len);
      }      
  }
  return status;
}


void ant_set_slave_control(uint16_t polling_interval, uint16_t timeout, uint8_t no_of_polls) {
  send_slave_control_msg(ANT_SLAVE_CHANNEL_NO, ant_slave.shared_address, polling_interval, timeout, no_of_polls);
}


static void scan_callback(uint8_t channel, struct ant_scan_event *evt) {
  uint8_t new_channel;
  
  switch(evt->type) {
    case ANT_SCAN_EVENT_NODE_FOUND:
      if(ant_state.device_type!=ANT_DEVICE_TYPE_SLAVE) {
        if(!ant_route_has_node(evt->node_id) || !ant_route_check(evt->node_id)) {
          ant_scan_connect(channel, ANT_SCAN_CONNECT_NODE);
        }
        break;
      }
      //no break;
    case ANT_SCAN_EVENT_NETWORK_FOUND:
      if(ant_state.network_id==ANT_NO_NETWORK || ant_state.network_id==evt->network_id) {
        ant_scan_connect(channel, ANT_SCAN_CONNECT_MASTER);
      }
      break;
    case ANT_SCAN_EVENT_NODE_CONNECTED:
      ant_state.network_id=evt->network_id;
      ant_callback(ANT_EVENT_CONNECTED, NULL, 0);
      
      ant_emit_on(ANT_MASTER_CHANNEL_NO);
      
      //find a free channel slot
      for(new_channel=ANT_SCAN_CHANNEL_LOW;new_channel<=ANT_SCAN_CHANNEL_HIGH;new_channel++) {
        if(ant_core_channels[new_channel].status==ANT_CHANNEL_STATUS_UNASSIGNED) {
          break;
        }
      }
      if(new_channel>ANT_SCAN_CHANNEL_HIGH) {
        ant_core_cleanup_channels();
        return;
      }
      ant_scan_on(new_channel);
      break;
    case ANT_SCAN_EVENT_MASTER_CONNECTED:
      ant_state.network_id=evt->network_id;
      ant_callback(ANT_EVENT_CONNECTED, NULL, 0);
      break;
    case ANT_SCAN_EVENT_ERROR:
      break;
    case ANT_SCAN_EVENT_CLOSED:
      ant_reset();
      break;
  }
}


static void master_callback(uint8_t channel, struct ant_master_event *evt) {
  uint8_t i, new_channel;
  
  switch(evt->type) {
    case ANT_MASTER_EVENT_NEW_SLAVE:
      break;
    case ANT_MASTER_EVENT_NEW_NODE:
      
      //find free channel slot
      for(i=0;i<ANT_NO_OF_CHANNELS;i++) {
        if(ant_core_channels[i].status==ANT_CHANNEL_STATUS_UNASSIGNED) {
          new_channel=i;
          break;
        }
      }
      //no free channel slot found
      if(i==ANT_NO_OF_CHANNELS) {
        ant_core_cleanup_channels();
        break;
      }
      //open routing channel
      ant_route_open_channel(new_channel, evt->node, true);
      break;
    case ANT_MASTER_EVENT_DATA:
      //receive data from slave
      ant_callback(ANT_EVENT_SLAVE_DATA, evt->msg->len<=5 ? evt->msg->data : evt->msg->burst, evt->msg->len);
      break;
    case ANT_MASTER_EVENT_CLOSED:
      break;
  }
}


static void slave_callback(uint8_t channel, struct ant_slave_event_t *evt) {
  struct ant_slave_msg *msg=evt->msg;
  switch(evt->type) {
    case ANT_SLAVE_EVENT_POLL_BEGIN:
      ant_callback(ANT_EVENT_POLL_BEGIN, msg->len<=5 ? msg->data : msg->burst, msg->len);
      break;
    case ANT_SLAVE_EVENT_POLL_END:
      ant_callback(ANT_EVENT_POLL_END, msg->len<=5 ? msg->data : msg->burst, msg->len);
      break;
    case ANT_SLAVE_EVENT_DATA:
      //receive data from master
      ant_callback(ANT_EVENT_MASTER_DATA, msg->len<=5 ? msg->data : msg->burst, msg->len);
      break;
    case ANT_SLAVE_EVENT_CLOSED:
      ant_scan_on(channel);
      break;
    default:
      ant_callback(ANT_EVENT_WAKEUP, NULL, 0);
  }
}


static void route_callback(uint8_t channel, struct ant_route_event *evt) {
  
  switch(evt->type) {
    case ANT_ROUTE_EVENT_DATA:
      //receive data from network
      if(evt->msg->len<=4) {
        ant_callback(ANT_EVENT_NETWORK_DATA, evt->msg->payload, evt->msg->len);
      }
      else {
        ant_callback(ANT_EVENT_NETWORK_DATA, evt->msg->burst, evt->msg->len);
      }
      break;
    case ANT_ROUTE_EVENT_NODE_ID_COLLISION:
      //change node id
      ant_state.node_id=ant_route_suggest_node_id();
      break;
    case ANT_ROUTE_EVENT_CLOSED:
      break;
  }
}



