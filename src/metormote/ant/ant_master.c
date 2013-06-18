#include  "ant_master.h"

struct ant_master_t ant_master;

static uint8_t alternation_index;
static uint16_t next_shared_address;

//forward declarations
static void event_callback(struct ant_event *evt);
static void master_data(struct ant_event *evt);
static struct ant_slave_t *find_slave(uint8_t channel, uint16_t shared_address);
static struct ant_slave_list_entry *find_slave_list_entry(uint8_t channel, uint16_t shared_address);
static struct ant_slave_list_entry *get_slave_list_entry(uint8_t channel, uint32_t device_id);
static void update_last_rx(uint8_t channel, uint16_t shared_address);
static int8_t add_slave(struct ant_slave_t *slave);
static void remove_slave(struct ant_slave_t *slave);
static void poll_slave(uint8_t channel);
static void master_control(uint8_t channel, uint16_t shared_address, uint8_t *data);
static void send_signaling(uint8_t channel);
static void master_connect(uint8_t channel, uint8_t *data);
static void inc_shared_address(uint8_t channel);

static void (*master_callback)(uint8_t channel, struct ant_master_event *evt);

void ant_master_init(void (*callback)(uint8_t channel, struct ant_master_event *evt)) {
  master_callback=callback;
  ant_master_reset();
}

void ant_master_reset() {
  struct ant_slave_list_entry *entry=ant_master.slave_list;
  while(entry) {
    mem_safe_free(entry);
    entry=entry->next;
  }
  memset(&ant_master, 0, sizeof(struct ant_master_t));
  next_shared_address=1;
}

uint8_t ant_master_has_slave(uint32_t device_id) {
  struct ant_slave_list_entry *entry=ant_master.slave_list;
  while(entry) {
    if(entry->slave.device_id==device_id) {
      return TRUE;
    }
    entry=entry->next;
  }
  return FALSE;
}



int8_t ant_master_open_channel(uint8_t channel, uint8_t frequency, uint16_t period) {
  struct ant_config config;
  
  memset(&config, 0, sizeof(struct ant_config));
  
  config.master=1;
  config.shared=1;
  config.bg_scan=0;
  config.freq_agility=0;
  config.tx_type=3;
  //config.tx_type=7;
  config.pairing=0;
  config.device_type=1;
  config.device_no=ant_state.node_id;
  //config.device_no=49; 
  config.frequency=frequency;
  config.period=period;
  config.tx_power=ANT_TX_POWER;
  config.timeout_hp=ANT_MASTER_CHANNEL_TIMEOUT_HIGH;
  config.timeout_lp=ANT_MASTER_CHANNEL_TIMEOUT_LOW;
  config.proximity=0;
  config.search_priority=32;
  
  if(ant_core_config(channel, &config)!=STATUS_OK) {
    return ERR_IO_ERROR;
  }
  
  return ant_core_open_channel(channel, &event_callback);
}



int8_t ant_master_send(uint8_t channel, uint32_t device_id, uint8_t *data, uint16_t len) {
  int8_t status;
  uint8_t *ptr;
  struct ant_slave_msg msg;
  struct ant_slave_list_entry *entry=get_slave_list_entry(channel, device_id);
  struct ant_slave_t *slave;
  
  if(!entry) {
    return ERR_IO_ERROR;
  }
  
  slave=&(entry->slave);
  if(!slave->shared_address){
    return ERR_BAD_ADDRESS;
  }
  
  msg.shared_address=slave->shared_address;
  msg.len=len;
  
  entry->next_poll=0;
  entry->poll_count=2;
  if(!ant_master.current) {
    ant_master.current=entry;
  }
  
  send_poll_msg(channel, slave->shared_address, true);
  
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


static void event_callback(struct ant_event *evt) {
  struct ant_master_event event;
  
  switch(evt->event_code) {
    case ANT_DATA_BROADCAST:
    case ANT_DATA_ACKNOWLEDGE:
    case ANT_DATA_BURST_TRANSFER:
      master_data(evt);
      break;
    case ANT_EVENT_TRANSFER_TX_COMPLETED:
    case ANT_EVENT_TRANSFER_TX_FAILED:
    case ANT_EVENT_TX:
      //if we have not sent any signaling data in this time slot we should 
      //poll the sensors and send address and channel available messages
      if(ant_master.tx_counter>0) {
        send_signaling(evt->channel);
        ant_master.tx_counter--;
        break;
      }
      //make sure we are in the right state
      ant_master.state=ANT_MASTER_STATE_AVAILABLE;
      poll_slave(evt->channel);
      break;
    case ANT_EVENT_CHANNEL_CLOSED:
      ant_core_unassign_channel(evt->channel);
      event.type=ANT_MASTER_EVENT_CLOSED;
      master_callback(evt->channel, &event);
      break;
  }
}


static void master_data(struct ant_event *evt) {
  uint8_t *ptr;
  struct ant_slave_msg msg;
  struct ant_master_event event;
  
  msg.shared_address=evt->shared_address;
  evt->stream->read(evt->stream, &msg.len, 1);
  evt->stream->read(evt->stream, msg.data, 5);
  evt->data_len-=6;
  
  //sensor signaling
  if(msg.shared_address==0xFFFF || msg.shared_address==0xFFFE) {
    if(evt->data_len>0) {
      ant_core_clear_rx_stream(evt->data_len);
      return;
    }
    ptr=(uint8_t *)&msg;
    master_connect(evt->channel, ptr+2);
    return;
  }    
  
  memset(&event, 0, sizeof(struct ant_master_event));
  
  //sensor data
  switch(evt->event_code) {
    case ANT_DATA_BROADCAST:
      ptr=(uint8_t *)&msg;
      master_control(evt->channel, msg.shared_address, ptr+2);
      break;
    case ANT_DATA_ACKNOWLEDGE:
      event.type=ANT_MASTER_EVENT_DATA;
      event.slave=find_slave(evt->channel, evt->shared_address);
      event.msg=&msg;
      master_callback(evt->channel, &event);
      break;
    case ANT_DATA_BURST_TRANSFER:
      msg.burst=mem_safe_malloc(evt->data_len);
      if(msg.burst!=NULL) {
        evt->stream->read(evt->stream, msg.burst, evt->data_len);
        event.type=ANT_MASTER_EVENT_DATA;
        event.slave=find_slave(evt->channel, evt->shared_address);
        event.msg=&msg;
        master_callback(evt->channel, &event);
      }
      else {
        ant_core_clear_rx_stream(evt->data_len);
      }
      mem_safe_free(msg.burst);
      
      update_last_rx(evt->channel, evt->shared_address);
      break;
  }
}

static struct ant_slave_t *find_slave(uint8_t channel, uint16_t shared_address) {
  struct ant_slave_list_entry *entry=ant_master.slave_list;
  while(entry) {
    if(entry->slave.shared_address==shared_address) {
      return &entry->slave;
    }
    entry=entry->next;
  }
  return NULL;
}

static struct ant_slave_list_entry *find_slave_list_entry(uint8_t channel, uint16_t shared_address) {
  struct ant_slave_list_entry *entry=ant_master.slave_list;
  while(entry) {
    if(entry->slave.shared_address==shared_address) {
      return entry;
    }
    entry=entry->next;
  }
  return NULL;
}

static struct ant_slave_list_entry *get_slave_list_entry(uint8_t channel, uint32_t device_id) {
  struct ant_slave_list_entry *entry=ant_master.slave_list;
  while(entry) {
    if(entry->slave.device_id==device_id) {
      return entry;
    }
    entry=entry->next;
  }
  return NULL;
}

static void update_last_rx(uint8_t channel, uint16_t shared_address) {
  struct ant_slave_t *slave=find_slave(channel, shared_address);
  if(slave) {
    slave->last_rx=rtc_get_time();
  }
}

static int8_t add_slave(struct ant_slave_t *slave) {
  struct ant_slave_list_entry **e=&ant_master.slave_list;
  struct ant_slave_list_entry *entry=mem_safe_calloc(1, sizeof(struct ant_slave_list_entry));
  if(entry==NULL) {
    return ERR_NO_MEMORY;
  }
  memcpy(&entry->slave, slave, sizeof(struct ant_slave_t));
  entry->poll_count=0;
  entry->next_poll=0;
  for(;;) {
    if(!*e) {
      *e=entry;
      break;
    }
    e=&((*e)->next);
  }
  return STATUS_OK;
}

static void remove_slave(struct ant_slave_t *slave) {
  struct ant_slave_list_entry **entry=&ant_master.slave_list;
  void *ptr;
  
  while(*entry) {
    if(&((*entry)->slave)==slave) {
      ptr=*entry;
      *entry=(*entry)->next;
      mem_safe_free(ptr);
      break;
    }
    entry=&((*entry)->next);
  }
}


static void poll_slave(uint8_t channel) {
  uint32_t now=rtc_get_time();
  struct ant_slave_list_entry *entry;
  struct ant_slave_t *slave;

  while(ant_master.current) {
    entry=ant_master.current;

    if(now > (entry->slave.last_rx+entry->slave.timeout)) {
      ant_master.current=entry->next;
      remove_slave(&entry->slave);
      continue;            
    }                  
    else if(now < entry->next_poll) {
      ant_master.current=ant_master.current->next;
      continue;
    }
    
    slave=&entry->slave;
    if(entry->poll_count==1) {
      send_poll_begin_msg(channel, slave->shared_address);
    }
    else if(entry->poll_count<(entry->slave.no_of_polls-1)) {
      send_poll_msg(channel, slave->shared_address, false);
    }
    else {
      send_poll_end_msg(channel, slave->shared_address);
    }
    
    if(++entry->poll_count>=entry->slave.no_of_polls) {
      entry->poll_count=0;
      entry->next_poll=now+entry->slave.poll_interval;
      ant_master.current=ant_master.current->next;
    }
    return;
  }

  switch(++alternation_index % 2) {
    case 0:
      send_address_available_msg(channel, next_shared_address);
      break;
    case 1:
      send_channel_available_msg(channel, ant_state.network_id);
      ant_master.current=ant_master.slave_list;
      break;
  }
}

static void master_control(uint8_t channel, uint16_t shared_address, uint8_t *data) {
  struct ant_slave_list_entry *entry=find_slave_list_entry(channel, shared_address);
  struct ant_slave_t *slave;
  uint16_t poll_interval;
  
  if(entry) {
    slave=&entry->slave;
    switch(data[0]) {
      case ANT_SLAVE_CONTROL_MESSAGE:
        poll_interval=data[1];
        poll_interval |= ((0xFFFF & data[2]) << 8);
        slave->timeout=data[3];
        slave->timeout |= ((0xFFFF & data[4]) << 8);
        slave->no_of_polls=data[5];
        
        if(slave->poll_interval!=poll_interval) {
          slave->poll_interval=poll_interval;
          entry->next_poll=rtc_get_time()+poll_interval;
        }
        break;
    }
  }
}

static void send_signaling(uint8_t channel) {
  switch(ant_master.state) {
    case ANT_MASTER_STATE_ASSIGNING_ADDRESS:
      send_busy_acquiring_msg(channel, ant_master.connecting_slave.device_id);
      break;
    case ANT_MASTER_STATE_ASSIGNING_CHANNEL:
      send_busy_acquiring_msg(channel, ant_master.connecting_node.device_id);
      break;
    case ANT_MASTER_STATE_CHANNEL_ASSIGNED:
    case ANT_MASTER_STATE_CHANNEL_CONFIRMED:
      send_channel_info_msg(channel, ant_master.connecting_node.freq, ant_master.connecting_node.period);
      break;
    default:
      break;
  }
}

static void master_connect(uint8_t channel, uint8_t *data) {
  uint32_t f;
  struct ant_slave_t *slave=&ant_master.connecting_slave;
  struct ant_node *node=&ant_master.connecting_node;
  struct ant_master_event event;
  
  switch(ant_master.state) {
    case ANT_MASTER_STATE_AVAILABLE:
      switch(data[0]) {
        case ANT_REQUEST_ADDRESS_MESSAGE:
          ant_master.state=ANT_MASTER_STATE_ASSIGNING_ADDRESS;
          slave->device_id = data[2];
          slave->device_id |= ((0xFFFFFFFF & data[3]) << 8);
          slave->device_id |= ((0xFFFFFFFF & data[4]) << 16);
          slave->device_id |= ((0xFFFFFFFF & data[5]) << 24);
          ant_master.tx_counter=ANT_MASTER_SEARCH_TIMEOUT;
          break;
        case ANT_REQUEST_CHANNEL_MESSAGE:
          ant_master.state=ANT_MASTER_STATE_ASSIGNING_CHANNEL;
          node->node_id = data[1];
          node->device_id = data[2];
          node->device_id |= ((0xFFFFFFFF & data[3]) << 8);
          node->device_id |= ((0xFFFFFFFF & data[4]) << 16);
          node->device_id |= ((0xFFFFFFFF & data[5]) << 24);
          ant_master.tx_counter=ANT_MASTER_SEARCH_TIMEOUT;
          break;
      }    
      break;
    case ANT_MASTER_STATE_ASSIGNING_ADDRESS:
      if(data[0]==ANT_CONFIRM_AQUIRE_MESSAGE) {
        if(ant_master_has_slave(slave->device_id)) {
          slave=&(get_slave_list_entry(channel, slave->device_id)->slave);
          slave->shared_address=next_shared_address;
          slave->last_rx=rtc_get_time();
        }
        else {
          slave->shared_address=next_shared_address;
          slave->last_rx=rtc_get_time();
          slave->poll_interval=ANT_MASTER_SENSOR_POLL_INTERVAL;
          slave->timeout=ANT_MASTER_SENSOR_TIMEOUT;
          slave->no_of_polls=ANT_MASTER_SENSOR_NO_OF_POLLS;
          if(add_slave(slave)==STATUS_OK) {
            event.type=ANT_MASTER_EVENT_NEW_SLAVE;
            event.slave=slave;
            master_callback(channel, &event);
          }
        }
        ant_master.tx_counter=0;
        memset(&ant_master.connecting_slave, 0, sizeof(struct ant_slave_t));
        inc_shared_address(channel);
      }
      break;
    case ANT_MASTER_STATE_ASSIGNING_CHANNEL:
      if(data[0]==ANT_CONFIRM_AQUIRE_MESSAGE) {
        ant_master.state=ANT_MASTER_STATE_CHANNEL_ASSIGNED;
        f = random();
        node->freq[0] = 61+(f & 0x3F);
        node->freq[1] = ((f >> 8) & 0x3F);
        node->freq[2] = 30+((f >> 16) & 0x3F);
        node->period = ANT_ROUTE_CHANNEL_PERIOD;
        ant_master.tx_counter=ANT_MASTER_SEARCH_TIMEOUT;
      }    
      break;
    case ANT_MASTER_STATE_CHANNEL_ASSIGNED:
      if(data[0]==ANT_CONFIRM_AQUIRE_MESSAGE) {
        ant_master.state=ANT_MASTER_STATE_CHANNEL_CONFIRMED;
        ant_master.tx_counter=4;
        event.type=ANT_MASTER_EVENT_NEW_NODE;
        event.node=node;
        master_callback(channel, &event);
      }
      break;
    default:
      break;
  }
} 

static void inc_shared_address(uint8_t channel) {
  uint16_t a=next_shared_address;
  for(;;) {
    while(find_slave(channel, ++a));
    
    if(a>0 && a<0xFFFE) {
      break;
    }
  }
  next_shared_address=a;
}