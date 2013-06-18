#include  "ant_route.h"

static struct ant_route_t route_state;

static int8_t add_node(uint8_t channel, struct ant_node *node);
static void remove_node(uint8_t channel);
static struct ant_node_list_entry *get_node_entry(uint8_t channel);
static struct ant_node_list_entry *find_node_entry(uint8_t node_id);
static bool has_node(uint8_t node_id);
static uint8_t inc_tx_counter(uint8_t channel);
static void reset_tx_counter(uint8_t channel);
static void update_last_hello(uint8_t channel);

static void route_event(struct ant_event *evt);
static void sensor_data(struct ant_event *evt);
static void routing_data(struct ant_event *evt);

static int8_t send_msg(struct ant_data_msg *msg);
static int8_t send_tc(void);
static int8_t send_mid(void);
static int8_t send_hna(void);
static int8_t send_hello(void);
static int8_t broadcast_all(uint8_t *msg, uint8_t exclude_channel);

static void emit(void);

//callback for route events
static void (*route_callback)(uint8_t channel, struct ant_route_event *);

void ant_route_init(void (*callback)(uint8_t, struct ant_route_event *)) {
  route_callback=callback;
  ant_route_reset();
  olsr_init(&(ant_state.node_id));
}

void ant_route_reset() {
  struct ant_node_list_entry *entry=route_state.node_list;
  while(entry) {
    mem_safe_free(entry);
    entry=entry->next;
  }
  memset(&route_state, 0, sizeof(struct ant_route_t));
}

bool ant_route_has_node(uint8_t node_id) {
  struct ant_node_list_entry *entry=route_state.node_list;
  while(entry) {
    if(entry->node.node_id==node_id) {
      return true;
    }
    entry=entry->next;
  }
  return false;
}

struct ant_node *ant_route_get_node(uint32_t device_id) {
  struct ant_node_list_entry *entry=route_state.node_list;
  while(entry) {
    if(entry->node.device_id==device_id) {
      return &entry->node;
    }
    entry=entry->next;
  }
  return NULL;
}

int8_t ant_route_open_channel(uint8_t channel, struct ant_node *node, bool master) {
  int8_t status;
  struct ant_config config;
  
  if(has_node(node->node_id)) {
    if(ant_core_close_channel(find_node_entry(node->node_id)->channel)!=STATUS_OK) {
      remove_node(find_node_entry(node->node_id)->channel);
    }
  }
    
  memset(&config, 0, sizeof(struct ant_config));
  
  config.master=master;
  config.shared=0;
  config.bg_scan=0;
  config.freq_agility=0;
  config.tx_type=1;
  config.pairing=0;
  config.device_type=1;
  if(master) {
    config.device_no=ant_state.node_id;
  }
  else {
    config.device_no=node->node_id;
  }
  config.frequency=node->freq[0];
  config.tx_power=ANT_TX_POWER;
  config.period=node->period;
  config.timeout_hp=ANT_ROUTE_CHANNEL_TIMEOUT_HIGH;
  config.timeout_lp=ANT_ROUTE_CHANNEL_TIMEOUT_LOW;
  config.freq[0]=node->freq[0];
  config.freq[1]=node->freq[1];
  config.freq[2]=node->freq[2];
  config.proximity=0;
  config.search_priority=64;
  
  status=ant_core_config(channel, &config);
  if(status!=STATUS_OK) {
    return ERR_IO_ERROR;
  }
  
  status=ant_core_open_channel(channel, &route_event);
  
  if(status==STATUS_OK) {
    add_node(channel, node);
  }
  
  return status;
}


int8_t ant_route_send(uint8_t destination, uint8_t *data, uint16_t len) {
  uint8_t status;
  struct ant_data_msg msg;
  
  msg.type=ANT_MSG_TYPE_DATA;
  msg.ttl=(OLSR_MAX_HOPS-1);
  msg.src=ant_state.node_id;
  msg.dst=destination;
  msg.len=len;
  if(len<=4) {
    memcpy(msg.payload, data, len);
  }
  else {
    msg.burst=data;
  }
  status=send_msg(&msg);
  
  return status;
}


static struct ant_node_list_entry *get_node_entry(uint8_t channel) {
  struct ant_node_list_entry *entry=route_state.node_list;
  while(entry) {
    if(entry->channel==channel) {
      return entry;
    }
    entry=entry->next;
  }
  return NULL;
}

static bool has_node(uint8_t node_id) {
  struct ant_node_list_entry *entry=route_state.node_list;
  while(entry) {
    if(entry->node.node_id==node_id) {
      return true;
    }
    entry=entry->next;
  }
  return false;
}


static struct ant_node_list_entry *find_node_entry(uint8_t node_id) {
  struct ant_node_list_entry *entry=route_state.node_list;
  while(entry) {
    if(entry->node.node_id==node_id) {
      return entry;
    }
    entry=entry->next;
  }
  return NULL;
}

static int8_t add_node(uint8_t channel, struct ant_node *node) {
  struct ant_node_list_entry **e=&route_state.node_list;
  struct ant_node_list_entry *entry=mem_safe_calloc(1, sizeof(struct ant_node_list_entry));
  if(entry==NULL) {
    return ERR_NO_MEMORY;
  }
  memcpy(&entry->node, node, sizeof(struct ant_node));
  entry->channel=channel;
  entry->tx_counter=0;
  entry->last_hello=rtc_get_time();
  for(;;) {
    if(!*e) {
      *e=entry;
      break;
    }
    e=&((*e)->next);
  }
  return STATUS_OK;
}

static void remove_node(uint8_t channel) {
  struct ant_node_list_entry **e=&route_state.node_list;
  struct ant_node_list_entry *entry=get_node_entry(channel);
  void *ptr;
  
  olsr_remove_link(channel);
      
  if(entry==NULL) {
    return;
  }
  
  while(*e) {
    if((*e)->channel==entry->channel) {
      ptr=*e;
      *e=(*e)->next;
      mem_safe_free(ptr);
      break;
    }
    e=&((*e)->next);
  }
}


static void reset_tx_counter(uint8_t channel) {
  struct ant_node_list_entry *entry=get_node_entry(channel);
  if(entry) {
    entry->tx_counter=0;
  }
}

static uint8_t inc_tx_counter(uint8_t channel) {
  struct ant_node_list_entry *entry=get_node_entry(channel);
  if(entry) {
    return ++entry->tx_counter;
  }
  return 0xFF;
}

static void update_last_hello(uint8_t channel) {
  struct ant_node_list_entry *entry=get_node_entry(channel);
  if(entry) {
    entry->last_hello=rtc_get_time();
  }
}

static void route_event(struct ant_event *evt) {
  struct ant_route_event route_event;
  
  switch(evt->event_code) {
    case ANT_DATA_BROADCAST:
      routing_data(evt);
      //reset the timeout counter when we get data from remote end
      reset_tx_counter(evt->channel);
      break;
    case ANT_DATA_ACKNOWLEDGE:
    case ANT_DATA_BURST_TRANSFER:
      sensor_data(evt);
      break;
    case ANT_EVENT_TX:
      if(ant_core_channels[evt->channel].master) {
        //if we have not heard from remote end then close the channel
        if(inc_tx_counter(evt->channel)>ANT_ROUTE_MAX_TX_COUNT) {
          ant_core_close_channel(evt->channel);
        }               
      }
      break;
    case ANT_EVENT_CHANNEL_CLOSED:
      ant_core_unassign_channel(evt->channel);
      remove_node(evt->channel);
      route_event.type=ANT_ROUTE_EVENT_CLOSED;
      route_callback(evt->channel, &route_event);
      break;
    case ANT_EVENT_RX_FAIL_GO_TO_SEARCH:
      ant_core_close_channel(evt->channel);
      break;
  }
  
  if(rtc_get_time()>route_state.last_emit) {
    emit();
    route_state.last_emit=rtc_get_time();
  }
  return;
}


static void sensor_data(struct ant_event *evt) {
  struct ant_data_msg msg;
  struct ant_route_event event;
  
  switch(evt->event_code) {
    case ANT_DATA_ACKNOWLEDGE:
    case ANT_DATA_BURST_TRANSFER:
      evt->stream->read(evt->stream, (uint8_t *)&msg, 8);
      evt->data_len-=8;
      msg.burst=(uint8_t *)mem_safe_malloc(evt->data_len);
      if(msg.burst!=NULL) {
        evt->stream->read(evt->stream, msg.burst, evt->data_len);
        //check if this node is the destination of the message
        if(msg.dst==ant_state.node_id) {
          event.type=ANT_ROUTE_EVENT_DATA;
          event.msg=&msg;
          route_callback(evt->channel, &event);
        }
        else if(msg.ttl!=0) {
          //decrease the ttl
          msg.ttl--;
          //forward message
          send_msg(&msg);
        }
      }
      else {
        ant_core_clear_rx_stream(evt->data_len);
      }    
      mem_safe_free(msg.burst);
      break;
    default:
      break;
  }
  return;
}


static int8_t send_msg(struct ant_data_msg *msg) {
  uint8_t i, channel, next;
  int8_t status;
  
  if(msg->dst==0) {
    return ERR_BAD_ADDRESS;
  }  
  else if(msg->dst==ant_state.node_id) {
    return ERR_BAD_ADDRESS;
  }
  
  //find the next node on the path
  next=olsr_route_next_node(msg->dst);
  //no route found
  if(next==0) return ERR_BAD_ADDRESS;
  
  //find the channel
  channel=-1;
  for(i=0;i<OLSR_MAX_LINK;i++) {
    if(olsr_link_set[i].neighbour==next) {
      channel=olsr_link_set[i].channel;
    }
  }
  
  //no channel found
  if(channel==-1) return ERR_BAD_ADDRESS;
  
  //check if channel is open
  if(ant_core_channels[channel].status!=ANT_CHANNEL_STATUS_TRACKING) {
    olsr_update();
    return ERR_IO_ERROR;
  }
  
  //add the message to the channels transmit queue
  if(msg->len<=4) {
    status=ant_core_send_acknowledged_data(channel, (uint8_t *)msg);      
  }
  else {
    status=ant_core_send_burst_transfer(channel, NULL, (uint8_t *)msg, msg->burst, msg->len);
  }
  return status;
}



static void routing_data(struct ant_event *evt) {
  struct ant_route_event event;
  uint8_t msg_type, channel;
  uint8_t data[8];
  
  if(evt->data_len>8) {
    ant_core_clear_rx_stream(evt->data_len);
    return;
  }
  
  channel=evt->channel;
  evt->stream->read(evt->stream, data, evt->data_len);
  msg_type=EXTRACT_BITS(data[0], 4, 4);
  
  switch(msg_type) {
    case OLSR_HELLO_MESSAGE:
    {
      update_last_hello(evt->channel);
      olsr_process_hello_msg((struct olsr_hello_msg *)data, channel);
      break;
    }
    case OLSR_TC_MESSAGE:
    {
      struct olsr_tc_msg *tc_msg=(struct olsr_tc_msg *)data;
      if(olsr_process_tc_msg(tc_msg, channel)) {
        tc_msg->ttl--;
        broadcast_all((uint8_t *)tc_msg, channel);
      }
      break;
    }
    case OLSR_MID_MESSAGE:
    {
      struct olsr_mid_msg *mid_msg=(struct olsr_mid_msg *)data;
      //check if we have a node id collision
      if(mid_msg->origin==ant_state.node_id && mid_msg->device_id!=ant_state.device_id) {
        //there exist another node with the same node_id
          event.type=ANT_ROUTE_EVENT_NODE_ID_COLLISION;
          route_callback(channel, &event);
          break;
      }
      
      if(olsr_process_mid_msg(mid_msg, channel)) {
        mid_msg->ttl--;
        broadcast_all((uint8_t *)mid_msg, channel);
      }
      break;
    }
    case OLSR_HNA_MESSAGE:
    {
      struct olsr_hna_msg *hna_msg=(struct olsr_hna_msg *)data;
      if(olsr_process_hna_msg(hna_msg, channel)) {
        hna_msg->ttl--;
        broadcast_all((uint8_t *)hna_msg, channel);
      }
      break;
    }
    default:
      break;
  }
}


static void emit() {
  olsr_update();
  
  switch(route_state.emit_state) {
    case OLSR_HELLO_MESSAGE:
      send_hello();
      route_state.emit_state=OLSR_TC_MESSAGE;
      break;
    case OLSR_TC_MESSAGE:
      //we should only emit topology control messages 
      //if we are selected mpr node by some other node
      if(olsr_is_selected_mpr()) {
        send_tc();
      }
      route_state.emit_state=OLSR_MID_MESSAGE;
      break;
    case OLSR_MID_MESSAGE:
      //if(ant_state.device_type==ANT_DEVICE_TYPE_BASESTATION) {
        //send mid messages for all connected 
        //sensors including itself
        while(send_mid()!=0);
      //}
      route_state.emit_state=OLSR_HNA_MESSAGE;      
      break;
    case OLSR_HNA_MESSAGE:
      send_hna();
    default:
      route_state.emit_state=OLSR_HELLO_MESSAGE;
      break;
  }
}



static int8_t send_tc() {
  int i, c;
  struct olsr_tc_msg msg;
  msg.type=OLSR_TC_MESSAGE;
  msg.ttl=(OLSR_MAX_HOPS-1);
  msg.origin=ant_state.node_id;
  msg.seq_no=route_state.tc_seq_no++;
  msg.ansn=route_state.ansn++;
  for(i=0;i<4;i++) msg.neighbour[i]=0;
  
  c=0;
  for(i=0;i<OLSR_MAX_MPR;i++) {
    if(olsr_mpr_select_set[i].neighbour) {
      msg.neighbour[c]=olsr_mpr_select_set[i].neighbour;
      c++;
      /*
      if(c == 4) {
        c=0;
        
        broadcast_all((uint8_t *)msg);
        
        msg->seq_no=sequence_no++;
        for(j=0;j<4;j++) msg->neighbour[j]=0;
      }
      */
    }
  }
  if(c) {
    broadcast_all((uint8_t *)&msg, -1);
  }
  
  return STATUS_OK;
}


static int8_t send_mid() {
  uint8_t i;
  struct olsr_mid_msg msg;
  msg.type=OLSR_MID_MESSAGE;
  msg.ttl=(OLSR_MAX_HOPS-1);
  msg.origin=ant_state.node_id;
  struct ant_slave_list_entry *entry=ant_master.slave_list;
  
  for(i=0;i<route_state.mid_index;i++) {
    if(!entry) break;
    entry=entry->next;
  }
  
  if(entry) {
    msg.device_type=entry->slave.device_type;
    msg.device_id=entry->slave.device_id;
    msg.seq_no=route_state.mid_seq_no++;
    broadcast_all((uint8_t *)&msg, -1);
    route_state.mid_index++;
    return STATUS_OK;
  }
  
  //send own device id
  msg.device_type=ant_state.device_type;
  msg.device_id=ant_state.device_id;
  msg.seq_no=route_state.mid_seq_no++;
  broadcast_all((uint8_t *)&msg, -1);
  route_state.mid_index=0;
  return STATUS_OK;    
}


static int8_t send_hna() {
  struct olsr_hna_msg msg;
   
  msg.type=OLSR_HNA_MESSAGE;
  msg.ttl=(OLSR_MAX_HOPS-1);
  msg.origin=ant_state.node_id;
  msg.seq_no=route_state.hna_seq_no++;
  msg.network_type=0;
  msg.network_id=0;
  return broadcast_all((uint8_t *)&msg, -1);
}


static int8_t send_hello() {
  int i, j, c;
  struct olsr_hello_msg msg;
  uint8_t link_type, neighbour_type;
  uint32_t time;
  
  //type
  msg.type=OLSR_HELLO_MESSAGE;
  msg.willingness=OLSR_WILL_DEFAULT;
  msg.origin=ant_state.node_id;
  msg.link_type=0;
  msg.neighbour_type=0;
  for(i=0;i<4;i++) msg.neighbour[i]=0;
  
  c=0;
  for(i=0;i<OLSR_MAX_LINK;i++) {
    
    //if not empty slot
    if(olsr_link_set[i].neighbour) {
      
      //link_type
      time=rtc_get_time();
      if(olsr_link_set[i].sym_time>=time) {
        link_type=OLSR_SYM_LINK;
      }
      else if(olsr_link_set[i].asym_time>=time) {
        link_type=OLSR_ASYM_LINK;
      }
      else {
        link_type=OLSR_LOST_LINK;
      }
      msg.link_type|=(link_type << (6-2*c));
      
      //neighbour_type
      neighbour_type=OLSR_NOT_NEIGH;
      for(j=0;j<OLSR_MAX_NEIGHBOUR;j++) {
        if(olsr_neighbour_set[j].neighbour==olsr_link_set[i].neighbour) {
          //we have selected this neighbour as mpr
          if(olsr_neighbour_set[j].mpr) {
            neighbour_type=OLSR_MPR_NEIGH;
          }
          else if(olsr_neighbour_set[j].status==OLSR_SYM) {
            neighbour_type=OLSR_SYM_NEIGH;
          }
          break;
        }
      }
      msg.neighbour_type|=(neighbour_type << (6-2*c));
      msg.neighbour[c]=olsr_link_set[i].neighbour;
      c++;
      /*
      if(c == 4) {
        ant_core_send_broadcast_data(channel, (uint8_t *)&msg);
        c=0;
        msg.link_type=0;
        msg.neighbour_type=0;
        for(j=0;j<4;j++) msg.neighbour[j]=0;
      }
      */
    }
  }
  return broadcast_all((uint8_t *)&msg, -1);
}


static int8_t broadcast_all(uint8_t *msg, uint8_t exclude_channel) {
  struct ant_node_list_entry *entry=route_state.node_list;
  int8_t status=STATUS_OK;
  
  while(entry) {
    if(ant_core_channels[entry->channel].status==ANT_CHANNEL_STATUS_TRACKING && entry->channel!=exclude_channel) {
      status=ant_core_send_broadcast_data(entry->channel, msg, false);
    }
    entry=entry->next;
  }
  return status;
}

uint8_t ant_route_suggest_node_id() {
  int i;
  uint8_t n;
  
  for(;;) {
    n=(uint8_t)random();
    if(n==0 || n==ant_state.node_id) continue;
    
    for(i=0;i<OLSR_MAX_TOPOLOGY;i++) {
      if(olsr_topology_set[i].dest==n) break; 
    }
    if(i==OLSR_MAX_TOPOLOGY) break;
  }
  
  return n;
}

bool ant_route_check(uint8_t node_id) {
  uint8_t channel;
  struct ant_node_list_entry *entry=find_node_entry(node_id);
  if(entry!=NULL && rtc_get_time()>entry->last_hello+ANT_ROUTE_TIMEOUT) {
    channel=entry->channel;
    remove_node(channel);
    ant_core_close_channel(channel);
    return false;
  }
  return true;
}