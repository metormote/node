/* A adaption of the Optimized Link State Routing Protocol (OLSR) as
   specified in http://tools.ietf.org/html/rfc3626.
*/


#include "olsr.h"

//forward declarations
static uint8_t get_remote_node_id(uint8_t channel);
static struct olsr_neighbour *checked_add_neighbour(uint8_t neighbour);
static void checked_delete_neighbour(struct olsr_link *link);
static struct olsr_duplicate *check_duplicate(uint8_t origin,  uint8_t seq_no, uint8_t source, uint8_t *forward);
static void update_duplicate(struct olsr_duplicate *duplicate, uint8_t origin,  uint8_t seq_no, uint8_t source, uint8_t forward, uint32_t time);
static void calc_route_table(void);
static void calc_mpr_set(void);

static uint8_t *node_id;

//link set array
struct olsr_link olsr_link_set[OLSR_MAX_LINK];
static uint8_t olsr_link_set_size = 0;

//neighbour set array
struct olsr_neighbour olsr_neighbour_set[OLSR_MAX_NEIGHBOUR];
static uint8_t olsr_neighbour_set_size = 0;

//2-hop neighbour set array
struct olsr_2hop olsr_2hop_set[OLSR_MAX_2HOP];
static uint8_t olsr_2hop_set_size = 0;

//mpr selector set array
struct olsr_mpr_select olsr_mpr_select_set[OLSR_MAX_NEIGHBOUR];
static uint8_t olsr_mpr_select_set_size = 0;

//topology set array
struct olsr_topology olsr_topology_set[OLSR_MAX_TOPOLOGY];
static uint8_t olsr_topology_set_size = 0;

//hna set array
struct olsr_hna olsr_hna_set[OLSR_MAX_HNA];
static uint8_t olsr_hna_set_size = 0;

//mid set array
struct olsr_mid olsr_mid_set[OLSR_MAX_MID];
static uint8_t olsr_mid_set_size = 0;

//routing table array
struct olsr_route olsr_route_table[OLSR_MAX_ROUTE_TABLE];

//duplicate set
static struct olsr_duplicate olsr_duplicate_set[OLSR_MAX_DUPLICATES];
static uint16_t olsr_duplicate_set_size = 0;

//mutex used to protect from simultaneous access to the routing structures
static ATOM_MUTEX olsr_mutex;


void olsr_init(uint8_t *n) {
  int i;
  
  node_id=n;
  
  /* init our link set */
  for (i = 0;i < OLSR_MAX_LINK;i++)
    olsr_link_set[i].neighbour = 0;

  /* init our neighbour table */
  for (i = 0;i < OLSR_MAX_NEIGHBOUR;i++)
    olsr_neighbour_set[i].neighbour = 0;
    
  /* init our 2hop table */
  for (i = 0;i < OLSR_MAX_2HOP;i++)
    olsr_2hop_set[i].neighbour = 0;
    
  /* init our mpr select set */
  for (i = 0;i < OLSR_MAX_MPR;i++)
    olsr_mpr_select_set[i].neighbour = 0;
    
  /* init our topology table */
  for (i = 0;i < OLSR_MAX_TOPOLOGY;i++)
    olsr_topology_set[i].dest = 0;
  
  /* init our mid table */
  for (i = 0;i < OLSR_MAX_MID;i++)
    olsr_mid_set[i].origin = 0;
    
  /* init our hna table */
  for (i = 0;i < OLSR_MAX_HNA;i++)
    olsr_hna_set[i].gateway = 0;
  
  /* init our neighbour table */
  for (i = 0;i < OLSR_MAX_ROUTE_TABLE;i++)
    olsr_route_table[i].dest = 0;
    
  /* init our duplicate cache */
  for (i = 0;i < OLSR_MAX_DUPLICATES;i++)
    olsr_duplicate_set[i].origin = 0;
  
  atomMutexCreate(&olsr_mutex);
}



static uint8_t get_remote_node_id(uint8_t channel) {
  uint16_t i;
  
  for(i=0;i<OLSR_MAX_LINK;i++) {
    if(olsr_link_set[i].neighbour==0) continue;
    
    if(olsr_link_set[i].channel==channel) {
      return olsr_link_set[i].neighbour;
    }
  }
  return 0;
}


void olsr_remove_link(uint8_t channel) {
  uint16_t i;
  
  atomMutexGet(&olsr_mutex, 0);
  
  for(i=0;i<OLSR_MAX_LINK;i++) {
    if(olsr_link_set[i].neighbour==0) continue;
    
    if(olsr_link_set[i].channel==channel) {
      olsr_link_set[i].time=0;
      olsr_update();
      break;
    }
  }
  atomMutexPut(&olsr_mutex);
}

void olsr_update() {
  uint16_t i, j;
  uint32_t time;
  
  time=rtc_get_time();
  
  //protect access to routing structures
  atomMutexGet(&olsr_mutex, 0);
  
  j=0;
  for(i=0; i<OLSR_MAX_LINK; i++) {
    if(olsr_link_set[i].neighbour==0) continue;
      
    if(olsr_link_set[i].time<time) {
      //check if we should delete the neighbour entry
      checked_delete_neighbour(&(olsr_link_set[i]));
    }
    else if(olsr_link_set[i].sym_time<time) {
      for(j=0; j<OLSR_MAX_NEIGHBOUR; j++) {
        if(olsr_neighbour_set[j].neighbour==0) continue;
        
        if(olsr_link_set[i].neighbour==olsr_neighbour_set[j].neighbour) {
          olsr_neighbour_set[j].status=OLSR_NOT_SYM;
        }
      }
    }
  }
  
  for(i=0; i<OLSR_MAX_2HOP; i++) {
    if(olsr_2hop_set[i].neighbour==0) continue;
            
    if(olsr_2hop_set[i].time<time) {
      olsr_2hop_set[i].neighbour=0;
      olsr_2hop_set_size--;
    }
  }
  
  for(i=0; i<OLSR_MAX_MPR; i++) {
    if(olsr_mpr_select_set[i].neighbour==0) continue;
    
    if(olsr_mpr_select_set[i].time<time) {
      olsr_mpr_select_set[i].neighbour=0;
      olsr_mpr_select_set_size--;
    }
  }
  
  for(i=0; i<OLSR_MAX_TOPOLOGY; i++) {
    if(olsr_topology_set[i].dest==0) continue;
    
    if(olsr_topology_set[i].time<time) {
      olsr_topology_set[i].dest=0;
      olsr_topology_set_size--;
    }
  }
  
  for(i=0; i<OLSR_MAX_DUPLICATES; i++) {
    if(olsr_duplicate_set[i].origin==0) continue;
    
    if(olsr_duplicate_set[i].time<time) {
      olsr_duplicate_set[i].origin=0;
      olsr_duplicate_set_size--;
    }
  }
  
  calc_mpr_set();
  
  atomMutexPut(&olsr_mutex);
}



/*process incoming hello message*/
void olsr_process_hello_msg(struct olsr_hello_msg *msg, uint8_t channel) {
  uint16_t i, j, k;
  struct olsr_link *link;
  struct olsr_neighbour *neigh;
  struct olsr_2hop *twohop;
  struct olsr_mpr_select *mpr;
  uint8_t link_type, neighbour_type;
  uint32_t time;
  
  time=rtc_get_time();
  
  atomMutexGet(&olsr_mutex, 0);
  
  link=NULL;
  for(i=0; i<OLSR_MAX_LINK; i++) {
    if(olsr_link_set[i].neighbour==0) continue;
      
    //check if we have a link with this channel in the link set
    if(olsr_link_set[i].channel==channel) {
      link=&(olsr_link_set[i]);
        
      //if its not the same node as before
      if(olsr_link_set[i].neighbour!=msg->origin) {
        checked_delete_neighbour(&(olsr_link_set[i]));
        link=NULL;
      }
      break;
    }
  }
  
  //if no existing link found then create a new link entry
  if(link==NULL) {
      if (olsr_link_set_size < OLSR_MAX_LINK) {
        //create new entry
        for (i = 0; i < OLSR_MAX_LINK; i++) {
          if (olsr_link_set[i].neighbour == 0) {
            link=&(olsr_link_set[i]);
            olsr_link_set_size++;
            break;
          }
        }
      } 
      else {
        link=&(olsr_link_set[0]);
        //replace oldest entry
        for (i = 1;i < OLSR_MAX_LINK;i++) {
          if (olsr_link_set[i].time < link->time) {
            link=&(olsr_link_set[i]);
          }
        }
        
        checked_delete_neighbour(link);
      }
      
      link->neighbour=msg->origin;
      link->channel=channel;
      link->sym_time=time-1;
      link->time=time+OLSR_NEIGHB_HOLD_TIME;
    }
    
    
    //update sym time
    link->asym_time=time+OLSR_NEIGHB_HOLD_TIME;
    for(i=0; i<4; i++) {
      //check if we find this node in the link set
      if(msg->neighbour[i]==*node_id) {
        link_type=EXTRACT_BITS(msg->link_type, (6-2*i), 2);
        if(link_type==OLSR_LOST_LINK) {
          link->sym_time=time-1;
        }
        else if(link_type==OLSR_SYM_LINK || link_type==OLSR_ASYM_LINK) {
          link->sym_time=time+OLSR_NEIGHB_HOLD_TIME;
          link->time=link->sym_time+OLSR_NEIGHB_HOLD_TIME;
        }
        break;
      }
    }
    if(link->asym_time>link->time) {
      link->time=link->asym_time;
    }
    
    
    //find and create neighbour entry if it does not exist
    neigh=checked_add_neighbour(msg->origin);
    
    //update neighbour status
    neigh->willingness=msg->willingness;
    neigh->status=OLSR_NOT_SYM;
    for(i=0; i<OLSR_MAX_LINK; i++) {
      if(olsr_link_set[i].neighbour==msg->origin && olsr_link_set[i].sym_time>=time) {
        neigh->status=OLSR_SYM;
      }
    }
    
    //if the originator is a symmetric neighbour then update the 2hop neighbors
    if(neigh->status==OLSR_SYM) {      
                
      for(j=0; j<4; j++) {
        neighbour_type=EXTRACT_BITS(msg->neighbour_type, (6-2*j), 2);
        
        //check if the 2-hop neighbour is this node
        if(msg->neighbour[j]==0 || msg->neighbour[j]==*node_id) continue;
      
        if(neighbour_type==OLSR_NOT_NEIGH) {
          for(k=0; k<OLSR_MAX_2HOP; k++) {
            if(olsr_2hop_set[k].neighbour==msg->origin && olsr_2hop_set[k].two_hop==msg->neighbour[j]) {
              olsr_2hop_set[k].neighbour=0;
              olsr_2hop_set[k].two_hop=0;
              olsr_2hop_set_size--;
            }
          }
        }
        else {
            
          twohop=NULL;  
          for(k=0; k<OLSR_MAX_2HOP; k++) {
            if(olsr_2hop_set[k].neighbour==0) continue;
            
            //check if we have an existing 2hop entry
            if(olsr_2hop_set[k].neighbour==msg->origin && olsr_2hop_set[k].two_hop==msg->neighbour[j]) {
              twohop=&(olsr_2hop_set[k]);
              break;
            }
          }
            
          //create a new 2hop entry if not found
          if(twohop==NULL) {
            if (olsr_2hop_set_size < OLSR_MAX_2HOP) {
              //create new entry
              for (k=0; k<OLSR_MAX_2HOP; k++) {
                if (olsr_2hop_set[k].neighbour == 0) {
                  twohop=&(olsr_2hop_set[k]);
                  olsr_2hop_set_size++;
                  break;
                }
              }
            } 
            else {
              twohop = &(olsr_2hop_set[0]);
              //replace oldest entry
              for (k=1; k<OLSR_MAX_2HOP; k++) {
                if (olsr_2hop_set[k].time < twohop->time) {
                  twohop = &(olsr_2hop_set[k]);
                }
              }
            }
            twohop->neighbour=msg->origin;
            twohop->two_hop=msg->neighbour[j];
          }
            
          twohop->time=time+OLSR_NEIGHB_HOLD_TIME;
        }
      }        
    }
    
    //update the mpr selector set
    for(i=0; i<4; i++) {
      if(msg->neighbour[i]==0) continue;
      
      neighbour_type=EXTRACT_BITS(msg->neighbour_type, (6-2*i), 2);
      
      //check if someone has selected us as mpr
      if(neighbour_type==OLSR_MPR_NEIGH && msg->neighbour[i]==*node_id) {
        mpr=NULL;
        
        for(j=0; j<OLSR_MAX_MPR; j++) {
          if(olsr_mpr_select_set[j].neighbour==0) continue;
          
          //check if we have an existing mpr entry
          if(olsr_mpr_select_set[j].neighbour==msg->origin) {
            mpr=&(olsr_mpr_select_set[j]);
            break;
          }
        }
        
        //create new mpr entry
        if(mpr==NULL) {
          if (olsr_mpr_select_set_size < OLSR_MAX_MPR) {
            //create new entry
            for (k=0; k<OLSR_MAX_MPR; k++) {
              if (olsr_mpr_select_set[k].neighbour == 0) {
                mpr=&(olsr_mpr_select_set[k]);
                olsr_mpr_select_set_size++;
                break;
              }
            }
          } 
          else {
            mpr = &(olsr_mpr_select_set[0]);
            //replace oldest entry
            for (k=1; k<OLSR_MAX_MPR; k++) {
              if (olsr_mpr_select_set[k].time < mpr->time) {
                mpr = &(olsr_mpr_select_set[k]);
              }
            }
          }
          mpr->neighbour=msg->origin;
        }
        
        mpr->time=time+OLSR_NEIGHB_HOLD_TIME;
      }
    }
    atomMutexPut(&olsr_mutex);
}



//check if we should add a neighbour entry
static struct olsr_neighbour *checked_add_neighbour(uint8_t neighbour) {
  uint16_t i,j;
  
  for(j=0; j<OLSR_MAX_NEIGHBOUR; j++) {
    if(olsr_neighbour_set[j].neighbour==neighbour) {
      return &(olsr_neighbour_set[j]);
    }
  }
  
  if (olsr_neighbour_set_size < OLSR_MAX_NEIGHBOUR) {
    //create new entry
    for (i = 0; i < OLSR_MAX_NEIGHBOUR; i++) {
      if (olsr_neighbour_set[i].neighbour == 0) {
        olsr_neighbour_set[i].neighbour=neighbour;
        olsr_neighbour_set_size++;
        return &(olsr_neighbour_set[i]);
      }
    }
  }
  
  return NULL;
}



//check if we should delete the neighbour entry
static void checked_delete_neighbour(struct olsr_link *link) {
  int j;
  uint8_t delete=1;
  
  for(j=0; j<OLSR_MAX_LINK; j++) {
    if(link==&(olsr_link_set[j])) continue;
    
    //check if we have another link to the same neighbour
    if(link->neighbour==olsr_link_set[j].neighbour) {
      delete=0;
      break;
    }
  }
  
  if(delete) {
    for(j=0; j<OLSR_MAX_NEIGHBOUR; j++) {
      if(olsr_neighbour_set[j].neighbour==link->neighbour) {
        olsr_neighbour_set[j].neighbour=0;
        olsr_neighbour_set_size--;
      }
    }
    
    for(j=0; j<OLSR_MAX_2HOP; j++) {
      if(olsr_2hop_set[j].neighbour==link->neighbour) {
        olsr_2hop_set[j].neighbour=0;
        olsr_2hop_set_size--;
      }
    }
    
    for(j=0; j<OLSR_MAX_MPR; j++) {
      if(olsr_mpr_select_set[j].neighbour==link->neighbour) {
        olsr_mpr_select_set[j].neighbour=0;
        olsr_mpr_select_set_size--;
      }
    }
    
  }
  link->neighbour=0;
  olsr_link_set_size--; 
}




uint8_t olsr_process_tc_msg(struct olsr_tc_msg *msg, uint8_t channel) {
  int i, j;
  struct olsr_duplicate *duplicate;
  struct olsr_topology *topology[4];
  uint8_t source;
  uint32_t time;
  uint8_t process; 
  uint8_t forward;
  
  //if the packet has expired, discard package
  if(msg->ttl < 1) return 0;
  
  //if this node sent this packet, do nothing
  if(msg->origin == *node_id) return 0;
  
  //find the node id of the sender of this message
  source=get_remote_node_id(channel);
  
  //if no node id found, discard message
  if(source==0) return 0; 
  
  time=rtc_get_time();
  
  //protect access to routing structures
  atomMutexGet(&olsr_mutex, 0);
  
  forward=1;
  duplicate=check_duplicate(msg->origin, msg->seq_no, source, &forward);
  
  //if we have not received this message before we should process it
  if(duplicate==NULL) {
    process=0;
    for(i=0;i<4;i++) topology[i]=NULL;
    
    //check if have received this message out of order
    for(i=0; i<OLSR_MAX_TOPOLOGY; i++) {
      if(olsr_topology_set[i].dest==0) 
        continue;
      
      if(olsr_topology_set[i].last==msg->origin) {
        //check if there exists a newer topology entry
        if(olsr_topology_set[i].ansn > msg->ansn) {
          process=0;
        }
        //delete older topology entry
        else if(olsr_topology_set[i].ansn < msg->ansn) {
          olsr_topology_set[i].dest=0;
          olsr_topology_set_size--;
        }
        else {
          for(j=0;i<4;j++) {
            if(olsr_topology_set[i].dest==msg->neighbour[j]) {
              topology[j]=&(olsr_topology_set[i]);
            }
          }
        }
      }
    }
    
    if(process) {
      
      for(j=0;j<4;j++) {
        
        if(topology[j]==NULL) {
          if (olsr_topology_set_size < OLSR_MAX_TOPOLOGY) {
            //create new entry
            for (i = 0; i < OLSR_MAX_TOPOLOGY; i++) {
              if (olsr_topology_set[i].dest == 0) {
                topology[j]=&(olsr_topology_set[i]);
                olsr_topology_set_size++;
                break;
              }
            }
          } 
          else {
            topology[j]=&(olsr_topology_set[0]);
            //replace oldest entry
            for (i = 1;i < OLSR_MAX_TOPOLOGY;i++) {
            if (olsr_topology_set[i].time < topology[j]->time) {
                topology[j] = &(olsr_topology_set[i]);
              }
            }
          }
          topology[j]->dest=msg->origin;
          topology[j]->last=msg->neighbour[j];
          topology[j]->ansn=msg->ansn;
          topology[j]->time=time+OLSR_TOP_HOLD_TIME;
        }
      }
    }
  }
  
  //if we should consider forwarding the package
  if(forward) {
    forward=0;
    //check if the ttl is larger than one
    if(msg->ttl>1) {
      //if the message originates from an mpr select node we should retransmit
      forward=olsr_is_selected_mpr();
    }
    
    //update the entry in duplicate set
    update_duplicate(duplicate, msg->origin, msg->seq_no, source, forward, time);
  }
  
  atomMutexPut(&olsr_mutex);
  
  return forward;
}



/*process incoming mid message*/
uint8_t olsr_process_mid_msg(struct olsr_mid_msg *msg, uint8_t channel) {
  int i;
  struct olsr_duplicate *duplicate;
  struct olsr_mid *mid;
  uint8_t source;
  uint32_t time;
  uint8_t forward;
  
  //if the packet has expired, discard package
  if(msg->ttl < 1) return 0;
  
  //if this node sent this packet, do nothing
  if(msg->origin == *node_id) return 0;
  
  //find the node id of the sender of this message
  source=get_remote_node_id(channel);
  
  //if no node id found, discard message
  if(source==0) return 0; 
  
  //protect access to routing structures
  atomMutexGet(&olsr_mutex, 0);
  
  time=rtc_get_time();
  
  forward=1;
  duplicate=check_duplicate(msg->origin, msg->seq_no, source, &forward);
  
  //if we have not received this message before we should process it
  if(duplicate==NULL) {
    mid=NULL;
    for(i=0; i<OLSR_MAX_MID; i++) {
      if(olsr_mid_set[i].origin==0) 
        continue;
      
      if(olsr_mid_set[i].device_id==msg->device_id) {
        mid=&(olsr_mid_set[i]);
      }
      //check if the entry has expired
      else if(olsr_mid_set[i].time<time) {
        olsr_mid_set[i].origin=0;
        olsr_mid_set_size--;
      }
    }
    
    if(mid==NULL) {
      if (olsr_mid_set_size < OLSR_MAX_MID) {
        //create new entry
        for (i = 0; i < OLSR_MAX_MID; i++) {
          if (olsr_mid_set[i].origin == 0) {
            mid=&(olsr_mid_set[i]);
            olsr_mid_set_size++;
            break;
          }
        }
      } 
      else {
        mid=&(olsr_mid_set[0]);
        //replace oldest entry
        for (i = 1;i < OLSR_MAX_MID;i++) {
          if (olsr_mid_set[i].time < mid->time) {
            mid=&(olsr_mid_set[i]);
          }
        }
      }
    }
    mid->origin=msg->origin;
    mid->device_type=msg->device_type;
    mid->device_id=msg->device_id;
    mid->time=time+OLSR_MID_HOLD_TIME;
  }
  
  //if we should consider forwarding the package
  if(forward) {
    forward=0;
    //check if the ttl is larger than one
    if(msg->ttl>1) {
      //if the message originates from an mpr select node we should retransmit
      forward=olsr_is_selected_mpr();
    }
    
    //update the entry in duplicate set
    update_duplicate(duplicate, msg->origin, msg->seq_no, source, forward, time);
  }
  
  atomMutexPut(&olsr_mutex);
  
  return forward;
}



uint8_t olsr_process_hna_msg(struct olsr_hna_msg *msg, uint8_t channel) {
  int i;
  struct olsr_duplicate *duplicate;
  struct olsr_hna *hna;
  uint8_t source;
  uint8_t forward;
  uint32_t time;
  
  //if the packet has expired, discard package
  if(msg->ttl < 1) return 0;
  
  //if this node sent this packet, do nothing
  if(msg->origin == *node_id) return 0;
  
  //find the node id of the sender of this message
  source=get_remote_node_id(channel);
  
  //if no node id found, discard message
  if(source==0) return 0; 
  
  time=rtc_get_time();
  
  //protect access to routing structures
  atomMutexGet(&olsr_mutex, 0);
  
  forward=1;
  duplicate=check_duplicate(msg->origin, msg->seq_no, source, &forward);
  
  //if we have not received this message before we should process it
  if(duplicate==NULL) {
    hna=NULL;
    
    for(i=0; i<OLSR_MAX_HNA; i++) {
      if(olsr_hna_set[i].gateway==0) 
        continue;
      
      if(olsr_hna_set[i].gateway==msg->origin && olsr_hna_set[i].network_id==msg->network_id) {
        hna=&(olsr_hna_set[i]);
      }
      //check if the entry has expired
      else if(olsr_hna_set[i].time<time) {
        olsr_hna_set[i].gateway=0;
        olsr_hna_set_size--;
      }
    }
    
    if(hna==NULL) {
      if (olsr_hna_set_size < OLSR_MAX_HNA) {
        //create new entry
        for (i = 0; i < OLSR_MAX_HNA; i++) {
          if (olsr_hna_set[i].gateway == 0) {
            hna=&(olsr_hna_set[i]);
            olsr_hna_set_size++;
            break;
          }
        }
      } 
      else {
        hna=&(olsr_hna_set[0]);
        //replace oldest entry
        for (i = 1;i < OLSR_MAX_HNA;i++) {
          if (olsr_hna_set[i].time < hna->time) {
            hna=&(olsr_hna_set[i]);
          }
        }
      }
    }
    hna->gateway=msg->origin;
    hna->network_type=msg->network_type;
    hna->network_id=msg->network_id;
    hna->time=time+OLSR_HNA_HOLD_TIME;
  }
  
  //if we should consider forwarding the package
  if(forward) {
    forward=0;
    //check if the ttl is larger than one
    if(msg->ttl>1) {
      //if the message originates from an mpr select node we should retransmit
      forward=olsr_is_selected_mpr();
    }
    
    //update the entry in duplicate set
    update_duplicate(duplicate, msg->origin, msg->seq_no, source, forward, time);
  }
  
  atomMutexPut(&olsr_mutex);
  
  return forward;
}


static struct olsr_duplicate *check_duplicate(uint8_t origin,  uint8_t seq_no, uint8_t source, uint8_t *forward) {
  uint16_t i;
  struct olsr_duplicate *duplicate=NULL;
  
  for (i = 0; i < OLSR_MAX_DUPLICATES; i++) {
    if (olsr_duplicate_set[i].origin == 0) continue;
      
    //check if the duplicate set contains an entry with the same origin and sequence number
    if (olsr_duplicate_set[i].origin == origin && olsr_duplicate_set[i].seq_no == seq_no) {
      duplicate=&(olsr_duplicate_set[i]);
      //check if we should forward the package
      if(duplicate->retransmitted) {
        *forward=0;
      }
      else {
        //if we have received this message from the same source before we should not retransmit
        if(duplicate->source==source) {
          *forward=0;
        }
      }
      return duplicate;
    }
  }
  
  return NULL;
}



static void update_duplicate(struct olsr_duplicate *duplicate, uint8_t origin,  uint8_t seq_no, uint8_t source, uint8_t forward, uint32_t time) {
  uint16_t i;
  
  //update the entry in duplicate set
    if(duplicate==NULL) {
      //create new duplicate entry
      if (olsr_duplicate_set_size < OLSR_MAX_DUPLICATES) {
        //create new entry
        for (i = 0; i < OLSR_MAX_DUPLICATES; i++) {
          if (olsr_duplicate_set[i].origin == 0) {
            duplicate=&(olsr_duplicate_set[i]);
            olsr_duplicate_set_size++;
            break;
          }
        }
      } 
      else {
        duplicate=&(olsr_duplicate_set[0]);
        //replace oldest entry
        for (i = 1;i < OLSR_MAX_DUPLICATES;i++) {
          if (olsr_duplicate_set[i].time < duplicate->time) {
            duplicate=&(olsr_duplicate_set[i]);
          }
        }
      }
      duplicate->origin=origin;
      duplicate->seq_no=seq_no;
    }
    duplicate->time=time+OLSR_DUP_HOLD_TIME;
    duplicate->retransmitted=forward;
    duplicate->source=source;
}



static void calc_mpr_set(void) {
  int i, j, k;
  uint8_t end;
  uint8_t degree[OLSR_MAX_NEIGHBOUR];
  uint8_t order[OLSR_MAX_NEIGHBOUR];
  
  //indicates from how many neighbors this 2-hop node can be reached
  uint8_t reachability[OLSR_MAX_2HOP];
  
  for(i=0;i<OLSR_MAX_NEIGHBOUR;i++) {
    if(olsr_neighbour_set[i].willingness==OLSR_WILL_ALWAYS) {
      olsr_neighbour_set[i].mpr=1;
    }
    else {
      olsr_neighbour_set[i].mpr=0;
    }
  }
  
  //calc degree of neighbour nodes
  for(i=0;i<OLSR_MAX_NEIGHBOUR;i++) {
    degree[i]=0;
    if(!olsr_neighbour_set[i].neighbour) continue;
    
    for(j=0;j<OLSR_MAX_2HOP;j++) {
      reachability[j]=0;
      if(olsr_2hop_set[j].neighbour==0) continue;
      
      if(olsr_neighbour_set[i].neighbour==olsr_2hop_set[j].neighbour) {
        //check that the 2-hop neighbour is not this node
        if(olsr_2hop_set[j].two_hop!=*node_id) {
          //exclude all nodes in the neighbour set
          for(k=0;k<OLSR_MAX_NEIGHBOUR;k++) {
            if(olsr_neighbour_set[k].neighbour==0) continue;
            
            if(olsr_2hop_set[j].two_hop==olsr_neighbour_set[k].neighbour) {
              break;
            }
          }
          //if it is not in the neighbour set, increase degree and reachability
          if(k==OLSR_MAX_NEIGHBOUR) {
            degree[i]++;
            reachability[j]++;
          }
        }
      }
    }
  }
  
  //add to the MPR set those nodes in N, which are the *only* nodes 
  //to provide reachability to a node in N2
  for(i=0;i<OLSR_MAX_2HOP;i++) {
    if(olsr_2hop_set[i].neighbour==0) continue;
    
    if(reachability[i]==1) {
      for(j=0;j<OLSR_MAX_NEIGHBOUR;j++) {
        if(olsr_neighbour_set[j].neighbour==0) continue;
        
        if(olsr_2hop_set[i].neighbour==olsr_neighbour_set[j].neighbour) {
          olsr_neighbour_set[j].mpr=1;
          break;
        }
      }
    }
  }
    
  //remove the nodes from N2 which are now covered by a node in the MPR set
  for(i=0;i<OLSR_MAX_2HOP;i++) {
    if(olsr_2hop_set[i].neighbour==0) continue;
    
    if(reachability[i]>1) {
      for(j=0;j<OLSR_MAX_NEIGHBOUR;j++) {
        if(olsr_neighbour_set[j].neighbour==0) continue;
        
        if(olsr_2hop_set[i].neighbour==olsr_neighbour_set[j].neighbour) {
          if(olsr_neighbour_set[j].mpr) reachability[i]=0;
          break;
        }
      }
    }
  }
  
  for(;;) {
    end=1;
    for(i=0;i<OLSR_MAX_NEIGHBOUR;i++) {
      order[i]=0;
      if(olsr_neighbour_set[i].neighbour==0) continue;
      
      for(j=0;j<OLSR_MAX_2HOP;j++) {
        if(olsr_2hop_set[j].neighbour==0) continue;
        
        if(reachability[j]>1) {
          if(olsr_neighbour_set[i].neighbour==olsr_2hop_set[j].neighbour) {
            end=0;
            order[i]++;
          }
        }
      }
    }
    
    if(end) break;
    
    //select mpr
    k=-1;
    for(i=0;i<OLSR_MAX_NEIGHBOUR;i++) {
      if(olsr_neighbour_set[i].neighbour==0) continue;
      
      if(order[i]>0) {
        if(k==-1) {
          k=i;
          continue;
        }
        
        if(olsr_neighbour_set[i].willingness>olsr_neighbour_set[k].willingness) {
          k=i;
        }
        else if(olsr_neighbour_set[i].willingness==olsr_neighbour_set[k].willingness) {
          if(order[i]>order[k]) {
            k=i;
          }
          else if(order[i]==order[k]) {
            if(degree[i]>degree[k]) {
              k=i;
            }
          }
        }
      }
    }
    if(k!=-1) olsr_neighbour_set[k].mpr=1;
    else break;
    
    //remove the nodes from N2 which are now covered by a node in the MPR set
    for(i=0;i<OLSR_MAX_2HOP;i++) {
      if(olsr_2hop_set[i].neighbour==0) continue;
      
      if(reachability[i]>1) {
        for(j=0;j<OLSR_MAX_NEIGHBOUR;j++) {
          if(olsr_neighbour_set[j].neighbour==0) continue;
          
          if(olsr_2hop_set[i].neighbour==olsr_neighbour_set[j].neighbour) {
            if(olsr_neighbour_set[j].mpr) reachability[i]=0;
            break;
          }
        }
      }
    }
  }
}



static void calc_route_table() {
  int i, j, k, index;
  uint8_t add;
  
  //protect access to routing structures
  atomMutexGet(&olsr_mutex, 0);
  
  index=0;
  
  //add the immediate neighbors
  for(i=0;i<OLSR_MAX_NEIGHBOUR;i++) {
    if(olsr_neighbour_set[i].neighbour==0) continue;
    
    olsr_route_table[index].dest=olsr_neighbour_set[i].neighbour;
    olsr_route_table[index].next=olsr_neighbour_set[i].neighbour;
    olsr_route_table[index].hops=1;
    index++;
  }
  
  /*
  for each node in N2, i.e., a 2-hop neighbor which is not a
  neighbor node or the node itself, and such that there exist at
  least one entry in the 2-hop neighbor set where
  N_neighbor_main_addr correspond to a neighbor node with
  willingness different of WILL_NEVER
 */
  for(i=0;i<OLSR_MAX_2HOP;i++) {
    if(olsr_2hop_set[i].neighbour==0) continue;
    //do not add itself
    if(olsr_2hop_set[i].two_hop==*node_id) continue;
  
    //check if it is a direct neighbour
    for(j=0;j<OLSR_MAX_NEIGHBOUR;j++) {
      if(olsr_neighbour_set[j].neighbour==0) continue;
        
      if(olsr_2hop_set[i].two_hop==olsr_neighbour_set[j].neighbour) {
        break;
      }
    }
    
    //if it is not a direct neighbor, we should add it
    if(j==OLSR_MAX_NEIGHBOUR) {
      olsr_route_table[index].dest=olsr_2hop_set[i].two_hop;
      olsr_route_table[index].next=olsr_2hop_set[i].neighbour;
      olsr_route_table[index].hops=2;
      index++;
    }
  }
  
  //look further than 2 hops
  for(i=2;i<OLSR_MAX_HOPS;i++) {
    add=TRUE;
    for(j=0;j<OLSR_MAX_TOPOLOGY;j++) {
      if(olsr_topology_set[j].dest==0) continue;
      
      //check if we have a route to this node already
      for(k=0;k<index;k++) {
        if(olsr_route_table[k].dest==olsr_topology_set[j].dest) {
          add=FALSE;
          break;
        }
      }
      
      //if we do not have a route to the destination, 
      //check if we have a route of the correct length to its neighbor
      if(add==TRUE) {
        add=FALSE;
        for(k=index;k!=0;k--) {
          if(olsr_route_table[k].hops==i && olsr_route_table[k].dest==olsr_topology_set[j].last) {
            add=TRUE;
            break;
          }
          else if(olsr_route_table[k].hops<i) {
            //no point in checking shorter paths as they should already have been found
            add=FALSE;
            break;
          }          
        }
      }    
      
      if(add==TRUE) {
        //add this 
        olsr_route_table[index].dest=olsr_topology_set[j].dest;
        olsr_route_table[index].next=olsr_topology_set[j].last;
        olsr_route_table[index].hops=i+1;
        index++;
      }
    }
  }
  
  //clear the unused entries of the routing table
  while(index<OLSR_MAX_ROUTE_TABLE) {
    olsr_route_table[index++].dest=0;
  }
  
  atomMutexPut(&olsr_mutex);
  
  return;
}


uint8_t olsr_route_next_node(uint8_t dst) {
  int i;
  
  calc_route_table();
  
  for(i=0;i<OLSR_MAX_ROUTE_TABLE;i++) {
    if(olsr_route_table[i].dest==0) {
      return 0;
    }
    else if(olsr_route_table[i].dest==dst) {
      return olsr_route_table[i].next;
    }
  }
  return 0;
}


uint8_t olsr_is_selected_mpr() {
  int i;
  
  for(i=0;i<OLSR_MAX_NEIGHBOUR;i++) {
    if(olsr_mpr_select_set[i].neighbour!=0) {
      return 1;
    }
  }
  return 0;
}



uint8_t olrs_find_node_id(uint8_t device_type, uint32_t device_id) {
  int i;
  
  for(i=0;i<OLSR_MAX_MID;i++) {
    if(olsr_mid_set[i].origin!=0) { 
      if((device_type==0 || olsr_mid_set[i].device_type==device_type)
          && (device_id==0 || olsr_mid_set[i].device_id==device_id)) {
        return olsr_mid_set[i].origin;
      }
    }  
  }
  return 0;
}