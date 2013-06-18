#include "ant_core.h"

static usart_rs232_options_t ANT_USART_OPTIONS = {
    .baudrate = ANT_UART_BAUDRATE,
    .charlength = USART_CHSIZE_8BIT_gc,
    .paritytype = USART_PMODE_DISABLED_gc,
    .stopbits = false
};

struct ant_channel ant_core_channels[ANT_NO_OF_CHANNELS];

static ATOM_MUTEX ant_mutex;

static ATOM_QUEUE ant_event_queue;
static uint8_t ant_event_queue_data[ANT_CORE_EVENT_QUEUE_SIZE*sizeof(struct ant_event)];

static ATOM_QUEUE ant_tx_queue;
static uint8_t ant_tx_queue_data[ANT_CORE_TX_QUEUE_SIZE*sizeof(struct ant_frame)];

static ATOM_QUEUE ant_rx_queue;
static uint8_t ant_rx_queue_data[ANT_CORE_RX_QUEUE_SIZE];

static uint8_t msg_buf[ANT_CORE_RX_QUEUE_SIZE];
static uint8_t *msg_buf_ptr;
static uint16_t msg_buf_len;
static uint8_t msg_block_len;
static uint8_t msg_block_offset;

static io_input_stream_t ant_rx_stream;

static volatile uint8_t rx_pos=0;
static volatile uint8_t rx_checksum=0;
static volatile struct ant_frame current_rx_frame;

static volatile uint8_t tx_pos=0x0;
static volatile uint8_t tx_checksum=0;
static volatile struct ant_frame current_tx_frame;

//forward declarations
static uint8_t ant_rx_interrupt_handler(void);
static uint8_t ant_tx_interrupt_handler(void);
static uint8_t ant_dre_interrupt_handler(void);
static uint8_t ant_core_process_rx_frame(void);
static void ant_core_start_tx(void);
static int8_t ant_core_next_event(struct ant_event *evt, uint32_t timeout);
static int8_t ant_core_send(struct ant_frame *msg, int32_t timeout);
static int8_t ant_core_send_blocking(uint8_t channel, struct ant_frame *msg, int32_t timeout);


//interrupt handler for ant receive complete
ISR(USARTD0_RXC_vect)
{
  atomIntEnter();
  atomIntExit(FALSE, ant_rx_interrupt_handler());
}

//interrupt handler for ant data register empty
ISR(USARTD0_DRE_vect)
{
  atomIntEnter();
  atomIntExit(FALSE, ant_dre_interrupt_handler());
}

//interrupt handler for ant transmit complete
ISR(USARTD0_TXC_vect)
{ 
  atomIntEnter();
  atomIntExit(FALSE, ant_tx_interrupt_handler());
}

/* Interrupt handler for rx events */
ISR(ANT_RX_INT_VECTOR) {
  atomIntEnter();
  ANT_RX_PORT->INTFLAGS=0;
  atomIntExit(FALSE, FALSE);
}

/* Interrupt handler for rts events */
ISR(ANT_RTS_INT_VECTOR) {
  atomIntEnter();
  if (ioport_get_pin_level(nRF24AP2_RTS)) {
    usart_set_dre_interrupt_level(ANT_USART, USART_DREINTLVL_OFF_gc);
  }
  else {
    usart_set_dre_interrupt_level(ANT_USART, USART_DREINTLVL_LO_gc);
  }
  ANT_RTS_PORT->INTFLAGS=0;
  atomIntExit(FALSE, FALSE);
}

static inline void ant_core_start_tx(void) {
  ant_core_set_rts_port_interrupt_level(PORT_INT0LVL_LO_gc);
  ioport_set_pin_level(nRF24AP2_SLEEP, false);
  if (!ioport_get_pin_level(nRF24AP2_RTS)) {
    usart_set_dre_interrupt_level(ANT_USART, USART_DREINTLVL_LO_gc);
  }  
}

static void inline ant_core_stop_tx(void) {
    usart_set_dre_interrupt_level(ANT_USART, USART_DREINTLVL_OFF_gc);
    ant_core_set_rts_port_interrupt_level(PORT_INT0LVL_OFF_gc);
    ioport_set_pin_level(nRF24AP2_SLEEP, true);
}


void ant_core_set_rx_port_interrupt_level(uint8_t level) {
  ANT_RX_PORT->INTCTRL = (ANT_RX_PORT->INTCTRL & ~ PORT_INT0LVL_gm) | level;
}

void ant_core_set_rts_port_interrupt_level(uint8_t level) {
  ANT_RTS_PORT->INTCTRL = (ANT_RTS_PORT->INTCTRL & ~ PORT_INT0LVL_gm) | level;
}


void ant_core_init() {
  uint8_t i;
  
  // Initialize ant usart driver in RS232 mode
  usart_init_rs232(ANT_USART, &ANT_USART_OPTIONS);
  
  atomMutexCreate(&ant_mutex);
  
  atomQueueCreate (&ant_event_queue, ant_event_queue_data, sizeof(struct ant_event), ANT_CORE_EVENT_QUEUE_SIZE);
  atomQueueCreate (&ant_tx_queue, ant_tx_queue_data, sizeof(struct ant_frame), ANT_CORE_TX_QUEUE_SIZE);
  atomQueueCreate (&ant_rx_queue, ant_rx_queue_data, sizeof(uint8_t), ANT_CORE_RX_QUEUE_SIZE);
  for(i=0;i<ANT_NO_OF_CHANNELS;i++) {
    atomSemCreate(&ant_core_channels[i].rx_sem, 0);
  }
  for(i=0;i<ANT_NO_OF_CHANNELS;i++) {
    atomSemCreate(&ant_core_channels[i].tx_sem, 0);
  }
  ant_rx_stream=io_input_stream_from_queue(&ant_rx_queue);
  ant_rx_stream.timeout=ANT_CORE_TIMEOUT;
  
  ioport_set_pin_level(nRF24AP2_nRESET, true);
  ant_core_sleep(FALSE);
  ant_core_suspend(FALSE);
  
  usart_set_rx_interrupt_level(ANT_USART, USART_RXCINTLVL_LO_gc);
  usart_set_tx_interrupt_level(ANT_USART, USART_TXCINTLVL_LO_gc);
  
  ANT_RX_PORT->INT0MASK = ANT_RX_PORT->INT0MASK | ioport_pin_to_mask(nRF24AP2_TX);
  ANT_RX_PORT->INTCTRL = (ANT_RX_PORT->INTCTRL & ~ PORT_INT0LVL_gm) | PORT_INT0LVL_OFF_gc;
  ANT_RX_PORT->PIN2CTRL = IOPORT_SENSE_BOTHEDGES;
  
  ANT_RTS_PORT->INT0MASK = ANT_RTS_PORT->INT0MASK | ioport_pin_to_mask(nRF24AP2_RTS);
  ANT_RTS_PORT->INTCTRL = (ANT_RTS_PORT->INTCTRL & ~ PORT_INT0LVL_gm) | PORT_INT0LVL_LO_gc;
  ANT_RTS_PORT->PIN2CTRL = IOPORT_SENSE_BOTHEDGES;
  
  return;
}



void ant_core_reset()
{
  uint8_t i;
  struct ant_frame buf;
  irqflags_t flags=cpu_irq_save();
  
  memset(&ant_core_channels, 0, sizeof(ant_core_channels));
  
  while(atomQueueGet(&ant_event_queue, -1, (uint8_t *)&buf)==ATOM_OK);
  while(atomQueueGet(&ant_rx_queue, -1, (uint8_t*)&buf)==ATOM_OK);
  rx_pos=0;
  //rx_byte=0;
  
  while(atomQueueGet(&ant_tx_queue, -1, (uint8_t*)&buf)==ATOM_OK);
  tx_pos=0x0;
  
  for(i=0;i<ANT_NO_OF_CHANNELS;i++) {
    atomSemResetCount(&ant_core_channels[i].rx_sem, 0);
  }
  for(i=0;i<ANT_NO_OF_CHANNELS;i++) {
    atomSemResetCount(&ant_core_channels[i].tx_sem, 0);
  }
  
  ioport_set_pin_level(nRF24AP2_nRESET, false);
  _delay_us(10);
  ioport_set_pin_level(nRF24AP2_nRESET, true);
  
  cpu_irq_restore(flags);
  
  ant_core_reset_state();
  
  return;
}

void ant_core_clear_rx_stream(uint16_t len) {
  uint8_t buf;
  if(len) {
    for(;len!=0 && atomQueueGet(&ant_rx_queue, -1, &buf)==ATOM_OK;len--);
  }
  else {
    while(atomQueueGet(&ant_rx_queue, -1, &buf)==ATOM_OK);
  }
}

void ant_core_sleep(uint8_t state)
{
  if (state) {
    ioport_set_pin_level(nRF24AP2_SLEEP, true);
    ant_core_set_rx_port_interrupt_level(PORT_INT0LVL_LO_gc);
  }  
  else {
    sysclk_enable_peripheral_clock(ANT_USART);
    ant_core_set_rx_port_interrupt_level(PORT_INT0LVL_OFF_gc);
  }  
  
  return;
}


void ant_core_suspend(uint8_t state)
{
  if (state)
  {
    ioport_set_pin_level(nRF24AP2_SLEEP, true);
    ioport_set_pin_level(nRF24AP2_nSUSPEND, false);
    _delay_us(100);
    ioport_set_pin_level(nRF24AP2_nSUSPEND, true);    
  }
  else
  {
    ioport_set_pin_level(nRF24AP2_nSUSPEND, true);  // note: needed for init...
    ioport_set_pin_level(nRF24AP2_SLEEP, false);
  }
  return;
}


/*
This method handles all events and data from the ant module
and forwards to the correct handler
*/
void ant_core_start(void (*callback)(void)) {
  struct ant_event evt;
  
  ant_core_reset();
  
  while(ant_core_next_event(&evt, ANT_CORE_RECEIVE_TIMEOUT)==STATUS_OK) {
    switch(evt.event_code) {
      case ANT_NOTIFICATION_STARTUP_MESSAGE:
        ant_core_enable_crystal();
        callback();
        break;    
      case ANT_DATA_BROADCAST:
      case ANT_DATA_ACKNOWLEDGE:
      case ANT_DATA_BURST_TRANSFER:
      case ANT_EVENT_RX_SEARCH_TIMEOUT:
      case ANT_EVENT_TRANSFER_TX_COMPLETED:
      case ANT_EVENT_TRANSFER_TX_FAILED:
      case ANT_EVENT_TX:
      case ANT_EVENT_RX_FAIL:
      case ANT_EVENT_RX_FAIL_GO_TO_SEARCH:
      case ANT_EVENT_CHANNEL_CLOSED:
        if(ant_core_channels[evt.channel].callback) {
          ant_core_channels[evt.channel].callback(&evt);
        }
        else {
          ant_core_clear_rx_stream(evt.data_len);
        }                 
        break;
      case ANT_EVENT_TRANSFER_TX_START:
      case ANT_EVENT_TRANSFER_RX_FAILED:
      case ANT_EVENT_CHANNEL_COLLISION:
      default:
        break;
    }
  }
}


static inline uint8_t ant_tx_interrupt_handler(void) {
  tx_pos=0x0;
  return FALSE;
}


static inline uint8_t ant_dre_interrupt_handler(void) {
  uint8_t tx_byte;
  uint8_t sched=FALSE;
  
  if(ioport_get_pin_level(nRF24AP2_RTS)) return sched;
  
  if(tx_pos==0) {
    if(atomQueueGet(&ant_tx_queue, -1, (uint8_t *)&current_tx_frame)==ATOM_OK)
    {
      ant_core_start_tx();
      //since we removed an element from the queue, 
      //we must call the scheduler when we exit the ISR 
      sched=TRUE;
    
      if(current_tx_frame.msg_id!=0) {
        tx_byte=ANT_SYNC_BYTE;
      }
      else {
        tx_byte=0;
      }                
    }
    else {
      //nothing to send
      ant_core_stop_tx();
      return FALSE;
    }
    
    tx_checksum=tx_byte;
  }
  else if(tx_pos==1) {
    if(current_tx_frame.msg_id!=0) {
      tx_byte=current_tx_frame.data_len;
    }
    else {
      tx_byte=0;
    }
    tx_checksum^=tx_byte;
  }
  else if(tx_pos==2) {
    tx_byte=current_tx_frame.msg_id;
    tx_checksum^=tx_byte;
  }
  else if(tx_pos<(3+current_tx_frame.data_len)) {
    tx_byte=current_tx_frame.data[tx_pos-3];
    tx_checksum^=tx_byte;
  }
  else if(tx_pos==(3+current_tx_frame.data_len)) {
    tx_byte=tx_checksum;
    //send some zero byte padding between frames
    tx_pos=0xFC;
  }  
  else {
    tx_byte=0;
  }
  
  (ANT_USART)->DATA = tx_byte;
  tx_pos++;
  return sched;
}

static inline uint8_t ant_rx_interrupt_handler(void) {
  uint8_t rx_byte; 
  
  if((ANT_USART)->STATUS & USART_BUFOVF_bm) {
    rx_pos=0;
    rx_byte = (uint8_t)(ANT_USART)->DATA;
    return FALSE;
  }
  
  rx_byte = (uint8_t)(ANT_USART)->DATA;
  
  //if we are at the start of a new frame we wait for a sync byte
  if(rx_pos==0 && rx_byte!=ANT_SYNC_BYTE) {
    return FALSE;
  }
    
  if(rx_pos==0) {
    //sync byte
    rx_checksum=ANT_SYNC_BYTE;
    rx_pos++;
  }
  else if(rx_pos==1) {
    //length of data byte
    if(rx_byte>16) {
      rx_pos=0;
      return FALSE;
    }    
    current_rx_frame.data_len=rx_byte;
    rx_checksum^=rx_byte;
    rx_pos++;
  }
  else if(rx_pos==2) {
    //message id
    current_rx_frame.msg_id=rx_byte;
    rx_checksum^=rx_byte;
    rx_pos++;
  }
  else if(rx_pos<(3+current_rx_frame.data_len)) {
    //data byte
    current_rx_frame.data[rx_pos-3]=rx_byte;
    rx_checksum ^= rx_byte;
    rx_pos++;
  }
  else if(rx_pos==(3+current_rx_frame.data_len)) {
    rx_pos=0;
    
    //verify checksum
    if(rx_checksum==rx_byte) {
      //if checksum ok then process frame
      return ant_core_process_rx_frame();
    }
    else {
      //else discard message
      rx_checksum=0;
    }
  }
  else {
    rx_pos=0;
  }
  //no new frame available, do not run scheduler
  return FALSE;
}


static uint8_t ant_core_process_rx_frame() {
  uint8_t i, seq, resp_msg_id;
  struct ant_event evt;
  evt.stream=&ant_rx_stream;
  
  switch(current_rx_frame.msg_id) {
    case ANT_EVENT_CHANNEL_RESPONSE:
      evt.channel=current_rx_frame.data[0];
      resp_msg_id=current_rx_frame.data[1];
      evt.event_code=current_rx_frame.data[2];
      evt.data_len=0;
      
      switch(resp_msg_id) {
        case 1:
          //ant event
          switch(evt.event_code) {
            case ANT_EVENT_TRANSFER_TX_FAILED:
            case ANT_EVENT_TRANSFER_TX_COMPLETED:
              ant_core_channels[evt.channel].event_code=evt.event_code;
              atomSemPut(&ant_core_channels[evt.channel].rx_sem);
            //fall through here
            case ANT_EVENT_TX:
              if(ant_core_channels[evt.channel].master) {
                atomSemPut(&ant_core_channels[evt.channel].tx_sem);
              
                if(atomSemGet(&ant_core_channels[evt.channel].tx_sem, -1)==ATOM_OK) {
                  //send broadcast
                  if(atomQueuePut(&ant_tx_queue, -1, (uint8_t *)&ant_core_channels[evt.channel].broadcast_frame)==ATOM_OK) {
                    ant_core_start_tx();
                    if(ant_core_channels[evt.channel].shared) {
                      memset(ant_core_channels[evt.channel].broadcast_frame.data+3, 0x00, 6);
                    }
                    else {
                      memset(ant_core_channels[evt.channel].broadcast_frame.data+1, 0x00, 8);
                    }
                  }
                }
              }
              atomQueuePut(&ant_event_queue, -1, (uint8_t *)&evt);  
              break;
            case ANT_EVENT_TRANSFER_TX_START:
              ant_core_start_tx();
              break;
            case ANT_EVENT_RX_FAIL_GO_TO_SEARCH:
              atomQueuePut(&ant_event_queue, -1, (uint8_t *)&evt);
              break;
            case ANT_EVENT_CHANNEL_CLOSED:
              ant_core_channels[evt.channel].status=ANT_CHANNEL_STATUS_ASSIGNED;
              atomQueuePut(&ant_event_queue, -1, (uint8_t *)&evt);
              break;
            case ANT_EVENT_RX_SEARCH_TIMEOUT:
            case ANT_EVENT_TRANSFER_RX_FAILED:
            case ANT_EVENT_RX_FAIL:
            case ANT_EVENT_CHANNEL_COLLISION:
            default:
              atomQueuePut(&ant_event_queue, -1, (uint8_t *)&evt);
              break;
          }
          
          break;
        default:
          if(evt.event_code!=ANT_RESPONSE_NO_ERROR) {
            //command response
            switch(evt.event_code) {
              //case ANT_RESPONSE_NO_ERROR:
              case ANT_TRANSFER_IN_PROGRESS:
              case ANT_TRANSFER_SEQUENCE_NUMBER_ERROR:
              case ANT_TRANSFER_IN_ERROR:
                break;
              case ANT_CHANNEL_ID_NOT_SET:
              case ANT_CLOSE_ALL_CHANNELS:
              case ANT_INVALID_MESSAGE:
              case ANT_INVALID_NETWORK_NUMBER:
              case ANT_INVALID_LIST_ID:
              case ANT_INVALID_SCAN_TX_CHANNEL:
              case ANT_INVALID_PARAMETER_PROVIDED:
              case ANT_CHANNEL_IN_WRONG_STATE:
              case ANT_CHANNEL_NOT_OPENED:
              default:
                break;
            }
          }
          ant_core_channels[evt.channel].event_code=evt.event_code;
          atomSemPut(&ant_core_channels[evt.channel].rx_sem);
          atomSemGet(&ant_core_channels[evt.channel].rx_sem, -1);
          break;
      }        
      break;  
    case ANT_DATA_ACKNOWLEDGE:
    case ANT_DATA_BROADCAST:
      evt.channel=current_rx_frame.data[0];
      evt.event_code=current_rx_frame.msg_id;
      evt.data_len=ant_core_channels[evt.channel].shared ? 6 : 8;
      
      msg_block_offset=9-evt.data_len;
      
      if(evt.data_len==6) {
        memcpy(&evt.shared_address, (void *)(current_rx_frame.data+1), 2);
      }
      else {
        evt.shared_address=0;
      }
      if(atomQueuePut(&ant_event_queue, -1, (uint8_t *)&evt)==ATOM_OK) {
        for(i=msg_block_offset;i<9;i++) {
          atomQueuePut(&ant_rx_queue, -1, (uint8_t *)current_rx_frame.data+i);
        }
      }
      
      if(!ant_core_channels[evt.channel].master) {
        
        if(atomSemGet(&ant_core_channels[evt.channel].tx_sem, -1)==ATOM_OK) {
          if(ant_core_channels[evt.channel].broadcast_frame.data_len > 0) {
            //send broadcast
            if(atomQueuePut(&ant_tx_queue, -1, (uint8_t *)&ant_core_channels[evt.channel].broadcast_frame)==ATOM_OK) {
              ant_core_start_tx();
              ant_core_channels[evt.channel].broadcast_frame.data_len=0;
              if(ant_core_channels[evt.channel].shared) {
                memset(ant_core_channels[evt.channel].broadcast_frame.data+3, 0x00, 6);
              }
              else {
                memset(ant_core_channels[evt.channel].broadcast_frame.data+1, 0x00, 8);
              }
            }
          }            
        }
        else {
          atomSemPut(&ant_core_channels[evt.channel].tx_sem);
        }
      }
      break;
    case ANT_DATA_BURST_TRANSFER:
      seq=((current_rx_frame.data[0] & 0xE0) >> 5);
      
      if(seq==0) {
        msg_buf_len=0;
        msg_block_len=ant_core_channels[current_rx_frame.data[0]].shared ? 6 : 8;
        msg_block_offset=9-msg_block_len;
        msg_buf_ptr=msg_buf;
      }
      
      msg_buf_len+=msg_block_len;
      if(msg_buf_len>ANT_CORE_RX_QUEUE_SIZE) {
        break;
      }
      memcpy(msg_buf_ptr, (uint8_t *)current_rx_frame.data+msg_block_offset, msg_block_len);
      msg_buf_ptr+=msg_block_len;
      
      //last frame in burst
      if((seq & 0x4)!=0x0) {
        //copy complete burst to rx_queue
        evt.channel=current_rx_frame.data[0] & 0x1F;
        evt.event_code=current_rx_frame.msg_id;
        evt.data_len=msg_buf_len;
        if(msg_block_len==6) {
          memcpy(&evt.shared_address, (void *)(current_rx_frame.data+1), 2);
        }
        else {
          evt.shared_address=0;
        }
        if((ant_rx_queue.max_num_msgs-ant_rx_queue.num_msgs_stored)>=msg_buf_len) {
          msg_buf_ptr=msg_buf;
          if(atomQueuePut(&ant_event_queue, -1, (uint8_t *)&evt)==ATOM_OK) {
            for(;msg_buf_len!=0;msg_buf_len--) {
              if(atomQueuePut(&ant_rx_queue, -1, msg_buf_ptr++)!=ATOM_OK) {
                msg_buf_ptr++;
              }        
            }
            return TRUE;
          }            
        }
        //if no space on queue then drop message
      }
      return FALSE;
    case ANT_REQUEST_CHANNEL_STATUS:
      evt.channel=current_rx_frame.data[0];
      ant_core_channels[evt.channel].status=current_rx_frame.data[1] & 0x03;
      atomSemPut(&ant_core_channels[evt.channel].rx_sem);
      atomSemGet(&ant_core_channels[evt.channel].rx_sem, -1);
      break;
    case ANT_REQUEST_CHANNEL_ID:
      evt.channel=current_rx_frame.data[0];
      
      ant_core_channels[evt.channel].device_no=current_rx_frame.data[1];
      ant_core_channels[evt.channel].device_no|=((0xFFFFFFFF & current_rx_frame.data[2]) << 8);
      ant_core_channels[evt.channel].device_type=current_rx_frame.data[3] & 0x7F;
      ant_core_channels[evt.channel].pairing=(current_rx_frame.data[3] & 0x80) >> 7;
      ant_core_channels[evt.channel].tx_type=current_rx_frame.data[4];
      
      atomSemPut(&ant_core_channels[evt.channel].rx_sem);
      atomSemGet(&ant_core_channels[evt.channel].rx_sem, -1);
      break;
    case ANT_NOTIFICATION_STARTUP_MESSAGE:
      evt.channel=0;
      evt.event_code=ANT_NOTIFICATION_STARTUP_MESSAGE;
      evt.data_len=0;
      atomQueuePut(&ant_event_queue, -1, (uint8_t *)&evt);
      break;
    default:
      break;
  }
  return TRUE;
}


static int8_t ant_core_next_event(struct ant_event *evt, uint32_t timeout) {
  if(atomQueueGet(&ant_event_queue, timeout, (uint8_t *)evt)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  return STATUS_OK;
}

static int8_t ant_core_send_blocking(uint8_t channel, struct ant_frame *msg, int32_t timeout) {
  
  atomSemResetCount(&ant_core_channels[channel].rx_sem, 0);
  
  if(atomMutexGet(&ant_mutex, timeout)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  if(atomQueuePut(&ant_tx_queue, timeout, (uint8_t *)msg)!=ATOM_OK) {
    atomMutexPut(&ant_mutex);
    return ERR_TIMEOUT;
  }
  
  ant_core_start_tx();
  
  atomMutexPut(&ant_mutex);
  
  if(atomSemGet(&ant_core_channels[channel].rx_sem, timeout)!=STATUS_OK) {
    //atomMutexPut(&ant_mutex);
    return ERR_TIMEOUT; 
  }
  
  return ant_core_channels[channel].event_code;
}


static int8_t ant_core_send(struct ant_frame *msg, int32_t timeout) {
  
  if(atomMutexGet(&ant_mutex, timeout)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  if(atomQueuePut(&ant_tx_queue, timeout, (uint8_t *)msg)!=ATOM_OK) {
    atomMutexPut(&ant_mutex);
    return ERR_TIMEOUT;
  }
  
  ant_core_start_tx();
  
  atomMutexPut(&ant_mutex);
  return STATUS_OK;
}


int8_t ant_core_config(uint8_t channel, struct ant_config *config) {
  uint8_t type, ext;
  int8_t status;
  
  ant_core_channels[channel].master=config->master;
  ant_core_channels[channel].shared=config->shared;
  
  type=0;
  if(config->master) {
    if(config->shared) {
      type=ANT_SHARED_MASTER;
    }
    else {
      type=ANT_MASTER;
    }
  }
  else {
    if(config->shared) {
      type=ANT_SHARED_SLAVE;
    }
    else {
      type=ANT_SLAVE;
    }
  }
  
  ext=0;
  if(config->bg_scan) {
    ext |= ANT_ENABLE_BACKGROUND_SCANNING;
  }
  
  if(config->freq_agility) {
    ext |= ANT_ENABLE_FREQUENCY_AGILITY;
  }
  
  if((status=ant_core_assign_channel(channel, type, ANT_PUBLIC_NETWORK, ext))!=STATUS_OK) {
    return status;
  }
  if((status=ant_core_set_channel_id(channel, config->device_no, config->pairing, config->device_type, config->tx_type))!=STATUS_OK) {
    return status;
  }
  if((status=ant_core_set_channel_rf_frequency(channel, config->frequency))!=STATUS_OK) {
    return status;
  }
  if((status=ant_core_set_channel_period(channel, config->period))!=STATUS_OK) {
    return status;
  }
  if((status=ant_core_set_channel_tx_power(channel, config->tx_power))!=STATUS_OK) {
    return status;
  }
  if((status=ant_core_set_channel_search_timeout(channel, config->timeout_hp))!=STATUS_OK) {
    return status;
  }
  if((status=ant_core_set_channel_low_prio_search_timeout(channel, config->timeout_lp))!=STATUS_OK) {
    return status;
  }
  //not supported apparently
  //if((status=ant_core_set_channel_search_priority(channel, config->search_priority))!=STATUS_OK) {
  //  return status;
  //}
  if((status=ant_core_set_proximity_search(channel, config->proximity))!=STATUS_OK) {
    return status;
  }
  if(config->freq_agility) {
    if((status=ant_core_config_frequency_agility(channel, config->freq[0], config->freq[1], config->freq[2]))!=STATUS_OK) {
      return status;
    }
  }
  return STATUS_OK;
}

int8_t ant_core_reset_state() {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  return ant_core_send(&msg, ANT_CORE_TIMEOUT);
}


int8_t ant_core_assign_channel(uint8_t channel, uint8_t channel_type, uint8_t network, uint8_t extended) {
  int8_t status;
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CONF_ASSIGN_CHANNEL;
  msg.data_len=4;
  msg.data[0]=channel;
  msg.data[1]=channel_type;
  msg.data[2]=network;
  msg.data[3]=extended;
  status=ant_core_send_blocking(channel, &msg, ANT_CORE_TIMEOUT);
  if(status==STATUS_OK) {
    ant_core_channels[channel].status=ANT_CHANNEL_STATUS_ASSIGNED;
  }
  return status;
}


int8_t ant_core_unassign_channel(uint8_t channel) {
  int8_t status;
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CONF_UNASSIGN_CHANNEL;
  msg.data_len=1;
  msg.data[0]=channel;
  status=ant_core_send_blocking(channel, &msg, ANT_CORE_TIMEOUT);
  if(status==STATUS_OK) {
    ant_core_channels[channel].status=ANT_CHANNEL_STATUS_UNASSIGNED;
  }
  return status;
}

int8_t ant_core_set_channel_id(uint8_t channel, uint16_t device_no, uint8_t pairing, uint8_t device_type, uint8_t tx_type) {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CONF_CHANNEL_ID;
  msg.data_len=5;
  msg.data[0]=channel;
  msg.data[1]=(uint8_t)(device_no & 0x00ff);
  msg.data[2]=(uint8_t)((device_no & 0xff00) >> 8);
  msg.data[3]=(device_type | (pairing << 7));
  msg.data[4]=tx_type;
  return ant_core_send_blocking(channel, &msg, ANT_CORE_TIMEOUT);
}


int8_t ant_core_set_channel_period(uint8_t channel, uint16_t period) {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CONF_CHANNEL_PERIOD;
  msg.data_len=3;
  msg.data[0]=channel;
  msg.data[1]=(uint8_t)(period & 0x00ff);
  msg.data[2]=(uint8_t)((period & 0xff00) >> 8);
  return ant_core_send_blocking(channel, &msg, ANT_CORE_TIMEOUT);
}

int8_t ant_core_set_channel_search_timeout(uint8_t channel, uint8_t timeout) {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CONF_SEARCH_TIMEOUT;
  msg.data_len=2;
  msg.data[0]=channel;
  msg.data[1]=timeout;
  return ant_core_send_blocking(channel, &msg, ANT_CORE_TIMEOUT);
}

int8_t ant_core_set_channel_rf_frequency(uint8_t channel, uint8_t rf_freq) {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CONF_CHANNEL_RF_FREQUENCY;
  msg.data_len=2;
  msg.data[0]=channel;
  msg.data[1]=rf_freq;
  return ant_core_send_blocking(channel, &msg, ANT_CORE_TIMEOUT);
}

//int8_t ant_core_set_network_key(uint8_t channel, uint8_t *key);

//int8_t ant_core_add_channel_id(uint16_t device_no, uint8_t device_type, uint8_t tx_type, uint8_t index)

//int8_t ant_core_config_list_id(uint8_t channel, uint8_t list_size, uint8_t exclude);

int8_t ant_core_set_channel_tx_power(uint8_t channel, uint8_t power) {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CONF_CHANNEL_TX_POWER;
  msg.data_len=2;
  msg.data[0]=channel;
  msg.data[1]=power;
  return ant_core_send_blocking(channel, &msg, ANT_CORE_TIMEOUT);
}

int8_t ant_core_set_channel_low_prio_search_timeout(uint8_t channel, uint8_t timeout) {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CONF_LOW_PRIO_SEARCH_TIMEOUT;
  msg.data_len=2;
  msg.data[0]=channel;
  msg.data[1]=timeout;
  return ant_core_send_blocking(channel, &msg, ANT_CORE_TIMEOUT);
}

int8_t ant_core_config_frequency_agility(uint8_t channel, uint8_t rf_freq1, uint8_t rf_freq2, uint8_t rf_freq3) {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CONF_FREQUENCY_AGILITY;
  msg.data_len=4;
  msg.data[0]=channel;
  msg.data[1]=rf_freq1;
  msg.data[2]=rf_freq2;
  msg.data[3]=rf_freq3;
  return ant_core_send_blocking(channel, &msg, ANT_CORE_TIMEOUT);
}

int8_t ant_core_set_proximity_search(uint8_t channel, uint8_t threshold) {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CONF_PROXIMITY_SEARCH;
  msg.data_len=2;
  msg.data[0]=channel;
  msg.data[1]=threshold;
  return ant_core_send_blocking(channel, &msg, ANT_CORE_TIMEOUT);
}

int8_t ant_core_set_channel_search_priority(uint8_t channel, uint8_t priority) {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CONF_SEARCH_PRIORITY;
  msg.data_len=2;
  msg.data[0]=channel;
  msg.data[1]=priority;
  return ant_core_send_blocking(channel, &msg, ANT_CORE_TIMEOUT);
}

int8_t ant_core_set_transmit_power(uint8_t power) {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CONF_TRANSMIT_POWER;
  msg.data_len=2;
  msg.data[0]=0;
  msg.data[1]=power;
  return ant_core_send_blocking(0, &msg, ANT_CORE_TIMEOUT);
}

int8_t ant_core_enable_extended_messages(uint8_t enable) {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CONF_ENABLE_EXT_RX_MSG;
  msg.data_len=2;
  msg.data[0]=0;
  msg.data[1]=enable;
  return ant_core_send_blocking(0, &msg, ANT_CORE_TIMEOUT);
}

int8_t ant_core_enable_led(uint8_t enable) {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CONF_ENABLE_LED;
  msg.data_len=2;
  msg.data[0]=0;
  msg.data[1]=enable;
  return ant_core_send_blocking(0, &msg, ANT_CORE_TIMEOUT);
}

int8_t ant_core_enable_crystal() {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CONF_CRYSTAL_ENABLE;
  msg.data_len=1;
  msg.data[0]=0;
  return ant_core_send_blocking(0, &msg, ANT_CORE_TIMEOUT);
}



int8_t ant_core_reset_system() {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CTRL_SYSTEM_RESET;
  msg.data_len=1;
  msg.data[0]=0;
  return ant_core_send(&msg, ANT_CORE_TIMEOUT);
}


int8_t ant_core_open_channel(uint8_t channel, void (*callback)(struct ant_event *)) {
  int8_t status;
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  ant_core_channels[channel].callback=callback;
  msg.msg_id=ANT_CTRL_OPEN_CHANNEL;
  msg.data_len=1;
  msg.data[0]=channel;
  status=ant_core_send_blocking(channel, &msg, ANT_CORE_TIMEOUT);
  if(status==STATUS_OK) {
    ant_core_channels[channel].status=ANT_CHANNEL_STATUS_TRACKING;
  }
  return status;
}

int8_t ant_core_close_channel(uint8_t channel) {
  int8_t status;
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CTRL_CLOSE_CHANNEL;
  msg.data_len=1;
  msg.data[0]=channel;
  ant_core_reset_state();
  status=ant_core_send_blocking(channel, &msg, ANT_CORE_TIMEOUT);
  if(status!=STATUS_OK) {
    if(ant_core_unassign_channel(channel)!=STATUS_OK) {
      ant_core_reset();
    }      
  }
  return status;
}


int8_t ant_core_request_message(uint8_t channel, uint8_t message_id) {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CTRL_REQUEST_MESSAGE;
  msg.data_len=2;
  msg.data[0]=channel;
  msg.data[1]=message_id;
  return ant_core_send_blocking(channel, &msg, ANT_CORE_TIMEOUT);
}

int8_t ant_core_open_rx_scan_mode() {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CTRL_OPEN_RX_SCAN_MODE;
  msg.data_len=1;
  msg.data[0]=0;
  return ant_core_send(&msg, ANT_CORE_TIMEOUT);
}

int8_t ant_core_sleep_message() {
  struct ant_frame msg;
  memset(&msg, 0, sizeof(struct ant_frame));
  msg.msg_id=ANT_CTRL_SLEEP_MESSAGE;
  msg.data_len=1;
  msg.data[0]=0;
  return ant_core_send(&msg, ANT_CORE_TIMEOUT);
}


int8_t ant_core_send_broadcast_data(uint8_t channel, uint8_t *data, bool blocking) {
  struct ant_frame msg;
  struct ant_frame *bc_frame;
  irqflags_t flags;
  
  if(ant_core_channels[channel].status!=ANT_CHANNEL_STATUS_TRACKING) {
    return ERR_IO_ERROR;
  }
  
  if(blocking) {
    
    if(atomSemGet(&ant_core_channels[channel].tx_sem, ANT_CORE_TIMEOUT)!=ATOM_OK) {
      return ERR_TIMEOUT;
    }
  
    if(atomMutexGet(&ant_mutex, ANT_CORE_TIMEOUT)!=ATOM_OK) {
      return ERR_TIMEOUT;
    }
  
    msg.msg_id=ANT_DATA_BROADCAST;
    msg.data_len=9;
    msg.data[0]=channel;
    memcpy(msg.data+1, data, 8);
  
    if(atomQueuePut(&ant_tx_queue, ANT_CORE_TIMEOUT, (uint8_t *)&msg)!=ATOM_OK) {
      atomMutexPut(&ant_mutex);
      return ERR_TIMEOUT;
    }
  
    ant_core_start_tx();
  
    atomMutexPut(&ant_mutex);
    
    return ant_core_channels[channel].event_code;
  }
  else {
    
    bc_frame = &ant_core_channels[channel].broadcast_frame;
    flags=cpu_irq_save();
    bc_frame->msg_id=ANT_DATA_BROADCAST;
    bc_frame->data_len=9;
    bc_frame->data[0]=channel;
    memcpy(bc_frame->data+1, data, 8);
    cpu_irq_restore(flags);
    
    return STATUS_OK;
  }
}

int8_t ant_core_send_acknowledged_data(uint8_t channel, uint8_t *data) {
  uint8_t status;
  struct ant_frame msg;
  
  if(ant_core_channels[channel].status!=ANT_CHANNEL_STATUS_TRACKING) {
    return ERR_IO_ERROR;
  }
  
  if(atomSemGet(&ant_core_channels[channel].tx_sem, ANT_CORE_TIMEOUT)!=ATOM_OK) {
    //atomMutexPut(&ant_mutex);
    return ERR_TIMEOUT; 
  }
  
  atomSemResetCount(&ant_core_channels[channel].rx_sem, 0);
  
  if(atomMutexGet(&ant_mutex, ANT_CORE_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  msg.msg_id=ANT_DATA_ACKNOWLEDGE;
  msg.data_len=9;
  msg.data[0]=channel;
  memcpy(msg.data+1, data, 8);
  
  if(atomQueuePut(&ant_tx_queue, ANT_CORE_TIMEOUT, (uint8_t *)&msg)!=ATOM_OK) {
    atomMutexPut(&ant_mutex);
    return ERR_TIMEOUT;
  }
  
  ant_core_start_tx();
  
  atomMutexPut(&ant_mutex);
  
  if((status=atomSemGet(&ant_core_channels[channel].rx_sem, ANT_CORE_TIMEOUT))!=ATOM_OK) {
    //atomMutexPut(&ant_mutex);
    return ERR_TIMEOUT; 
  }
  
  return ant_core_channels[channel].event_code;
}


int8_t ant_core_send_burst_transfer(uint8_t channel, uint16_t *shared_address, uint8_t *head, uint8_t *burst, uint16_t data_len) {
  struct ant_frame msg;
  uint8_t no_of_pgks, last_data_len;
  uint16_t j;
  uint8_t *ptr;
  uint8_t block_len, offset;
  
  msg.msg_id=ANT_DATA_BURST_TRANSFER;
  msg.data_len=9;
  msg.data[0]=channel;
  if(shared_address!=NULL) {
    block_len=6;
    offset=3;
    msg.data[1]=(uint8_t)  *shared_address;
    msg.data[2]=(uint8_t) ((*shared_address) >> 8);
  }
  else {
    block_len=8;
    offset=1;
  }  
  
  if(ant_core_channels[channel].status!=ANT_CHANNEL_STATUS_TRACKING) {
    return ERR_IO_ERROR;
  }
  
  if(atomSemGet(&ant_core_channels[channel].tx_sem, ANT_CORE_TIMEOUT)!=ATOM_OK) {
    //atomMutexPut(&ant_mutex);
    return ERR_TIMEOUT; 
  }
  
  atomSemResetCount(&ant_core_channels[channel].rx_sem, 0);
  
  if(atomMutexGet(&ant_mutex, ANT_CORE_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  //first packet
  memcpy(msg.data+offset, head, block_len);
  
  if(atomQueuePut(&ant_tx_queue, ANT_CORE_TIMEOUT, (uint8_t *)&msg)!=ATOM_OK) {
    atomMutexPut(&ant_mutex);
    return ERR_TIMEOUT;
  }
  
  ant_core_start_tx();
  
  //burst the rest of the packages
  no_of_pgks=data_len / block_len;
  last_data_len=data_len % block_len;
  if(last_data_len>0) {
    no_of_pgks++;
  }
  else {
    last_data_len=block_len;  
  }
  ptr=burst;
  for(j=0;j<no_of_pgks; j++) {
    msg.data[0]=(channel | (((j % 3)+1) << 5));
    if(j==no_of_pgks-1) {
      msg.data[0]|=0x80;
      memcpy(msg.data+offset, ptr, last_data_len);
    }
    else {
      memcpy(msg.data+offset, ptr, block_len);
      ptr+=block_len;
    }
    
    if(atomQueuePut(&ant_tx_queue, ANT_CORE_TIMEOUT, (uint8_t *)&msg)!=ATOM_OK) {
      atomMutexPut(&ant_mutex);
      return ERR_TIMEOUT;
    }
    
    ant_core_start_tx();
  }
  
  atomMutexPut(&ant_mutex);
  
  if(atomSemGet(&ant_core_channels[channel].rx_sem, ANT_CORE_TIMEOUT)!=ATOM_OK) {
    //atomMutexPut(&ant_mutex);
    return ERR_TIMEOUT;
  }
  
  return ant_core_channels[channel].event_code;
}




int8_t ant_core_set_shared_address_msg(uint8_t channel, uint16_t shared_address) {
  struct ant_frame *msg;
  irqflags_t flags;
  
  if(ant_core_channels[channel].status!=ANT_CHANNEL_STATUS_TRACKING) {
    return ERR_IO_ERROR;
  }
  
  if(atomMutexGet(&ant_mutex, ANT_CORE_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  msg = &ant_core_channels[channel].broadcast_frame;
  flags=cpu_irq_save();
  msg->msg_id=ANT_DATA_BROADCAST;
  msg->data_len=9;
  msg->data[0]=channel;
  msg->data[1]=(uint8_t)  shared_address;
  msg->data[2]=(uint8_t) (shared_address >> 8);
  memset(msg->data+3, 0, 6);
  cpu_irq_restore(flags);
  
  if(atomQueuePut(&ant_tx_queue, ANT_CORE_TIMEOUT, (uint8_t *)msg)!=ATOM_OK) {
    atomMutexPut(&ant_mutex);
    return ERR_TIMEOUT;
  }
  
  ant_core_start_tx();
  
  atomMutexPut(&ant_mutex);
  
  return STATUS_OK;
}


int8_t ant_core_init_cw_test_mode() {
  struct ant_frame msg;
  msg.msg_id=ANT_TEST_CW_INIT;
  msg.data_len=1;
  msg.data[0]=0;
  return ant_core_send(&msg, ANT_CORE_TIMEOUT);
}

int8_t ant_core_set_cw_test_mode(uint8_t tx_power, uint8_t rf_freq) {
  struct ant_frame msg;
  msg.msg_id=ANT_TEST_CW_TEST;
  msg.data_len=3;
  msg.data[0]=0;
  msg.data[1]=tx_power;
  msg.data[2]=rf_freq;
  return ant_core_send(&msg, ANT_CORE_TIMEOUT);
}

void ant_core_cleanup_channels() {
  uint8_t channel, status;
  
  //check if any channel has closed without reporting it
  for(channel=0;channel<ANT_NO_OF_CHANNELS;channel++) {
    status=ant_core_channels[channel].status;
    
    if(ant_core_request_message(channel, ANT_REQUEST_CHANNEL_STATUS)==STATUS_OK) {
      if(status!=ant_core_channels[channel].status) {
        switch(ant_core_channels[channel].status) {
          case ANT_CHANNEL_STATUS_SEARCHING:
          case ANT_CHANNEL_STATUS_TRACKING:
            if(status!=ANT_CHANNEL_STATUS_TRACKING 
                || status!=ANT_CHANNEL_STATUS_SEARCHING) {
              ant_core_close_channel(channel);
            }
            break;
          case ANT_CHANNEL_STATUS_ASSIGNED:
            ant_core_unassign_channel(channel);
            break;
          case ANT_CHANNEL_STATUS_UNASSIGNED:
          default:
            break;
        }
      }
    }
  }
}


uint8_t ant_core_channel_is_open(uint8_t channel) {
  uint8_t open;
  
  if(!ant_core_request_message(channel, ANT_REQUEST_CHANNEL_STATUS)) {
    return FALSE;
  }
  
  if(ant_core_channels[channel].status==ANT_CHANNEL_STATUS_SEARCHING
      || ant_core_channels[channel].status==ANT_CHANNEL_STATUS_TRACKING) {
    open=TRUE;
  }
  else {
    open=FALSE;
  }
  return open;
}

