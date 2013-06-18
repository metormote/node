/*
 * node.c
 *
 * Created: 10/4/2011 11:24:19 AM
 *  Author: Administrator
 */ 

#include "node.h"


/* Local data */
#define STATE_EEPROM_PAGE_ADDRESS   0

/* Idle thread's stack area */
static uint8_t *idle_thread_stack=(uint8_t *)RAMEND;

/* ANT thread's stack area */
static uint8_t *ant_thread_stack=(uint8_t *)( RAMEND-IDLE_THREAD_STACK_SIZE_BYTES);

/* BLE thread's stack area */
static uint8_t *ble_thread_stack=(uint8_t *)( RAMEND-IDLE_THREAD_STACK_SIZE_BYTES
                                              -ANT_THREAD_STACK_SIZE_BYTES);
/* Node thread's stack area */
static uint8_t *node_thread_stack=(uint8_t *)( RAMEND-IDLE_THREAD_STACK_SIZE_BYTES
                                              -ANT_THREAD_STACK_SIZE_BYTES
                                              -BLE_THREAD_STACK_SIZE_BYTES);


/* Application threads' TCBs */
static ATOM_TCB ant_tcb;

static ATOM_TCB ble_tcb;

static ATOM_TCB node_tcb;

static ATOM_SEM sleep_sem;
static ATOM_SEM node_thread_sem;

struct nvm_device_serial serial;

/* Forward declarations */
static void init(void);
static void sleep_aquire(void);
static void sleep_release(void);
static void sleep(void);
static void timer_callback(uint32_t time);
static void ant_thread_func (uint32_t data);
static void ant_callback(ant_event_t event, uint8_t *data, uint16_t len);
static void ble_thread_func (uint32_t data);
static void ble_callback(ble_events_t event, uint8_t *data, uint16_t len);
static void node_thread_func (uint32_t data);
static void idle_thread_callback (void);
static void ui_listener(uint8_t event);
static void power_listener(enum POWER_EVENT event);
static int8_t process_message(uint64_t msg_code, pb_istream_t *msg_stream);
static int8_t process_node_msg(pb_istream_t *msg_stream, pb_ostream_t *resp_stream);
static int8_t update_firmware(pb_istream_t *msg_stream);
static int8_t process_callback_data(uint8_t *data, uint16_t len);
static int8_t send(pb_bytes_array_t *env_bytes, uint64_t msg_code, uint8_t *msg_buf, uint16_t len);
static int8_t config_snapshot(pb_ostream_t *msg_output_stream, void *arg);
static void read_state(void);
static void save_state(void);

static void (*app_init_callback)(void);
static void (*app_callback)(void);
static int8_t (*msg_callback)(uint64_t msg_code, pb_istream_t *msg_stream, uint64_t *resp_msg_code, pb_ostream_t *resp_stream);


static uint32_t keydown;

static struct node_signature_t signature = {
  .device_id=0x0,
  .key={0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F},
  .temp_offset=0
};

static struct node_state_t state = {
  .type=NODE_TYPE_HUB,
  .transfer_interval=10,
  .polling_interval=10,
  .timeout=60,
  .no_of_polls=8,
  .rx_nonce=0,
  .tx_nonce=0
};

static void timer_callback(uint32_t time) {
  atomIntEnter();
  if(state.type==NODE_TYPE_SLAVE) {
    sleep_aquire();
    if(ant_slave.last_rx + (uint32_t)state.timeout <= rtc_get_time()) {
      ant_reset();
    }
    rtc_set_alarm_relative((uint32_t)state.timeout);
    sleep_release();
  }    
  atomIntExit(FALSE, TRUE);
}

static void sleep_aquire() {
  if(state.type!=NODE_TYPE_SLAVE)
    return;
  atomSemPut(&sleep_sem);
}

static void sleep_release() {
  if(state.type!=NODE_TYPE_SLAVE)
    return;
  atomSemGet(&sleep_sem, -1);
}

static void sleep() {
  if(state.type!=NODE_TYPE_SLAVE)
    return;
  
  ui_led_off(UI_GREEN_LED);
  if(atomSemGet(&sleep_sem, -1)==ATOM_WOULDBLOCK) {
    cpu_irq_disable();
    ant_standby();
    ble_standby();
    sleep_set_mode(SLEEP_MODE_PWR_SAVE);
    sleep_enable();
    cpu_irq_enable();
    _delay_ms(1);
    sleep_enter();
    cpu_irq_disable();
    sleep_disable();
    ant_resume();
    ble_resume();
    cpu_irq_enable();
  }
  else {
    atomSemPut(&sleep_sem);
  }
  ui_led_on(UI_GREEN_LED);
}

static void init() {
    //initialize interrupt controller
    pmic_init();
    
    //init io ports
    ioport_init();
    
    //initialize board
    board_init();
    
    //initialize system clock
    sysclk_init();
    //ccp_write_io((uint8_t *)&OSC.XOSCFAIL, 0x01);
    
    /* Enable the AES clock. */
    sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_AES);
    
    //initialize sleep manager
    sleepmgr_init();
    //never sleep deeper than extended standby
    sleepmgr_lock_mode(SLEEPMGR_ESTDBY);
  
    //initialize real time counter
    //osc_enable(OSC_ID_RC32KHZ);
    rtc_init();
    rtc_set_callback(timer_callback);
    
    nvm_user_sig_read_buffer(0, &signature, sizeof(struct node_signature_t));
    
    //seed random generator
    seed();
    
    read_state();
    
    //init the crypto module
    crypto_init(signature.key);
    
    //init memory allocation mutex
    mem_init();
    
    //init the leds and button
    ui_init();
    ui_set_ui_listener(ui_listener);
    
    //init power
    power_init();
    power_good_init();
    power_set_pg_listener(power_listener);
    
    //create sleep semaphore
    atomSemCreate(&sleep_sem, 0);
  
    //create node thread semaphore
    atomSemCreate(&node_thread_sem, 0);
  
    if(power_is_good()) state.type=NODE_TYPE_HUB;
    else state.type=NODE_TYPE_SLAVE;
    
    if(state.type==NODE_TYPE_SLAVE) {
      rtc_set_alarm_relative(state.timeout);
    }
    
    if(app_init_callback!=NULL) app_init_callback();
}


int node_start (void (*init_callback)(void), void (*app_entry_point)(void), int8_t (*callback)(uint64_t, pb_istream_t *, uint64_t *, pb_ostream_t *))
{
  int8_t status, ant_status, node_status, ble_status;
  uint16_t sp;
  
  app_init_callback = init_callback;
  msg_callback = callback;
  app_callback = app_entry_point;
  
 /**
  * Set stack pointer to a temporary stack position required
  * during this startup function. Note that all the stacks must reside
  * in the top of the memory to make malloc() work. malloc() will not
  * allocate any memory above the current stack pointer.
  */
  sp=RAMEND-IDLE_THREAD_STACK_SIZE_BYTES
    -ANT_THREAD_STACK_SIZE_BYTES
    -BLE_THREAD_STACK_SIZE_BYTES
    -NODE_THREAD_STACK_SIZE_BYTES;
  
  //we assign the stack pointer in two steps because of
  //http://savannah.nongnu.org/bugs/?25778
  SPH=(uint8_t)(sp>>8);
  SPL=(uint8_t)sp;
  
  //initialize all modules
  init();
  
  
   /**
    * Note: to protect OS structures and data during initialization,
    * interrupts must remain disabled until the first thread
    * has been restored. They are re-enabled at the very end of
    * the first thread restore, at which point it is safe for a
    * reschedule to take place.
    */

   /**
    * Initialize the OS before creating our threads.
    *
    */
  status = atomOSInit(idle_thread_stack, 
            IDLE_THREAD_STACK_SIZE_BYTES, idle_thread_callback);
    
  if (status == ATOM_OK)
  {
      /* Enable the system tick timer */
      avrInitSystemTickTimer();
      
      /* Create ANT thread */ 
      ant_status = atomThreadCreate(&ant_tcb,
                    ANT_THREAD_PRIO, ant_thread_func, 0,
                    ant_thread_stack,
                    ANT_THREAD_STACK_SIZE_BYTES);
         
      //ant_status = 0;
    
      /* Create BLE thread */
      ble_status = atomThreadCreate(&ble_tcb,
                    BLE_THREAD_PRIO, ble_thread_func, 0,
                    ble_thread_stack,
                    BLE_THREAD_STACK_SIZE_BYTES);
      
      //ble_status = 0;
      
      /* Create node thread */
      node_status = atomThreadCreate(&node_tcb,
                    NODE_THREAD_PRIO, node_thread_func, 0,
                    node_thread_stack,
                    NODE_THREAD_STACK_SIZE_BYTES);
      
      //node_status = 0;
      
      if (ant_status == ATOM_OK && node_status==ATOM_OK && ble_status==ATOM_OK)
      {
         /**
          * Application threads successfully created. It is
          * now possible to start the OS. Execution will not return
          * from atomOSStart(), which will restore the context of
          * our application thread and start executing it.
          * 
          * Note that interrupts are still disabled at this point.
          * They will be enabled as we restore and execute our first
          * thread in archFirstThreadRestore().
          */
          atomOSStart();
      }
  }
    
  for (;;);

  /* There was an error starting the OS if we reach here */
  return (0);
    
}


/**
 * \b ant_idle_thread_callback
 *
 * Callback for the idle thread.
 *
 * @return None
 */
static void idle_thread_callback (void)
{
    for(;;) {
      sleepmgr_enter_sleep();
    }
}


int8_t node_send_message(uint64_t msg_code, int8_t (*callback)(pb_ostream_t *stream, void *arg), void *arg)
{
    int8_t status=STATUS_OK;
    pb_bytes_array_t *env_bytes;
    uint8_t *msg_buf;
    pb_ostream_t msg_output_stream;
  
    if(ant_get_network_id()==ANT_NO_NETWORK && !ble_is_connected())
      return ERR_IO_ERROR;
    
    //allocate memory for the envelope
    env_bytes=(pb_bytes_array_t *)mem_safe_malloc(ENV_BUF_SIZE);
    if(env_bytes==NULL) {
      return ERR_NO_MEMORY;
    }
    msg_buf=(uint8_t *)env_bytes+(ENV_BUF_SIZE-MSG_BUF_SIZE);
    msg_output_stream=pb_ostream_from_buffer(msg_buf, MSG_BUF_SIZE);
  
    callback(&msg_output_stream, arg);
    
    status=send(env_bytes, msg_code, msg_buf, msg_output_stream.bytes_written);
    
    mem_safe_free(env_bytes);
    
    return status;
}    


int8_t node_snapshot(pb_ostream_t *msg_output_stream, void *arg) {
  node_Out msg;
  
  memset(&msg, 0, sizeof(node_Out));
  
  if(power_read_temperature(&msg.temperature)==STATUS_OK) {
    msg.temperature+=signature.temp_offset;
    msg.has_temperature=true;
  }
  
  if(power_read_batt_lvl(&msg.batteryLevel)==STATUS_OK) {
    msg.has_batteryLevel=true;
  }
  
  //serialize the message
  if (!pb_encode(msg_output_stream, node_Out_fields, &msg))
  return ERR_IO_ERROR;
  
  return STATUS_OK;
}


static int8_t config_snapshot(pb_ostream_t *msg_output_stream, void *arg) {
  node_In msg;
  uint8_t leds;
  
  memset(&msg, 0, sizeof(node_In));
  
  msg.has_leds=TRUE;
  leds=0;
  if(ui_is_led_on(UI_GREEN_LED))
    leds |= 0x1;
  if(ui_is_led_on(UI_RED_LED))
    leds |= 0x2;
  if(ui_is_led_on(UI_YELLOW_LED))
    leds |= 0x4;
  if(ui_is_led_on(UI_BLUE_LED))
    leds |= 0x8;
    
  msg.leds=leds;
  
  msg.has_noOfPolls=TRUE;
  msg.noOfPolls=state.no_of_polls;
  
  msg.has_transferInterval=TRUE;
  msg.transferInterval=state.transfer_interval;
  
  msg.has_pollingInterval=TRUE;
  msg.pollingInterval=state.polling_interval;
  
  msg.has_timeout=TRUE;
  msg.timeout=state.timeout;
  
  //serialize the message
  if (!pb_encode(msg_output_stream, node_In_fields, &msg))
  return ERR_IO_ERROR;
  
  return STATUS_OK;
}



/**
 * \b ant_thread_func
 *
 * Entry point for ANT application thread.
 *
 * @param[in] data Unused (optional thread entry parameter)
 *
 * @return None
 */
static void ant_thread_func (uint32_t data)
{ 
  // initialize rf circuits...
  ant_init(state.type==NODE_TYPE_HUB ? ANT_DEVICE_TYPE_NODE : ANT_DEVICE_TYPE_SLAVE, (uint32_t)signature.device_id, (uint32_t)state.network_id);
      
  for(;;) {
      ant_start(ant_callback);
  }    
}    

//data from ant layer addressed to this node 
static void ant_callback(ant_event_t evt, uint8_t *data, uint16_t len) {
  
  switch(evt) {
    case ANT_EVENT_START:
      atomSemResetCount(&sleep_sem, 0);
      break;
    case ANT_EVENT_SCAN_START:
      ant_slave.last_rx=rtc_get_time();
      rtc_set_alarm_relative(ANT_CONNECT_TIMEOUT);
      sleep_aquire();
      ui_led_blink(UI_GREEN_LED, 10, 10, 0xFFFF);
      break;
    case ANT_EVENT_CONNECTED:
      rtc_set_alarm_relative(state.timeout);
      state.network_id=ant_get_network_id();
      save_state();
      ui_led_blink_off(UI_GREEN_LED);
      sleep_release();
      break;
    case ANT_EVENT_POLL_BEGIN:
      rtc_set_alarm_relative(state.timeout);
      atomSemPut(&node_thread_sem);
      break;
    case ANT_EVENT_POLL_END:
      sleep();
      break;
    case ANT_EVENT_MASTER_DATA:
    case ANT_EVENT_NETWORK_DATA:
    case ANT_EVENT_SLAVE_DATA:
      process_callback_data(data, len);
      break;
    default:
      sleep();
      break;
  }
}


/**
 * \b ble_thread_func
 *
 * Entry point for BLE application thread.
 *
 * @param[in] data Unused (optional thread entry parameter)
 *
 * @return None
 */
static void ble_thread_func (uint32_t data) {
  sleep_aquire();
  //init after ant setup is finished
  atomTimerDelay(SYSTEM_TICKS_PER_SEC);
  
  ble_init();
  
  sleep_release();
  
  for(;;) {
    //ble_set_device_id(signature.device_id);
    ble_start(ble_callback);
    
    sleep_aquire();
    
    ble_reset();
    
    sleep_release();
  }
}


static void ble_callback(ble_events_t event, uint8_t *data, uint16_t len) {
  
  switch(event) {
    case BLE_EVENT_CONNECTING:
      sleep_aquire();
      break;
    case BLE_EVENT_CONNECTED:
      ui_led_on(UI_BLUE_LED);
      break;
    case BLE_EVENT_DISCONNECT:
      ui_led_off(UI_BLUE_LED);
      sleep_release();
      break;
    case BLE_EVENT_DATA:
      process_callback_data(data, len);
      break;
  }
}  


/**
 * \b node_thread_func
 *
 * Entry point for node application thread.
 *
 * @param[in] data Unused (optional thread entry parameter)
 *
 * @return None
 */
static void node_thread_func (uint32_t data)
{ 
  if(state.type==NODE_TYPE_SLAVE) {
      atomSemGet(&node_thread_sem, 0);
  }
  
  for(;;) {
    
    app_callback();
              
    if(state.type==NODE_TYPE_SLAVE) {
      ant_set_slave_control(state.polling_interval, state.timeout, state.no_of_polls);
      
      atomSemGet(&node_thread_sem, 0);
    }
    else {
      atomSemGet(&node_thread_sem, state.transfer_interval*SYSTEM_TICKS_PER_SEC);
    }
  }
}



static int8_t process_callback_data(uint8_t *data, uint16_t len) {
  uint8_t hmac[SHA1_BYTES];
  uint8_t *key_buffer;
  iop_Envelope env;
  pb_istream_t istream;
  pb_istream_t msg_stream;
  io_input_stream_t src;
  
  sleep_aquire();
  
  istream=pb_istream_from_buffer(data, len);
  msg_stream=pb_istream_from_buffer(NULL, 0);
  
  //decode envelope
  if (!decode_envelope(&env, &istream, &msg_stream)) {
    sleep_release();
    return ERR_BAD_FORMAT;
  }    
  
  //check if the message is for this node
  if(env.dst==signature.device_id) {
    src=io_input_stream_from_buffer((uint8_t *)msg_stream.state, msg_stream.bytes_left);
    key_buffer=(uint8_t *)mem_safe_malloc(SHA1_BLOCK_BYTES);
    if(key_buffer==NULL) {
      return ERR_NO_MEMORY;
    }
    calc_hash(hmac, key_buffer, signature.key, &env, &src);
    mem_safe_free(key_buffer);
    
    //if hash do not match, discard message
    if(memcmp(hmac, env.hash.bytes, SHA1_BYTES)!=0) {
      return ERR_BAD_DATA;
    }
  
    //check that the received nonce is greater than the last received
    if(state.rx_nonce>env.nonce) {
      //return ERR_BAD_DATA;
    }
    
    state.rx_nonce=env.nonce;
    save_state();
    len=(uint16_t)msg_stream.bytes_left;
    
    crypto_start_decrypt();
    crypto_decrypt(msg_stream.state, env.iv.bytes, &len);
    crypto_final();
    
    msg_stream.bytes_left=len;
    
    if(env.msgCode==FIRMWARE_UPGRADE_MSG_CODE) {
      update_firmware(&msg_stream);
    }
    else {
      process_message(env.msgCode, &msg_stream);
    }
  }
  else {
    //deliver the message to the recipient
    ant_send_to_device(env.dst, data, len);
  }
  
  if(state.type==NODE_TYPE_SLAVE) {
    ant_set_slave_control(state.polling_interval, state.timeout, state.no_of_polls);
    sleep_release();
  }
  else {
    ui_led_blink(UI_GREEN_LED, 2, 4, 8);
  }
  return STATUS_OK;
}


static int8_t process_message(uint64_t msg_code, pb_istream_t *msg_stream)
{
  int8_t status=STATUS_OK;
  uint8_t *msg_buf;
  uint64_t resp_msg_code;
  pb_bytes_array_t *env_bytes;
  pb_ostream_t msg_output_stream;
  
  //allocate memory for the envelope
  env_bytes=(pb_bytes_array_t *)mem_safe_malloc(ENV_BUF_SIZE);
  if(env_bytes==NULL) {
    return ERR_NO_MEMORY;
  }
  
  msg_buf=(uint8_t *)env_bytes+(ENV_BUF_SIZE-MSG_BUF_SIZE);
  
  msg_output_stream=pb_ostream_from_buffer(msg_buf, MSG_BUF_SIZE);
  
  switch(msg_code) {
    case NODE_IN_MSG_CODE:
      resp_msg_code=NODE_IN_MSG_CODE;
      status=process_node_msg(msg_stream, &msg_output_stream);
      break;
    default:
      msg_callback(msg_code, msg_stream, &resp_msg_code, &msg_output_stream);
      break;
  }
  
  if(msg_output_stream.bytes_written > 0) {
    status=send(env_bytes, resp_msg_code, msg_buf, msg_output_stream.bytes_written);
  }
        
  mem_safe_free(env_bytes);
  
  return status;
}


static int8_t send(pb_bytes_array_t *env_bytes, uint64_t msg_code, uint8_t *msg_buf, uint16_t len) {
  int8_t status=STATUS_OK;
  uint8_t *tmp;
  iop_Envelope env;
  pb_ostream_t output_stream;
  pb_istream_t msg_input_stream;
  io_input_stream_t istream;
  
  memset((void *)&env, 0, sizeof(iop_Envelope));
  
  env.msgCode=msg_code;
  env.has_dst=false;
  env.has_src=true;
  env.src=signature.device_id;
  env.has_iv=true;
  env.iv.size=AES_KEY_SIZE;
  env.has_hash=true;
  env.hash.size=SHA1_BYTES;
  env.has_nonce=true;
  env.nonce = ++state.tx_nonce;
  if((state.tx_nonce % 100)==0) {
    save_state();
  }
  
  crypto_init_iv(env.iv.bytes);
  crypto_start_encrypt(env.iv.bytes);
  crypto_encrypt(msg_buf, &len);
  crypto_final();
  
  //calculate hash
  tmp=(uint8_t *)env_bytes;
  istream=io_input_stream_from_buffer(msg_buf, (uint32_t)len);
  calc_hash(env.hash.bytes, tmp, signature.key, &env, &istream);
  
  output_stream=pb_ostream_from_buffer(env_bytes->bytes, ENV_BUF_SIZE-sizeof(size_t));
  msg_input_stream=pb_istream_from_buffer(msg_buf, len);
  encode_envelope(&output_stream, &env, &msg_input_stream);
  env_bytes->size=output_stream.bytes_written;
  
  if(ant_get_network_id()!=ANT_NO_NETWORK) {
    status=ant_send_to_base(env_bytes->bytes, env_bytes->size);
  }
  
  ble_send(env_bytes->bytes, env_bytes->size);
  
  return status;
}


static int8_t process_node_msg(pb_istream_t *msg_stream, pb_ostream_t *resp_stream) {
  int8_t status=STATUS_OK;
  uint8_t i;
  node_In msg;
  
  //decode message
  if (pb_decode(msg_stream, node_In_fields, &msg)) {
      
    if(msg.has_transferInterval && msg.transferInterval>0) {
      state.transfer_interval=msg.transferInterval;
    }
    if(msg.has_timeout && msg.timeout>0) {
      state.timeout=(uint16_t)msg.timeout;
      rtc_set_alarm_relative((uint32_t)state.timeout);
    }
    if(msg.has_pollingInterval && msg.pollingInterval>0) {
      state.polling_interval=(uint16_t)msg.pollingInterval;
    }
    if(msg.has_noOfPolls && msg.noOfPolls>0) {
      state.no_of_polls=(uint8_t)msg.noOfPolls;
    }
    if(state.timeout<4*state.polling_interval) {
      state.timeout=4*state.polling_interval;
    }
    
    if(msg.has_leds) {
      for(i=0;i<LED_COUNT;i++) {
        if(msg.leds & (1 << i)) {
          ui_led_on(i);
        }
        else {
          ui_led_off(i);
        }
      }
    }
    
    status=config_snapshot(resp_stream, NULL);
    
    if(state.type==NODE_TYPE_HUB) {
      atomSemPut(&node_thread_sem);
    }
  }
  
  return status;
}


static int8_t update_firmware(pb_istream_t *msg_stream) {
  uint8_t *buffer;
  uint16_t crc;
  irqflags_t flags;
  firmware_Update msg;
  
  //decode message
  if (!pb_decode(msg_stream, firmware_Update_fields, &msg)) {
    return ERR_BAD_FORMAT;
  }
  
  crc=(uint16_t)msg.crc;
  
  //allocate memory for the envelope
  buffer=(uint8_t *)mem_safe_malloc(SPM_PAGESIZE);
  if(buffer==NULL) {
    return ERR_NO_MEMORY;
  }
  
  for (uint16_t i = 0; i < SPM_PAGESIZE; i++)
  {
    buffer[i] = pgm_read_byte_far(XB_APP_TEMP_START + XB_APP_TEMP_SIZE - SPM_PAGESIZE + i);
  }
    
  buffer[SPM_PAGESIZE-6] = 'X';
  buffer[SPM_PAGESIZE-5] = 'B';
  buffer[SPM_PAGESIZE-4] = 'I';
  buffer[SPM_PAGESIZE-3] = 'A';
  buffer[SPM_PAGESIZE-2] = (crc >> 8) & 0xff;
  buffer[SPM_PAGESIZE-1] = crc & 0xff;
  
  flags=cpu_irq_save();
  _delay_ms(100);
  if (xboot_app_temp_write_page(XB_APP_TEMP_SIZE - SPM_PAGESIZE, buffer, 1) == XB_SUCCESS) {
    _delay_ms(100);
    xboot_reset();
  }
  cpu_irq_restore(flags);
  
  mem_safe_free(buffer);
  
  return ERR_BAD_ADDRESS;
}

static void power_listener(enum POWER_EVENT event) {
  switch(event) {
    case POWER_GOOD_ON:
      ant_reset();
      break;
    case POWER_GOOD_OFF:
    default:
      break;
  };
}


static void ui_listener(uint8_t event) {
  switch(event) {
    case UI_EVENT_BUTTON_DOWN:
      ui_led_on(UI_GREEN_LED);
      keydown=rtc_get_time();
      break;
    case UI_EVENT_BUTTON_UP:
      if((rtc_get_time()-keydown)>2) {
        ui_led_on(UI_RED_LED);
        state.network_id=ANT_NO_NETWORK;
        save_state();
        _delay_ms(100);
        reset_do_soft_reset();
        for(;;);
      }
      
      ui_led_off(UI_GREEN_LED);
      
      break;
  }
}

static void read_state() {
  uint8_t *p;
  
  p=(uint8_t *)&state;
  nvm_eeprom_read_buffer(STATE_EEPROM_PAGE_ADDRESS*EEPROM_PAGE_SIZE, p, EEPROM_PAGE_SIZE);
  nvm_eeprom_read_buffer((STATE_EEPROM_PAGE_ADDRESS+1)*EEPROM_PAGE_SIZE, p+EEPROM_PAGE_SIZE, sizeof(struct node_state_t)-EEPROM_PAGE_SIZE);
  if(state.tx_nonce==0xFFFFFFFFFFFFFFFF) {
    state.type=NODE_TYPE_HUB;
    state.network_id=0;
    state.transfer_interval=10;
    state.polling_interval=10;
    state.timeout=60;
    state.no_of_polls=8;
    state.rx_nonce=0;
    state.tx_nonce=0;
    save_state();
  }
  else {
    state.rx_nonce+=100;
    state.tx_nonce+=100;
  }
}

static void save_state() {
  uint8_t i;
  uint8_t *p;
  
  p=(uint8_t *)&state;
  nvm_eeprom_flush_buffer();
  for(i=0;i<EEPROM_PAGE_SIZE;i++) {
    nvm_eeprom_load_byte_to_buffer(i, *p++);
  }
  nvm_eeprom_atomic_write_page(STATE_EEPROM_PAGE_ADDRESS);
  
  nvm_eeprom_flush_buffer();
  for(i=0;i<sizeof(struct node_state_t)-EEPROM_PAGE_SIZE;i++) {
    nvm_eeprom_load_byte_to_buffer(i, *p++);
  }
  nvm_eeprom_atomic_write_page(STATE_EEPROM_PAGE_ADDRESS+1);
}
