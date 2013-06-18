/*
 * ble_core.c
 *
 * Created: 10/1/2011 5:44:00 PM
 *  Author: Administrator
 */ 
#include "ble_core.h"

struct spi_device BLE_DEVICE = {
  .id = BLE_REQN
};

struct ble_state_t ble_state;
struct ble_pipe_state_t ble_pipe_state[BLE_MAX_NO_OF_PIPES];

ATOM_QUEUE ble_rx_queue;
static uint8_t ble_rx_queue_data[BLE_CORE_RX_QUEUE_SIZE*sizeof(struct ble_aci_packet_t)];

static ATOM_MUTEX ble_mutex;
static ATOM_SEM ble_command_sem;
static ATOM_SEM ble_mode_sem;
static ATOM_SEM ble_credit_sem;
static ATOM_SEM ble_connect_sem;
static ATOM_SEM ble_timing_sem;
static ATOM_SEM ble_tx_sem;

static volatile struct ble_aci_packet_t current_rx_packet;
static volatile struct ble_aci_packet_t current_tx_packet;
static volatile uint8_t *rx_packet_ptr;
static volatile uint8_t *tx_packet_ptr;
static volatile uint8_t rx_packet_bytes_left;
static volatile uint8_t tx_packet_bytes_left;
static volatile uint8_t packet_pos;
static volatile uint8_t max_credits;

static int8_t transmit(int32_t timeout);
static uint8_t ble_spi_interrupt_handler(void);
static void process_rx_packet(void);
static int8_t send_system_command(enum BLE_SYSTEM_COMMAND cmd, uint8_t *pdu, uint8_t len);
static int8_t wait_for_mode(enum BLE_OPERATING_MODE mode);



/* Interrupt handler for rx events */
ISR(BLE_RDYN_INT_VECTOR) {
  atomIntEnter();
  spi_select_device(BLE_SPI_MODULE, &BLE_DEVICE);
  packet_pos=0;
  ble_spi_interrupt_handler();
  atomIntExit(FALSE, FALSE);
}

ISR(BLE_SPI_INT_VECTOR) {
  atomIntEnter();
  atomIntExit(FALSE, ble_spi_interrupt_handler());
}

static inline int8_t transmit(int32_t timeout) {
  int8_t status=STATUS_OK;
  spi_select_device(BLE_SPI_MODULE, &BLE_DEVICE);
  if(atomSemGet(&ble_tx_sem, timeout)!=ATOM_OK) {
    status=ERR_TIMEOUT;
  }
  spi_deselect_device(BLE_SPI_MODULE, &BLE_DEVICE);
  return status;
}

static uint8_t ble_spi_interrupt_handler() {
  if(ioport_get_pin_level(BLE_RDYN)) {
    spi_deselect_device(BLE_SPI_MODULE, &BLE_DEVICE);
    return FALSE;
  }
  
  switch(packet_pos) {
    case 0:
      rx_packet_ptr=(uint8_t *)&current_rx_packet;
      tx_packet_ptr=(uint8_t *)&current_tx_packet;
      tx_packet_bytes_left=current_tx_packet.length+1;
      break;
    case 1:
      //debug byte
      break;
    case 2:
      spi_read_single(BLE_SPI,(uint8_t *)rx_packet_ptr++);
      rx_packet_bytes_left=current_rx_packet.length;
      break;
    default:
      spi_read_single(BLE_SPI,(uint8_t *)rx_packet_ptr++);
      break;
  }
  
  if(tx_packet_bytes_left>0) {
    spi_write_single(BLE_SPI,*tx_packet_ptr++);
    tx_packet_bytes_left--;
  }
  else if(rx_packet_bytes_left>0) {
    spi_write_single(BLE_SPI, 0);
    rx_packet_bytes_left--;
  }
  else if(packet_pos<2) {
    spi_write_single(BLE_SPI, 0);
  }
  else {
    spi_deselect_device(BLE_SPI_MODULE, &BLE_DEVICE);
    process_rx_packet();
    packet_pos=0;
    current_rx_packet.length=0;
    current_tx_packet.length=0;
    atomSemPut(&ble_tx_sem);
    atomSemGet(&ble_tx_sem, -1);
    return TRUE;
  }
  
  packet_pos++;
  return FALSE;
}

static void process_rx_packet() {
  uint8_t i;
  
  switch(current_rx_packet.opcode) {
    case BLE_DATAEVT_DATA_CREDIT:
      for(i=0;i<current_rx_packet.pdu[0];i++) {
        atomSemPut(&ble_credit_sem);
      }    
      break;
    case BLE_DATAEVT_PIPE_ERROR:
      i=current_rx_packet.pdu[0]-1;
      ble_pipe_state[i].status=current_rx_packet.pdu[1];
      atomSemPut(&(ble_pipe_state[i].status_sem));
      atomSemGet(&(ble_pipe_state[i].status_sem), -1);
      atomSemPut(&(ble_pipe_state[i].ack_sem));
      atomSemGet(&(ble_pipe_state[i].ack_sem), -1);
      atomSemPut(&ble_credit_sem);
      break;
    case BLE_DATAEVT_DATA_RECEIVED:
      atomQueuePut(&ble_rx_queue, -1, (uint8_t *)&current_rx_packet);
      break;
    case BLE_DATAEVT_DATA_ACK:
      i=current_rx_packet.pdu[0]-1;
      ble_pipe_state[i].status=STATUS_OK;
      atomSemPut(&(ble_pipe_state[i].ack_sem));
      atomSemGet(&(ble_pipe_state[i].ack_sem), -1);
      break;
    case BLE_SYSEVT_ECHO:
    case BLE_SYSEVT_HARDWARE_ERROR:
      break;
    case BLE_SYSEVT_DEVICE_STARTED:
      ble_state.operating_mode=current_rx_packet.pdu[0];
      ble_state.hw_error=current_rx_packet.pdu[1];
      max_credits=current_rx_packet.pdu[2];
      atomSemResetCount(&ble_credit_sem, max_credits);
      atomSemPut(&ble_mode_sem);
      atomSemGet(&ble_mode_sem, -1);
      break;
    case BLE_SYSEVT_COMMAND_RESPONSE:
      ble_state.command_sent=current_rx_packet.pdu[0];
      ble_state.command_status=current_rx_packet.pdu[1];
      memcpy(ble_state.command_response_data, (uint8_t *)current_rx_packet.pdu+2, (size_t)current_rx_packet.length-3);
      atomSemPut(&ble_command_sem);
      atomSemGet(&ble_command_sem, -1);
      break;
    case BLE_SYSEVT_CONNECTED:
      ble_state.connected=1;
      ble_state.peer_address_type=current_rx_packet.pdu[0];
      for(i=0;i<6;i++) ble_state.peer_address[i]=current_rx_packet.pdu[i+1];
      ble_state.connection_interval=current_rx_packet.pdu[7];
      ble_state.connection_interval|=((0xFFFF & current_rx_packet.pdu[8]) << 8);
      ble_state.slave_latency=current_rx_packet.pdu[9];
      ble_state.slave_latency|=((0xFFFF & current_rx_packet.pdu[10]) << 8);
      ble_state.supervision_timeout=current_rx_packet.pdu[11];
      ble_state.supervision_timeout|=((0xFFFF & current_rx_packet.pdu[12]) << 8);
      ble_state.master_clock_accuracy=current_rx_packet.pdu[13];
      atomSemPut(&ble_connect_sem);
      atomSemGet(&ble_connect_sem, -1);
      break;
    case BLE_SYSEVT_DISCONNECTED:
      for(i=0;i<BLE_MAX_NO_OF_PIPES;i++) {
        ble_pipe_state[i].open_bit=0;
        ble_pipe_state[i].closed_bit=1;
      }
      ble_state.connected=0;
      ble_state.aci_status=current_rx_packet.pdu[0];
      ble_state.ble_status=current_rx_packet.pdu[1];
      atomSemResetCount(&ble_credit_sem, max_credits);
      atomSemPut(&ble_connect_sem);
      atomSemGet(&ble_connect_sem, -1);
      break;
    case BLE_SYSEVT_BOND_STATUS:
      atomSemPut(&ble_connect_sem);
      atomSemGet(&ble_connect_sem, -1);
      break;
    case BLE_SYSEVT_PIPE_STATUS:
      for(i=1;i<=BLE_MAX_NO_OF_PIPES;i++) {
        ble_pipe_state[i-1].open_bit=(current_rx_packet.pdu[i / 8] & (1U << (i % 8))) ? 1 : 0;
        ble_pipe_state[i-1].closed_bit=(current_rx_packet.pdu[(i / 8) + 8] & (1U << (i % 8))) ? 1 : 0;
        if(ble_pipe_state[i-1].open_bit) {
          atomSemPut(&(ble_pipe_state[i-1].status_sem));
          atomSemGet(&(ble_pipe_state[i-1].status_sem), -1);
        }
      }
      break;
    case BLE_SYSEVT_TIMING:
      ble_state.connection_interval=current_rx_packet.pdu[0];
      ble_state.connection_interval|=((0xFFFF & current_rx_packet.pdu[1]) << 8);
      ble_state.slave_latency=current_rx_packet.pdu[2];
      ble_state.slave_latency|=((0xFFFF & current_rx_packet.pdu[3]) << 8);
      ble_state.supervision_timeout=current_rx_packet.pdu[4];
      ble_state.supervision_timeout|=((0xFFFF & current_rx_packet.pdu[5]) << 8);
      atomSemPut(&ble_timing_sem);
      atomSemGet(&ble_timing_sem, -1);
      break;
    case BLE_SYSEVT_DISPLAY_KEY:
      break;
    case BLE_SYSEVT_KEY_REQUEST:
      break;
    default:
      break;
  }
}


static int8_t send_system_command(enum BLE_SYSTEM_COMMAND cmd, uint8_t *pdu, uint8_t len) {
  
  if(atomMutexGet(&ble_mutex, BLE_COMMAND_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  current_tx_packet.length=len+1;
  current_tx_packet.opcode=cmd;
  if(len) memcpy((void *)current_tx_packet.pdu, pdu, len);
  
  if(transmit(2*BLE_COMMAND_TIMEOUT)!=STATUS_OK) {
    atomMutexPut(&ble_mutex);
    return ERR_TIMEOUT;
  }
  
  atomMutexPut(&ble_mutex);
  
  if(atomSemGet(&ble_command_sem, BLE_COMMAND_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  if(ble_state.command_sent!=cmd) {
    return ERR_IO_ERROR;
  }
  
  return ble_state.command_status;
}


static int8_t wait_for_mode(enum BLE_OPERATING_MODE mode) {
  
  if(ble_state.operating_mode==mode) {
    return STATUS_OK;
  }
  
  if(atomSemGet(&ble_mode_sem, BLE_CONNECT_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  if(ble_state.operating_mode!=mode) {
    return ERR_PROTOCOL;
  }
  
  return STATUS_OK;
}


void ble_core_init() {
  uint8_t i;
  
  atomQueueCreate (&ble_rx_queue, ble_rx_queue_data, sizeof(struct ble_aci_packet_t), BLE_CORE_RX_QUEUE_SIZE);
  
  atomSemCreate(&ble_credit_sem, 0);
  atomSemCreate(&ble_command_sem, 0);
  atomSemCreate(&ble_connect_sem, 0);
  atomSemCreate(&ble_mode_sem, 0);
  atomSemCreate(&ble_timing_sem, 0);
  atomSemCreate(&ble_tx_sem, 0);
  atomMutexCreate(&ble_mutex);
  for(i=0;i<BLE_MAX_NO_OF_PIPES;i++) {
    atomSemCreate(&(ble_pipe_state[i].status_sem), 0);
    atomSemCreate(&(ble_pipe_state[i].ack_sem), 0);
  }
      
  spi_master_init(BLE_SPI_MODULE);
  spi_master_setup_device(BLE_SPI_MODULE, &BLE_DEVICE, SPI_MODE_0, BLE_SPI_MASTER_SPEED, 0);
  (BLE_SPI_MODULE)->CTRL |= SPI_DORD_bm;
  spi_enable(BLE_SPI_MODULE);
  
  //enable spi interrupts
  (BLE_SPI_MODULE)->INTCTRL=SPI_INTLVL_LO_gc;
  
  //set BLE_RDYN pin 0 as source of interrupts 
  BLE_RDYN_PORT->INT0MASK |= ioport_pin_to_mask(BLE_RDYN);
  //set interrupt level
  BLE_RDYN_PORT->INTCTRL = (BLE_RDYN_PORT->INTCTRL & ~PORT_INT0LVL_gm) | PORT_INT0LVL_LO_gc;
  //sense both edges
  BLE_RDYN_PORT->PIN0CTRL = IOPORT_SENSE_FALLING;
}


void ble_core_reset()
{
  uint8_t i;
  struct ble_aci_packet_t pkg;
  irqflags_t flags=cpu_irq_save();
  
  while(atomQueueGet(&ble_rx_queue, -1, (uint8_t *)&pkg)==ATOM_OK);
  
  atomSemResetCount(&ble_credit_sem, 0);
  atomSemResetCount(&ble_tx_sem, 0);
  atomSemResetCount(&ble_mode_sem, 0);
  atomSemResetCount(&ble_connect_sem, 0);
  atomSemResetCount(&ble_command_sem, 0);
  atomSemResetCount(&ble_timing_sem, 0);
  atomMutexDelete(&ble_mutex);
  atomMutexCreate(&ble_mutex);
  for(i=0;i<BLE_MAX_NO_OF_PIPES;i++) {
    atomSemResetCount(&(ble_pipe_state[i].status_sem), 0);
    atomSemResetCount(&(ble_pipe_state[i].ack_sem), 0);
  }
  
  ioport_set_pin_level(BLE_RESET, false);
  _delay_ms(10);
  ioport_set_pin_level(BLE_RESET, true);
  
  cpu_irq_restore(flags);
  
  wait_for_mode(BLE_OPERATING_MODE_SETUP);
  
  return;
}

void ble_core_standby(bool stdby) {
  if(!stdby) {
    spi_master_init(BLE_SPI_MODULE);
    spi_enable(BLE_SPI_MODULE);
  }
}


void ble_core_clear_rx_buffer() {
  struct ble_aci_packet_t rx_pkg;
  while(atomQueueGet(&ble_rx_queue, -1, (uint8_t *)&rx_pkg)!=ATOM_WOULDBLOCK);
}


int8_t ble_core_setup(uint_farptr_t ptr) {
  uint8_t i=0;
  int8_t status;
  struct ble_aci_packet_t pkg;
  
  status=wait_for_mode(BLE_OPERATING_MODE_SETUP);
  if(status!=STATUS_OK) {
    return status;
  }
  
  status=BLE_ACI_STATUS_TRANSACTION_CONTINUE;
  do {
    memcpy_PF(&pkg, ptr+i*32, 32);
    status=send_system_command(pkg.opcode, pkg.pdu, pkg.length-1);
    i++;
  } while(status==BLE_ACI_STATUS_TRANSACTION_CONTINUE);
  
  if(status!=BLE_ACI_STATUS_TRANSACTION_COMPLETE) {
    return ERR_PROTOCOL;
  }
  
  status=wait_for_mode(BLE_OPERATING_MODE_STANDBY);
  if(status!=STATUS_OK) {
    return status;
  }
  
  return STATUS_OK;
}  


int8_t ble_core_test(uint8_t feature) {
  int8_t status;
  uint8_t pdu[1];
  pdu[0]=feature;
  status=send_system_command(BLE_SYSCOM_TEST, pdu, 1);
  if(status!=STATUS_OK) {
    return status;
  }
  
  status=wait_for_mode(BLE_OPERATING_MODE_TEST);
  if(status!=STATUS_OK) {
    return status;
  }
  
  return status;
}

int8_t ble_core_sleep() {
  
  if(atomMutexGet(&ble_mutex, BLE_COMMAND_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  current_tx_packet.length=1;
  current_tx_packet.opcode=BLE_SYSCOM_SLEEP;
  
  if(transmit(BLE_COMMAND_TIMEOUT)!=STATUS_OK) {
    atomMutexPut(&ble_mutex);
    return ERR_TIMEOUT;
  }
  
  atomMutexPut(&ble_mutex);
  
  return STATUS_OK;
}

int8_t ble_core_wakeup() {
  int8_t status;
  
  if(atomMutexGet(&ble_mutex, BLE_COMMAND_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  current_tx_packet.length=1;
  current_tx_packet.opcode=BLE_SYSCOM_WAKEUP;
  
  status=transmit(BLE_COMMAND_TIMEOUT);
  
  atomMutexPut(&ble_mutex);
  
  if(status!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  status=wait_for_mode(BLE_OPERATING_MODE_STANDBY);
  if(status!=STATUS_OK) {
    return status;
  }
  
  if(atomSemGet(&ble_command_sem, BLE_COMMAND_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  if(ble_state.command_sent!=BLE_SYSCOM_WAKEUP) {
    return ERR_IO_ERROR;
  }
  
  return ble_state.command_status;
  
}


int8_t ble_core_get_device_version(uint8_t *version) {
  int8_t status;
  status=send_system_command(BLE_SYSCOM_GET_DEVICE_VERSION, NULL, 0);
  
  if(status!=STATUS_OK) {
    return status;
  }
  
  memcpy(version, ble_state.command_response_data, 11);
  
  return STATUS_OK;
}

bool ble_core_is_connected() {
  return ble_state.connected;
}

int8_t ble_core_connect(uint16_t timeout, uint16_t adv_interval) {
  int8_t status;
  uint8_t pdu[4];
  
  pdu[0]=timeout;
  pdu[1]=(timeout >> 8);
  pdu[2]=adv_interval;
  pdu[3]=(adv_interval >> 8);
  status=send_system_command(BLE_SYSCOM_CONNECT, pdu, 4);
  if(status!=BLE_ACI_STATUS_SUCCESS) {
    return status;
  }
  
  if(atomSemGet(&ble_connect_sem, timeout*SYSTEM_TICKS_PER_SEC)!=ATOM_OK)
  {
    return ERR_TIMEOUT;
  }
  
  if(!ble_state.connected) {
    return ble_state.aci_status;
  }
  return STATUS_OK;
}

int8_t ble_core_wait_for_pipe(uint8_t pipe, int32_t timeout) {
  
  if(ble_pipe_state[pipe-1].open_bit && !ble_pipe_state[pipe-1].closed_bit) {
    return STATUS_OK;
  }
  
  if(atomSemGet(&(ble_pipe_state[pipe-1].status_sem), timeout)!=ATOM_OK)
  {
    return ERR_TIMEOUT;
  }
  
  if(!ble_pipe_state[pipe-1].open_bit || ble_pipe_state[pipe-1].closed_bit) {
    return ERR_IO_ERROR;
  }
  return STATUS_OK;
}

int8_t ble_core_disconnect(uint8_t reason) {
  int8_t status;
  uint8_t pdu[1];
  
  pdu[0]=reason;
  status=send_system_command(BLE_SYSCOM_DISCONNECT, pdu, 1);
  if(status!=BLE_ACI_STATUS_SUCCESS) {
    return status;
  }
  
  if(atomSemGet(&ble_connect_sem, BLE_COMMAND_TIMEOUT)!=ATOM_OK)
  {
    return ERR_TIMEOUT;
  }
  
  if(ble_state.connected) {
    return ble_state.aci_status;
  }
  
  return STATUS_OK;
}

int8_t ble_core_open_adv_pipe(uint8_t bitmap[8]) {
  int8_t status;
  
  status=send_system_command(BLE_SYSCOM_OPEN_ADV_PIPE, bitmap, 8);
  if(status!=BLE_ACI_STATUS_SUCCESS) {
    return status;
  }
  
  return STATUS_OK;
}


int8_t ble_core_open_remote_pipe(uint8_t pipe) {
  int8_t status;
  uint8_t pdu[1];
  
  pdu[0]=pipe;
  status=send_system_command(BLE_SYSCOM_OPEN_REMOTE_PIPE, pdu, 1);
  if(status!=BLE_ACI_STATUS_SUCCESS) {
    return status;
  }
  
  if(atomSemGet(&(ble_pipe_state[pipe-1].status_sem), BLE_COMMAND_TIMEOUT)!=ATOM_OK)
  {
    return ERR_TIMEOUT;
  }
  
  return STATUS_OK;
}


int8_t ble_core_close_remote_pipe(uint8_t pipe) {
  int8_t status;
  uint8_t pdu[1];
  
  pdu[0]=pipe;
  status=send_system_command(BLE_SYSCOM_OPEN_REMOTE_PIPE, pdu, 1);
  if(status!=BLE_ACI_STATUS_SUCCESS) {
    return status;
  }
  
  if(atomSemGet(&(ble_pipe_state[pipe-1].status_sem), BLE_COMMAND_TIMEOUT)!=ATOM_OK)
  {
    return ERR_TIMEOUT;
  }
  
  return STATUS_OK;
}


int8_t ble_core_send_data(uint8_t pipe, uint8_t *data, uint8_t len, bool wait_for_ack) {
  
  if(atomMutexGet(&ble_mutex, BLE_COMMAND_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  if(!ble_state.connected || !ble_pipe_state[pipe-1].open_bit) {
    atomMutexPut(&ble_mutex);
    return ERR_PROTOCOL;
  }
  
  if(atomSemGet(&ble_credit_sem, BLE_COMMAND_TIMEOUT)!=ATOM_OK) {
    atomMutexPut(&ble_mutex);
    return ERR_IO_ERROR;
  }
  
  current_tx_packet.length=len+2;
  current_tx_packet.opcode=BLE_DATACOM_SEND_DATA;
  current_tx_packet.pdu[0]=pipe;
  memcpy((void *)(current_tx_packet.pdu+1), data, len);
  
  if(transmit(BLE_COMMAND_TIMEOUT)!=STATUS_OK) {
    atomMutexPut(&ble_mutex);
    return ERR_TIMEOUT;
  }
  
  atomMutexPut(&ble_mutex);
  
  if(wait_for_ack) {
    if(atomSemGet(&(ble_pipe_state[pipe-1].ack_sem), BLE_ACK_TIMEOUT)!=ATOM_OK) {
      return ERR_TIMEOUT;
    }
  }
  
  return ble_pipe_state[pipe-1].status;
}


int8_t ble_core_set_local_data(uint8_t pipe, uint8_t *data, uint8_t len) {
  
  if(atomMutexGet(&ble_mutex, BLE_COMMAND_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  if(!ble_pipe_state[pipe-1].open_bit) {
    atomMutexPut(&ble_mutex);
    return ERR_PROTOCOL;
  }
  
  if(atomSemGet(&ble_credit_sem, BLE_COMMAND_TIMEOUT)!=ATOM_OK) {
    atomMutexPut(&ble_mutex);
    return ERR_IO_ERROR;
  }
  
  current_tx_packet.length=len+2;
  current_tx_packet.opcode=BLE_DATACOM_SET_LOCAL_DATA;
  current_tx_packet.pdu[0]=pipe;
  memcpy((void *)(current_tx_packet.pdu+1), data, len);
  
  if(transmit(BLE_COMMAND_TIMEOUT)!=STATUS_OK) {
    atomMutexPut(&ble_mutex);
    return ERR_TIMEOUT;
  }
  
  atomMutexPut(&ble_mutex);
  
  if(atomSemGet(&ble_command_sem, BLE_COMMAND_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  if(ble_state.command_sent!=BLE_DATACOM_SET_LOCAL_DATA) {
    return ERR_IO_ERROR;
  }
  
  return ble_pipe_state[pipe-1].status;
}

int8_t ble_core_change_timing_request(uint16_t interval_min, uint16_t interval_max, uint16_t slave_latency, uint16_t timeout) {
  int8_t status;
  uint8_t pdu[8];
  
  pdu[0]=interval_min;
  pdu[1]=(interval_min >> 8);
  pdu[2]=interval_max;
  pdu[3]=(interval_max >> 8);
  pdu[4]=slave_latency;
  pdu[5]=(slave_latency >> 8);
  pdu[6]=timeout;
  pdu[7]=(timeout >> 8);
  
  if(atomMutexGet(&ble_mutex, BLE_COMMAND_TIMEOUT)!=ATOM_OK) {
    return ERR_TIMEOUT;
  }
  
  status=send_system_command(BLE_SYSCOM_CHANGE_TIMING_REQUEST, pdu, 8);
  if(status!=BLE_ACI_STATUS_SUCCESS) {
    atomMutexPut(&ble_mutex);
    return status;
  }
  
  if(atomSemGet(&ble_timing_sem, 2*BLE_CONNECT_TIMEOUT)!=ATOM_OK)
  {
    atomMutexPut(&ble_mutex);
    return ERR_TIMEOUT;
  }
  
  atomMutexPut(&ble_mutex);
  return STATUS_OK;
}

