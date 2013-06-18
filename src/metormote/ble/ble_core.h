/*
 * ble_core.h
 *
 * Created: 10/1/2011 5:43:47 PM
 *  Author: Erik
 */ 


#ifndef BLE_CORE_H_
#define BLE_CORE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "sysclk.h"
#include "board.h"
#include "util/delay.h"
#include "spi.h"
#include "spi_master.h"
#include "atom.h"
#include "atommutex.h"
#include "atomsem.h"
#include "atomqueue.h"
#include "atomport.h"

/* system commands */
enum BLE_SYSTEM_COMMAND {
  BLE_SYSCOM_TEST                   =0x01,
  BLE_SYSCOM_SLEEP                  =0x04,
  BLE_SYSCOM_GET_DEVICE_VERSION     =0x09,
  BLE_SYSCOM_ECHO                   =0x02,
  BLE_SYSCOM_WAKEUP                 =0x05,
  BLE_SYSCOM_GET_BATTERY_LEVEL      =0x0B,
  BLE_SYSCOM_GET_TEMPERATURE        =0x0C,
  BLE_SYSCOM_SETUP                  =0x06,
  BLE_SYSCOM_SET_TX_POWER           =0x12,
  BLE_SYSCOM_GET_DEVICE_ADDRESS     =0x0A,
  BLE_SYSCOM_CONNECT                =0x0F,
  BLE_SYSCOM_BOND                   =0x10,
  BLE_SYSCOM_DISCONNECT             =0x11,
  BLE_SYSCOM_CHANGE_TIMING_REQUEST  =0x13,
  BLE_SYSCOM_OPEN_REMOTE_PIPE       =0x14,
  BLE_SYSCOM_OPEN_ADV_PIPE          =0x1B,
  BLE_SYSCOM_DTM_COMMAND            =0x03,
  BLE_SYSCOM_READ_DYNAMIC_DATA      =0x07,
  BLE_SYSCOM_WRITE_DYNAMIC_DATA     =0x08,
  BLE_SYSCOM_RADIO_RESET            =0x0E
};

/* system events */
enum BLE_SYSTEM_EVENT {
  BLE_SYSEVT_DEVICE_STARTED         =0x81,
  BLE_SYSEVT_ECHO                   =0x82,
  BLE_SYSEVT_HARDWARE_ERROR         =0x83,
  BLE_SYSEVT_COMMAND_RESPONSE       =0x84,
  BLE_SYSEVT_CONNECTED              =0x85,
  BLE_SYSEVT_DISCONNECTED           =0x86,
  BLE_SYSEVT_BOND_STATUS            =0x87,
  BLE_SYSEVT_PIPE_STATUS            =0x88,
  BLE_SYSEVT_TIMING                 =0x89,
  BLE_SYSEVT_DISPLAY_KEY            =0x8E,
  BLE_SYSEVT_KEY_REQUEST            =0x8F
};

/* data commands */
enum BLE_DATA_COMMAND {
  BLE_DATACOM_SEND_DATA             =0x15,
  BLE_DATACOM_REQUEST_DATA          =0x17,
  BLE_DATACOM_SET_LOCAL_DATA        =0x0D,
  BLE_DATACOM_SEND_DATA_ACK         =0x16
};


/* data events */
enum BLE_DATA_EVENT {
  BLE_DATAEVT_DATA_RECEIVED         =0x8C,
  BLE_DATAEVT_PIPE_ERROR            =0x8D,
  BLE_DATAEVT_DATA_CREDIT           =0x8A,
  BLE_DATAEVT_DATA_ACK              =0x8B
};

/* data events */
enum BLE_OPERATING_MODE {
  BLE_OPERATING_MODE_TEST           =0x01,
  BLE_OPERATING_MODE_SETUP          =0x02,
  BLE_OPERATING_MODE_STANDBY        =0x03
};

/*
 * ACI status codes applicable for nRF8001.The status code is used to indicate the 
 * general command execution status or to identify the cause of an error.
 */
/* Success */
#define BLE_ACI_STATUS_SUCCESS                        0x00 
/* Transaction continuation status */
#define BLE_ACI_STATUS_TRANSACTION_CONTINUE           0x01 
/* Transaction completed */
#define BLE_ACI_STATUS_TRANSACTION_COMPLETE           0x02
/* Extended status, further checks needed */ 
#define BLE_ACI_STATUS_EXTENDED                       0x03
/* Unknown error */ 
#define BLE_ACI_STATUS_ERROR_UNKNOWN                  0x80
/* Internal error */ 
#define BLE_ACI_STATUS_ERROR_INTERNAL                 0x81
/* Unknown command */ 
#define BLE_ACI_STATUS_ERROR_CMD_UNKNOWN              0x82
/* Command invalid in the current device state */ 
#define BLE_ACI_STATUS_ERROR_DEVICE_STATE_INVALID     0x83
/* Invalid length */
#define BLE_ACI_STATUS_ERROR_INVALID_LENGTH           0x84
/* Invalid input parameters */
#define BLE_ACI_STATUS_ERROR_INVALID_PARAMETER        0x85
/* Busy */
#define BLE_ACI_STATUS_ERROR_BUSY                     0x86
/* Invalid data format or contents */
#define BLE_ACI_STATUS_ERROR_INVALID_DATA             0x87
/* CRC mismatch */
#define BLE_ACI_STATUS_ERROR_CRC_MISMATCH             0x88
/* Unsupported setup format */
#define BLE_ACI_STATUS_ERROR_UNSUPPORTED_SETUP_FORMAT 0x89
/* Invalid sequence number during a write dynamic data sequence */
#define BLE_ACI_STATUS_ERROR_INVALID_SEQ_NO           0x8A
/* Setup data is locked and cannot be modified */
#define BLE_ACI_STATUS_ERROR_SETUP_LOCKED             0x8B
/* Setup error due to lock verification failure */
#define BLE_ACI_STATUS_ERROR_LOCK_FAILED              0x8C
/* Bond required: Local Pipes need bonded/trusted peer */
#define BLE_ACI_STATUS_ERROR_BOND_REQUIRED            0x8D
/* Command rejected as a transaction is still pending */
#define BLE_ACI_STATUS_ERROR_REJECTED                 0x8E
/* Pipe Error Event : Data size exceeds size specified for pipe, Transmit failed */
#define BLE_ACI_STATUS_ERROR_DATA_SIZE                0x8F
/* Pipe Error Event : Transmit failed, Invalid or unavailable Pipe number or unknown pipe type */
#define BLE_ACI_STATUS_ERROR_PIPE_INVALID             0x90
/* Pipe Error Event : Credit not available */
#define BLE_ACI_STATUS_ERROR_CREDIT_NOT_AVAILABLE     0x91
/* Pipe Error Event : Peer device has sent an error on an pipe operation on*/
#define BLE_ACI_STATUS_ERROR_PEER_ATT_ERROR           0x92
/* Connection was not established before the BTLE advertising was stopped */
#define BLE_ACI_STATUS_ERROR_ADVT_TIMEOUT             0x93
/* Remote device triggered a Security Manager Protocol error */
#define BLE_ACI_STATUS_ERROR_PEER_SMP_ERROR           0x94


/*
 * Bonding status codes applicable for the BondStatusEvent. The status code is used to report 
 * the bonding procedure execution status.
*/
/* Bonding succeeded */
#define BLE_ACI_BOND_STATUS_SUCCESS                     0x00 
/* Bonding failed */
#define BLE_ACI_BOND_STATUS_FAILED                      0x01 
/* Bonding error: Timeout while bonding */
#define BLE_ACI_BOND_STATUS_FAILED_TIMED_OUT            0x02 
/* Bonding error: Passkey entry failed */
#define BLE_ACI_BOND_STATUS_FAILED_PASSKEY_ENTRY_FAILED 0x81 
/* Bonding error: OOB unavailable */
#define BLE_ACI_BOND_STATUS_FAILED_OOB_UNAVAILABLE      0x82 
/* Bonding error: Authentication request failed */
#define BLE_ACI_BOND_STATUS_FAILED_AUTHENTICATION_REQ   0x83 
/* Bonding error: Confirm value failed */
#define BLE_ACI_BOND_STATUS_FAILED_CONFIRM_VALUE        0x84 
/* Bonding error: Pairing unsupported */
#define BLE_ACI_BOND_STATUS_FAILED_PAIRING_UNSUPPORTED  0x85 
/* Bonding error: Invalid encryption key size */
#define BLE_ACI_BOND_STATUS_FAILED_ENCRYPTION_KEY_SIZE  0x86 
/* Bonding error: Unsupported SMP command */
#define BLE_ACI_BOND_STATUS_FAILED_SMP_CMD_UNSUPPORTED  0x87 
/* Bonding error: Unspecified reason */
#define BLE_ACI_BOND_STATUS_FAILED_UNSPECIFIED_REASON   0x88 
/* Bonding error: Too many attempts */
#define BLE_ACI_BOND_STATUS_FAILED_REPEATED_ATTEMPTS    0x89 
/* Bonding error: Invalid parameters */
#define BLE_ACI_BOND_STATUS_FAILED_INVALID_PARAMETERS   0x8A 


/*
 * Error codes applicable for the PipeErrorEvent. The error code is used to report an 
 * error related to data transfer or service pipe association.
*/
/* The attribute handle given was not valid on this server */
#define BLE_ERROR_INVALID_HANDLE                    0x01
/* The attribute cannot be read. */
#define BLE_ERROR_READ_NOT_PERMITTED                0x02
/* The attribute cannot be written. */
#define BLE_ERROR_WRITE_NOT_PERMITTED               0x03
/* The attribute PDU was invalid. */
#define BLE_ERROR_INVALID_PDU                       0x04
/* The attribute requires authentication before it can be read or written. */
#define BLE_ERROR_INSUFFICIENT_AUTHENTICATION       0x05
/* Attribute server does not support the request received from the client. */
#define BLE_ERROR_REQUEST_NOT_SUPPORTED             0x06
/* Offset specified was past the end of the attribute. */
#define BLE_ERROR_INVALID_OFFSET                    0x07
/* The attribute requires authorization before it can be read or written. */
#define BLE_ERROR_INSUFFICIENT_AUTHORIZATION        0x08
/* Too many prepare writes have been queued. */
#define BLE_ERROR_PREPARE_QUEUE_FULL                0x09
/* No attribute found within the given attribute handle range. */
#define BLE_ERROR_ATTRIBUTE_NOT_FOUND               0x0A
/* The attribute cannot be read or written using the Read Blob Request */
#define BLE_ERROR_ATTRIBUTE_NOT_LONG                0x0B
/* The Encryption Key Size used for encrypting this link is insufficient. */
#define BLE_ERROR_INSUFFICIENT_ENCRYPTION_KEY_SIZE  0x0C
/* The attribute value length is invalid for the operation. */
#define BLE_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH    0x0D
/* The attribute request that was requested has encountered an error that was unlikely, and therefore could not be completed as requested. */
#define BLE_ERROR_UNLIKELY_ERROR                    0x0E
/* The attribute requires encryption before it can be read or written. */
#define BLE_ERROR_INSUFFICIENT_ENCRYPTION           0x0F
/* The attribute type is not a supported grouping attribute as defined by a higher layer specification. */
#define BLE_ERROR_UNSUPPORTED_GROUP_TYPE            0x10
/* Insufficient Resources to complete the request */
#define BLE_ERROR_INSUFFICIENT_RESOURCES            0x11

#define BLE_DISCONNECT_REASON_USER            0x01
#define BLE_DISCONNECT_REASON_TIMING          0x02

//! Select the SPI module AT45DBX is connected to
#define BLE_SPI_MODULE          BLE_SPI

//! SPI master speed in Hz.
#define BLE_SPI_MASTER_SPEED    1000000

//! Number of bits in each SPI transfer.
#define BLE_SPI_BITS            8

#define BLE_MAX_NO_OF_PIPES       4
#define BLE_CORE_RX_QUEUE_SIZE    4

#define BLE_COMMAND_TIMEOUT       2*SYSTEM_TICKS_PER_SEC
#define BLE_CONNECT_TIMEOUT       5*SYSTEM_TICKS_PER_SEC
#define BLE_ACK_TIMEOUT           2*SYSTEM_TICKS_PER_SEC

#ifdef __cplusplus
extern "C" { 
#endif


/* bluetooth low energy state */
struct ble_state_t {
  uint8_t operating_mode:2, hw_error:1, connected:1, peer_address_type:2, reserved:4;
  uint8_t peer_address[6];
  uint16_t connection_interval;
  uint16_t slave_latency;
  uint16_t supervision_timeout;
  uint8_t master_clock_accuracy;
  uint8_t aci_status;
  uint8_t ble_status;
  uint8_t command_sent;
  uint8_t command_status;
  uint8_t command_response_data[27];
};

struct ble_pipe_state_t {
  volatile uint8_t open_bit:1,closed_bit:1,reserved:6;
  volatile uint8_t status;
  ATOM_SEM status_sem;
  ATOM_SEM ack_sem;
};

/* application controller interface packet */
struct ble_aci_packet_t {
  uint8_t length;
  uint8_t opcode;
  uint8_t pdu[30];
};

extern ATOM_QUEUE ble_rx_queue;

void ble_core_init(void);
void ble_core_reset(void);
void ble_core_standby(bool stdby);
void ble_core_clear_rx_buffer(void);
int8_t ble_core_setup(uint_farptr_t ptr);
int8_t ble_core_test(uint8_t feature);
int8_t ble_core_sleep(void);
int8_t ble_core_wakeup(void);
int8_t ble_core_get_device_version(uint8_t *version);
int8_t ble_core_connect(uint16_t timeout, uint16_t adv_interval);
int8_t ble_core_wait_for_pipe(uint8_t pipe, int32_t timeout);
bool ble_core_is_connected(void);
int8_t ble_core_disconnect(uint8_t reason);
int8_t ble_core_send_data(uint8_t pipe, uint8_t *data, uint8_t len, bool wait_for_ack);
int8_t ble_core_set_local_data(uint8_t pipe, uint8_t *data, uint8_t len);
int8_t ble_core_change_timing_request(uint16_t interval_min, uint16_t interval_max, uint16_t slave_latency, uint16_t timeout);
int8_t ble_core_open_adv_pipe(uint8_t bitmap[8]);
int8_t ble_core_open_remote_pipe(uint8_t pipe);
int8_t ble_core_close_remote_pipe(uint8_t pipe);

#ifdef __cplusplus
}
#endif

#endif /* BLE_CORE_H_ */