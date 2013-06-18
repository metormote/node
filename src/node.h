/*
 * node.h
 *
 * Created: 10/4/2011 11:24:31 AM
 *  Author: Administrator
 */ 


#ifndef NODE_H_
#define NODE_H_


/*
 * Include header files for all drivers that have been imported from
 * AVR Software Framework (ASF).
 */
#include "asf.h"
#include "board.h"
#include "conf_board.h"
#include "atom.h"
#include "atomport-private.h"
#include "atomsem.h"
#include "ant/ant.h"
#include "ble/ble.h"
#include "utils/ui.h"
#include "utils/mem.h"
#include "utils/ui.h"
#include "utils/base64.h"
#include "utils/crypto.h"
#include "utils/hmac_sha1.h"
#include "utils/utils.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "iop.pb.h"
#include "node.pb.h"
#include "power.h"
#include "seed.h"
#include "fw/firmware.pb.h"
#include "xboot/xbootapi.h"
/*
 * Idle thread stack size
 *
 * This needs to be large enough to handle any interrupt handlers
 * and callbacks called by interrupt handlers (e.g. user-created
 * timer callbacks) as well as the saving of all context when
 * switching away from this thread.
 *
 * In this case, the idle stack is allocated at the end of the 
 * ram memory area.
 */
#define IDLE_THREAD_STACK_SIZE_BYTES       0x100

/*
 * Main thread stack sizes
 *
 * The Main thread stacks generally need to be larger than the idle
 * thread stack, as not only does it need to store interrupt handler
 * stack saves and context switch saves, but the application main thread
 * will generally be carrying out more nested function calls and require
 * stack for application code local variables etc.
 *
 */
#define ANT_THREAD_STACK_SIZE_BYTES        0x2A0
#define ANT_THREAD_PRIO                    32

#define BLE_THREAD_STACK_SIZE_BYTES        0x200
#define BLE_THREAD_PRIO                    16

#define NODE_THREAD_STACK_SIZE_BYTES       0x1A0
#define NODE_THREAD_PRIO                   128

#define NODE_OUT_MSG_CODE                  32
#define NODE_IN_MSG_CODE                   33

#define FIRMWARE_UPGRADE_MSG_CODE          6

#define ENV_BUF_SIZE  256
#define MSG_BUF_SIZE  192

typedef enum {
    NODE_TYPE_HUB = 0,
    NODE_TYPE_SLAVE  = 1
} node_type_t;

struct node_signature_t {
  uint64_t device_id;
  uint8_t key[AES_KEY_SIZE];
  int16_t temp_offset;
};

struct node_state_t {
  node_type_t type;
  uint64_t network_id;
  uint32_t transfer_interval;
  uint16_t polling_interval;
  uint16_t timeout;
  uint8_t no_of_polls;
  uint64_t rx_nonce;
  uint64_t tx_nonce;
};

int node_start (void (*init_callback)(void), void (*app_entry_point)(void), int8_t (*callback)(uint64_t, pb_istream_t *, uint64_t *, pb_ostream_t *));
int8_t node_send_message(uint64_t msg_code, int8_t (*callback)(pb_ostream_t *stream, void *arg), void *arg);
int8_t node_snapshot(pb_ostream_t *msg_output_stream, void *arg);

#endif /* NODE_H_ */