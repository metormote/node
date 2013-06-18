/*
 * ant_msg.h
 *
 * Created: 2/22/2013 4:33:48 PM
 *  Author: Administrator
 */ 


#ifndef ANT_MSG_H_
#define ANT_MSG_H_

#include "ant_core.h"
#include "ant_state.h"

#define ANT_ADDRESS_AVAILABLE_MESSAGE      0xFF
#define ANT_BUSY_AQUIRING_MESSAGE          0xFE
#define ANT_REQUEST_ADDRESS_MESSAGE        0xFD
#define ANT_CONFIRM_AQUIRE_MESSAGE         0xFC
#define ANT_NO_ADDRESS_AVAILABLE_MESSAGE   0xFB
#define ANT_SHARED_CHANNEL_INFO_MESSAGE    0xFA
#define ANT_REQUEST_CHANNEL_INFO_MESSAGE   0xF0

#define ANT_CHANNEL_AVAILABLE_MESSAGE      0x80
#define ANT_CHANNEL_INFO_MESSAGE           0x81
#define ANT_REQUEST_CHANNEL_MESSAGE        0x82
#define ANT_NETWORK_INFO_MESSAGE           0x83
#define ANT_SLAVE_CONTROL_MESSAGE          0x84
#define ANT_SLAVE_POLL_BEGIN_MESSAGE       0x85
#define ANT_SLAVE_POLL_MESSAGE             0x86
#define ANT_SLAVE_POLL_END_MESSAGE         0x87

#ifdef __cplusplus
extern "C" { 
#endif

void send_channel_available_msg(uint8_t channel, uint32_t network_id);
void send_address_available_msg(uint8_t channel, uint16_t new_shared_address);
void send_busy_acquiring_msg(uint8_t channel, uint32_t device_id);
void send_shared_channel_info_msg(uint8_t channel, uint16_t shared_address);
void send_channel_info_msg(uint8_t channel, uint8_t *freq, uint16_t period);
int8_t send_request_channel_msg(uint8_t channel, uint8_t node_id, uint32_t device_id);
int8_t send_request_address_msg(uint8_t channel, uint32_t device_id);
int8_t send_confirm_aquire_msg(uint8_t channel, uint32_t device_id);
void send_slave_control_msg(uint8_t channel, uint16_t shared_address, uint16_t poll_interval, uint16_t timeout, uint8_t no_of_polls);
void send_poll_begin_msg(uint8_t channel, uint16_t shared_address);
void send_poll_msg(uint8_t channel, uint16_t shared_address, bool blocking);
void send_poll_end_msg(uint8_t channel, uint16_t shared_address);

#ifdef __cplusplus
}
#endif

#endif /* ANT_MSG_H_ */