/*
 * ant_msg.c
 *
 * Created: 2/22/2013 4:34:03 PM
 *  Author: Administrator
 */ 
#include "ant_msg.h"


void send_channel_available_msg(uint8_t channel, uint32_t network_id) {
  uint8_t data[8];
  data[0]=0xFF;
  data[1]=0xFF;
  data[2]=ANT_CHANNEL_AVAILABLE_MESSAGE;
  data[3]=ant_state.node_id;
  data[4]=(uint8_t) network_id;
  data[5]=(uint8_t) (network_id >> 8);
  data[6]=(uint8_t) (network_id >> 16);
  data[7]=(uint8_t) (network_id >> 24);
  ant_core_send_broadcast_data(channel, data, false);
  
  return;
}


void send_address_available_msg(uint8_t channel, uint16_t new_shared_address) {
  uint8_t data[8];
  
  data[0]=0xFF;
  data[1]=0xFF;
  data[2]=ANT_ADDRESS_AVAILABLE_MESSAGE;
  data[3]=0;
  data[4]=0;
  data[5]=4;
  data[6]=(uint8_t) new_shared_address;
  data[7]=(uint8_t) new_shared_address >> 8;
  ant_core_send_broadcast_data(channel, data, false);
  
  return;
}


void send_busy_acquiring_msg(uint8_t channel, uint32_t device_id) {
  uint8_t data[8];
  data[0]=0xFE;
  data[1]=0xFF;
  data[2]=ANT_BUSY_AQUIRING_MESSAGE;
  data[3]=0;
  data[4]=(uint8_t) device_id;
  data[5]=(uint8_t) (device_id >> 8);
  data[6]=(uint8_t) (device_id >> 16);
  data[7]=(uint8_t) (device_id >> 24);
  ant_core_send_broadcast_data(channel, data, false);
  
  return;
}

void send_shared_channel_info_msg(uint8_t channel, uint16_t shared_address) {
  uint8_t data[8];
  data[0]=(uint8_t) shared_address;
  data[1]=(uint8_t) (shared_address >> 8);
  data[2]=ANT_SHARED_CHANNEL_INFO_MESSAGE;
  data[3]=0;
  data[4]=0;
  data[5]=4;
  data[6]=0;
  data[7]=20;
  ant_core_send_broadcast_data(channel, data, false);
  return;
}

void send_channel_info_msg(uint8_t channel, uint8_t *freq, uint16_t period) {
  uint8_t data[8];
  data[0]=0xFE;
  data[1]=0xFF;
  data[2]=ANT_CHANNEL_INFO_MESSAGE;
  data[3]=freq[0];
  data[4]=freq[1];
  data[5]=freq[2];
  data[6]=(uint8_t) period;
  data[7]=(uint8_t) (period >> 8);
  ant_core_send_broadcast_data(channel, data, false);
  return;
}



int8_t send_request_channel_msg(uint8_t channel, uint8_t node_id, uint32_t device_id) {
  uint8_t data[8];
  data[0]=0xFF;
  data[1]=0xFF;
  data[2]=ANT_REQUEST_CHANNEL_MESSAGE;
  data[3]=node_id;
  data[4]=(uint8_t) device_id;
  data[5]=(uint8_t) (device_id >> 8);
  data[6]=(uint8_t) (device_id >> 16);
  data[7]=(uint8_t) (device_id >> 24);
  return ant_core_send_acknowledged_data(channel, data);
}


int8_t send_request_address_msg(uint8_t channel, uint32_t device_id) {
  uint8_t data[8];
  data[0]=0xFF;
  data[1]=0xFF;
  data[2]=ANT_REQUEST_ADDRESS_MESSAGE;
  data[3]=0;
  data[4]=(uint8_t) device_id;
  data[5]=(uint8_t) (device_id >> 8);
  data[6]=(uint8_t) (device_id >> 16);
  data[7]=(uint8_t) (device_id >> 24);
  return ant_core_send_acknowledged_data(channel, data);
}


int8_t send_confirm_aquire_msg(uint8_t channel, uint32_t device_id) {
  uint8_t data[8];
  data[0]=0xFE;
  data[1]=0xFF;
  data[2]=ANT_CONFIRM_AQUIRE_MESSAGE;
  data[3]=0;
  data[4]=(uint8_t) device_id;
  data[5]=(uint8_t) (device_id >> 8);
  data[6]=(uint8_t) (device_id >> 16);
  data[7]=(uint8_t) (device_id >> 24);
  return ant_core_send_acknowledged_data(channel, data);
}


void send_slave_control_msg(uint8_t channel, uint16_t shared_address, uint16_t poll_interval, uint16_t timeout, uint8_t no_of_polls) {
  uint8_t data[8];
  data[0]=(uint8_t) shared_address;
  data[1]=(uint8_t) (shared_address >> 8);
  data[2]=ANT_SLAVE_CONTROL_MESSAGE;
  data[3]=(uint8_t) poll_interval;
  data[4]=(uint8_t) (poll_interval >> 8);
  data[5]=(uint8_t) timeout;
  data[6]=(uint8_t) (timeout >> 8);
  data[7]=no_of_polls;
  ant_core_send_broadcast_data(channel, data, false);
  return;
}


void send_poll_begin_msg(uint8_t channel, uint16_t shared_address) {
  uint8_t data[8];
  data[0]=(uint8_t) shared_address;
  data[1]=(uint8_t) (shared_address >> 8);
  data[2]=ANT_SLAVE_POLL_BEGIN_MESSAGE;
  data[3]=0;
  data[4]=0;
  data[5]=0;
  data[6]=0;
  data[7]=0;
  ant_core_send_broadcast_data(channel, data, false);
  return;
}

void send_poll_msg(uint8_t channel, uint16_t shared_address, bool blocking) {
  uint8_t data[8];
  data[0]=(uint8_t) shared_address;
  data[1]=(uint8_t) (shared_address >> 8);
  data[2]=ANT_SLAVE_POLL_MESSAGE;
  data[3]=0;
  data[4]=0;
  data[5]=0;
  data[6]=0;
  data[7]=0;
  ant_core_send_broadcast_data(channel, data, blocking);
  return;
}

void send_poll_end_msg(uint8_t channel, uint16_t shared_address) {
  uint8_t data[8];
  data[0]=(uint8_t) shared_address;
  data[1]=(uint8_t) (shared_address >> 8);
  data[2]=ANT_SLAVE_POLL_END_MESSAGE;
  data[3]=0;
  data[4]=0;
  data[5]=0;
  data[6]=0;
  data[7]=0;
  ant_core_send_broadcast_data(channel, data, false);
  return;
}
