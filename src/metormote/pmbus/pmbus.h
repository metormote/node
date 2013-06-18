/*
 * pmbus.h
 *
 * Created: 2/14/2012 9:53:11 PM
 *  Author: Administrator
 */ 

#ifndef PMBUS_H_
#define PMBUS_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include "board.h"
#include <util/delay.h>
#include "twi_master.h"
#include "utils.h"
#include "atom.h"
#include "atomqueue.h"
#include "atommutex.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "pmbus.pb.h"

/*
#define PMBUS_COMMAND_PAGE                    0x00
#define PMBUS_COMMAND_OPERATION               0x01
#define PMBUS_COMMAND_ON_OFF_CONFIG           0x02
#define PMBUS_COMMAND_CLEAR_FAULTS            0x03
#define PMBUS_COMMAND_PHASE                   0x04

#define PMBUS_COMMAND_WRITE_PROTECT           0x10
#define PMBUS_COMMAND_STORE_DEFAULT_ALL       0x11
#define PMBUS_COMMAND_RESTORE_DEFAULT_ALL     0x12
#define PMBUS_COMMAND_STORE_DEFAULT_CODE      0x13
#define PMBUS_COMMAND_RESTORE_DEFAULT_CODE    0x14
#define PMBUS_COMMAND_STORE_USER_ALL          0x15
#define PMBUS_COMMAND_RESTORE_USER_ALL        0x16
#define PMBUS_COMMAND_STORE_USER_CODE         0x17
#define PMBUS_COMMAND_RESTORE_USER_CODE       0x18
#define PMBUS_COMMAND_CAPABILITY              0x19
#define PMBUS_COMMAND_QUERY                   0x1A

#define PMBUS_COMMAND_VOUT_MODE               0x20   
#define PMBUS_COMMAND_VOUT_COMMAND            0x21 
#define PMBUS_COMMAND_VOUT_TRIM               0x22
#define PMBUS_COMMAND_VOUT_CAL_OFFSET         0x23 
#define PMBUS_COMMAND_VOUT_MAX                0x24
#define PMBUS_COMMAND_VOUT_MARGIN_HIG         0x25 
#define PMBUS_COMMAND_VOUT_MARGIN_LOW         0x26
#define PMBUS_COMMAND_VOUT_TRANSITION_RATE    0x27
#define PMBUS_COMMAND_VOUT_DROOP              0x28  
#define PMBUS_COMMAND_VOUT_SCALE_LOOP         0x29   
#define PMBUS_COMMAND_VOUT_SCALE_MONITOR      0x2A

#define PMBUS_COMMAND_COEFFICIENTS            0x30
#define PMBUS_COMMAND_POUT_MAX                0x31   
#define PMBUS_COMMAND_MAX_DUTY                0x32
#define PMBUS_COMMAND_FREQUENCY_SWITC         0x33      
#define PMBUS_COMMAND_VIN_ON                  0x35
#define PMBUS_COMMAND_VIN_OFF                 0x36
#define PMBUS_COMMAND_INTERLEAVE              0x37


#define PMBUS_COMMAND_IOUT_CAL_GAIN           0x38  
#define PMBUS_COMMAND_IOUT_CAL_OFFSET         0x39
#define PMBUS_COMMAND_FAN_CONFIG_1_2          0x3A
#define PMBUS_COMMAND_FAN_COMMAND_1           0x3B
#define PMBUS_COMMAND_FAN_COMMAND_2           0x3C
#define PMBUS_COMMAND_FAN_CONFIG_3_4          0x3D
#define PMBUS_COMMAND_FAN_COMMAND_3           0x3E
#define PMBUS_COMMAND_FAN_COMMAND_4           0x3F   
#define PMBUS_COMMAND_VOUT_OV_FAULT_LIMIT     0x40
#define PMBUS_COMMAND_VOUT_OV_FAULT_RESPONSE  0x41
#define PMBUS_COMMAND_VOUT_OV_WARN_LIMIT      0x42
#define PMBUS_COMMAND_VOUT_UV_WARN_LIMIT      0x43
#define PMBUS_COMMAND_VOUT_UV_FAULT_LIMIT     0x44
#define PMBUS_COMMAND_VOUT_UV_FAULT_RESPONSE  0x45
#define PMBUS_COMMAND_IOUT_OC_FAULT_LIMIT     0x46
#define PMBUS_COMMAND_IOUT_OC_FAULT_RESPONSE  0x47
#define PMBUS_COMMAND_IOUT_OC_LV_FAULT_LIMIT  0x48
#define PMBUS_COMMAND_IOUT_OC_LV_FAULT_RESPONSE 0x49
#define PMBUS_COMMAND_IOUT_OC_WARN_LIMIT      0x4A
#define PMBUS_COMMAND_IOUT_UC_FAULT_LIMIT     0x4B   
#define PMBUS_COMMAND_IOUT_UC_FAULT_RESPONSE  0x4C


#define PMBUS_COMMAND_OT_FAULT_LIMIT          0x4F
#define PMBUS_COMMAND_OT_FAULT_RESPONSE       0x50  
#define PMBUS_COMMAND_OT_WARN_LIMIT           0x51
#define PMBUS_COMMAND_UT_WARN_LIMIT           0x52
#define PMBUS_COMMAND_UT_FAULT_LIMIT          0x53
#define PMBUS_COMMAND_UT_FAULT_RESPONSE       0x54
#define PMBUS_COMMAND_VIN_OV_FAULT_LIMIT      0x55
#define PMBUS_COMMAND_VIN_OV_FAULT_RESPONSE   0x56
#define PMBUS_COMMAND_VIN_OV_WARN_LIMIT       0x57
#define PMBUS_COMMAND_VIN_UV_WARN_LIMIT       0x58

#define PMBUS_COMMAND_VIN_UV_FAULT_LIMIT      0x59
#define PMBUS_COMMAND_VIN_UV_FAULT_RESPONSE   0x5A
#define PMBUS_COMMAND_IIN_OC_FAULT_LIMIT      0x5B
#define PMBUS_COMMAND_IIN_OC_FAULT_RESPONSE   0x5C
#define PMBUS_COMMAND_IIN_OC_WARN_LIMIT       0x5D
#define PMBUS_COMMAND_POWER_GOOD_ON           0x5E 
#define PMBUS_COMMAND_POWER_GOOD_OFF          0x5F
#define PMBUS_COMMAND_TON_DELAY               0x60
#define PMBUS_COMMAND_TON_RISE                0x61
#define PMBUS_COMMAND_TON_MAX_FAULT_LIMIT     0x62
#define PMBUS_COMMAND_TON_MAX_FAULT_RESPONSE  0x63
#define PMBUS_COMMAND_TOFF_DELAY              0x64
#define PMBUS_COMMAND_TOFF_FALL               0x65
#define PMBUS_COMMAND_TOFF_MAX_WARN_LIMIT     0x66
#define PMBUS_COMMAND_POUT_OP_FAULT_LIMIT     0x68
#define PMBUS_COMMAND_POUT_OP_FAULT_RESPONSE  0x69
#define PMBUS_COMMAND_POUT_OP_WARN_LIMIT      0x6A
#define PMBUS_COMMAND_PIN_OP_WARN_LIMIT       0x6B


#define PMBUS_COMMAND_STATUS_BYTE             0x78
#define PMBUS_COMMAND_STATUS_WORD             0x79

#define PMBUS_COMMAND_STATUS_VOUT             0x7A
#define PMBUS_COMMAND_STATUS_IOUT             0x7B
#define PMBUS_COMMAND_STATUS_INPUT            0x7C
#define PMBUS_COMMAND_STATUS_TEMPERATURE      0x7D
#define PMBUS_COMMAND_STATUS_CML              0x7E
#define PMBUS_COMMAND_STATUS_OTHER            0x7F
#define PMBUS_COMMAND_STATUS_MFR_SPECIFIC     0x80
#define PMBUS_COMMAND_STATUS_FANS_1_2         0x81
#define PMBUS_COMMAND_STATUS_FANS_3_4         0x82
#define PMBUS_COMMAND_READ_VIN                0x88
#define PMBUS_COMMAND_READ_IIN                0x89
#define PMBUS_COMMAND_READ_VCAP               0x8A
#define PMBUS_COMMAND_READ_VOUT               0x8B
#define PMBUS_COMMAND_READ_IOUT               0x8C
#define PMBUS_COMMAND_READ_TEMPERATURE_1      0x8D
#define PMBUS_COMMAND_READ_TEMPERATURE_2      0x8E
#define PMBUS_COMMAND_READ_TEMPERATURE_3      0x8F
#define PMBUS_COMMAND_READ_FAN_SPEED_1        0x90
#define PMBUS_COMMAND_READ_FAN_SPEED_2        0x91
#define PMBUS_COMMAND_READ_FAN_SPEED_3        0x92
#define PMBUS_COMMAND_READ_FAN_SPEED_4        0x93   
#define PMBUS_COMMAND_READ_DUTY_CYCLE         0x94
#define PMBUS_COMMAND_READ_FREQUENCY          0x95
#define PMBUS_COMMAND_READ_POUT               0x96
#define PMBUS_COMMAND_READ_PIN                0x97
#define PMBUS_COMMAND_PMBUS_REVISION          0x98
#define PMBUS_COMMAND_MFR_ID                  0x99
#define PMBUS_COMMAND_MFR_MODEL               0x9A
#define PMBUS_COMMAND_MFR_REVISION            0x9B
#define PMBUS_COMMAND_MFR_LOCATION            0x9C
#define PMBUS_COMMAND_MFR_DATE                0x9D  
#define PMBUS_COMMAND_MFR_SERIAL              0x9E 
 
#define PMBUS_COMMAND_MFR_VIN_MIN             0xA0
#define PMBUS_COMMAND_MFR_VIN_MAX             0xA1
#define PMBUS_COMMAND_MFR_IIN_MAX             0xA2
#define PMBUS_COMMAND_MFR_PIN_MAX             0xA3
#define PMBUS_COMMAND_MFR_VOUT_MIN            0xA4
#define PMBUS_COMMAND_MFR_VOUT_MAX            0xA5
#define PMBUS_COMMAND_MFR_IOUT_MAX            0xA6
#define PMBUS_COMMAND_MFR_POUT_MAX            0xA7
#define PMBUS_COMMAND_MFR_TAMBIENT_MAX        0xA8
#define PMBUS_COMMAND_MFR_TAMBIENT_MIN        0xA9
#define PMBUS_COMMAND_MFR_EFFICIENCY_LL       0xAA
#define PMBUS_COMMAND_MFR_EFFICIENCY_HL       0xAB

#define PMBUS_COMMAND_USER_DATA_00            0xB0
#define PMBUS_COMMAND_USER_DATA_15            0xBF

#define PMBUS_COMMAND_MFR_SPECIFIC_00         0xD0
#define PMBUS_COMMAND_MFR_SPECIFIC_45         0xFD

#define PMBUS_COMMAND_MFR_SPECIFIC_COMMAND_EXT 0xFE
#define PMBUS_COMMAND_PMBUS_COMMAND_EXT       0xFF 
*/


#define PMBUS_TIMEOUT                   1*SYSTEM_TICKS_PER_SEC

#define PMBUS_COMMAND_QUEUE_SIZE        16

#define PMBUS_OUT_MSG_CODE              65
#define PMBUS_IN_MSG_CODE               66

#define PMBUS_STATUS_CML_MASK           0x0002
#define PMBUS_STATUS_TEMPERATURE_MASK   0x0004
#define PMBUS_STATUS_IOUT_MASK          0x4010
#define PMBUS_STATUS_VOUT_MASK          0x8020
#define PMBUS_STATUS_OTHER_MASK         0x0200
#define PMBUS_STATUS_FANS_MASK          0x0400
#define PMBUS_STATUS_MFR_MASK           0x1000
#define PMBUS_STATUS_INPUT_MASK         0x2000


#ifdef __cplusplus
extern "C" { 
#endif


int8_t pmbus_init(void);
int8_t pmbus_scan(uint8_t range_low, uint8_t range_high, uint8_t *result);
int8_t pmbus_read_device_status(pb_ostream_t *out, void *address);
int8_t pmbus_process_message(pb_istream_t *msg_stream, pb_ostream_t *resp_stream);

#ifdef __cplusplus
}
#endif

#endif /* PMBUS_H_ */