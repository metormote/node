/*
 * pmbus.c
 *
 * Created: 2/14/2012 9:52:57 PM
 *  Author: Administrator
 */ 
# include "pmbus.h"


static twi_options_t OPTIONS = {
  .chip = 0x20,
  .speed = 100000
};

static ATOM_MUTEX pmbus_mutex;
static ATOM_QUEUE command_queue;
uint8_t command_queue_data[PMBUS_COMMAND_QUEUE_SIZE*sizeof(void *)];

static bool encode_reply_callback(pb_ostream_t *stream, const pb_field_t *field, const void *arg);
static bool decode_command_callback(pb_istream_t *stream, const pb_field_t *field, void *arg);
static bool decode_command_data_callback(pb_istream_t *stream, const pb_field_t *field, void *arg);
static bool encode_device_status_callback(pb_ostream_t *stream, const pb_field_t *field, const void *arg);
static int8_t execute_command(pb_ostream_t *stream, uint8_t address, pmbus_Command *command, pmbus_Response *response, const pb_field_t *field);
static bool encode_response_callback(pb_ostream_t *stream, const pb_field_t *field, const void *arg);


int8_t pmbus_init() {
  int8_t status;
   
  atomMutexCreate(&pmbus_mutex);
  atomQueueCreate(&command_queue, command_queue_data, sizeof(void *), PMBUS_COMMAND_QUEUE_SIZE);
  
  status=twi_master_setup(SENSOR_TWI, &OPTIONS);
  (SENSOR_TWI)->CTRL = TWI_SDAHOLD_bm;
  (SENSOR_TWI)->MASTER.CTRLB = ((SENSOR_TWI)->MASTER.CTRLB & ~ TWI_MASTER_TIMEOUT_gm) | TWI_MASTER_TIMEOUT_50US_gc;
  
  return status;
}

int8_t pmbus_scan(uint8_t range_low, uint8_t range_high, uint8_t *result) {
  uint8_t status;
  twi_package_t tx_package, rx_package;
  uint8_t address;
  uint8_t i,b,mask;
  uint8_t *p;
  
  rx_package.addr[0]=pmbus_CommandCode_STATUS_BYTE;
  //rx_package.addr=pmbus_CommandCode_MFR_MODEL;
  rx_package.addr_length=1;
  rx_package.length=1;
  rx_package.buffer=&b;
    
  p=result;
  mask=1;
  i=0;
  
  atomMutexGet(&pmbus_mutex, PMBUS_TIMEOUT);
  
  for(address=range_low;address<range_high;address++) {
    rx_package.chip=address;
    status=twi_master_read(SENSOR_TWI, &rx_package);
    if(status == STATUS_OK) {
      *p|=mask;
    }
    if((++i % 8)==0) {
      p++;
      mask=1;
    }
    else {
      mask <<=  1;
    }
    _delay_ms(1);
  }
  
  atomMutexPut(&pmbus_mutex);
  return STATUS_OK;
}


int8_t pmbus_process_message(pb_istream_t *msg_stream, pb_ostream_t *resp_stream) {
  pmbus_In msg;
  pmbus_Out reply;
  
  atomMutexGet(&pmbus_mutex, PMBUS_TIMEOUT);
  msg.command.funcs.decode=decode_command_callback;
  
  //decode message
  if (!pb_decode(msg_stream, pmbus_In_fields, &msg)) {
    atomMutexPut(&pmbus_mutex);
    return ERR_IO_ERROR;
  }
  
  reply.pmbusAddress=msg.pmbusAddress;
  reply.response.funcs.encode=encode_reply_callback;
  reply.response.arg = &(reply.pmbusAddress);
  
  //encode reply
  if (!pb_encode(resp_stream, pmbus_Out_fields, &reply)) {
    atomMutexPut(&pmbus_mutex);
    return ERR_IO_ERROR;
  }
  
  atomMutexPut(&pmbus_mutex);
  return STATUS_OK;
}

static bool decode_command_callback(pb_istream_t *stream, const pb_field_t *field, void *arg) {
  pmbus_Command *command=(pmbus_Command *)mem_safe_calloc(1, sizeof(pmbus_Command));
  pb_bytes_array_t *data=(pb_bytes_array_t *)mem_safe_malloc(32+sizeof(size_t));
  
  data->size=0;
  command->data.funcs.decode=decode_command_data_callback;
  command->data.arg=data;
  
  if(pb_decode(stream, field->ptr, command)) {
    if(atomQueuePut(&command_queue, -1, (uint8_t *)&command)==ATOM_OK) {
      return true;
    }
  }
  mem_safe_free(command);
  mem_safe_free(data);
  return false;
}

static bool decode_command_data_callback(pb_istream_t *stream, const pb_field_t *field, void *arg) {
  //pmbus_Command *command=(pmbus_Command *)arg;
  pb_bytes_array_t *data=(pb_bytes_array_t *)arg;
  //command->data.arg=data;
  data->size=stream->bytes_left;
  if(pb_read(stream, data->bytes, data->size)) {
    return true;
  }
  return false;
}

static bool encode_reply_callback(pb_ostream_t *stream, const pb_field_t *field, const void *arg) {
  uint8_t address;
  int8_t status;
  pmbus_Command *command;
  pmbus_Response response;
  uint8_t resp_buf[32];
  pb_bytes_array_t *resp_bytes = (pb_bytes_array_t *)resp_buf;
  
  if(arg==NULL) return false;
  
  memset(&response, 0, sizeof(pmbus_Response));
  
  address = *((uint8_t *)arg);
  
  //set response callback
  response.data.funcs.encode=encode_response_callback;
  response.data.arg = resp_bytes;
  
  status=STATUS_OK;
  while(atomQueueGet(&command_queue, -1, (uint8_t *)&command)==ATOM_OK) {
    status=execute_command(stream, address, command, &response, field);
    mem_safe_free(command->data.arg);
    mem_safe_free(command);
  }
  
  return status==STATUS_OK;
}



int8_t pmbus_read_device_status(pb_ostream_t *stream, void *address) {
  bool status=STATUS_OK;
  pmbus_Out msg;
  
  atomMutexGet(&pmbus_mutex, PMBUS_TIMEOUT);
  
  msg.pmbusAddress = *((uint8_t *)address);
  
  //set msg callback
  msg.response.funcs.encode=encode_device_status_callback;
  msg.response.arg = address;
  //start reading status and serialize the result
  if (!pb_encode(stream, pmbus_Out_fields, &msg)) {
    status=ERR_IO_ERROR;
  }    
  
  atomMutexPut(&pmbus_mutex);
  
  return status;
}    


static bool encode_device_status_callback(pb_ostream_t *stream, const pb_field_t *field, const void *arg) {
  uint8_t address;
  uint16_t status_word;
  pmbus_Command command;
  pmbus_Response response;
  uint8_t resp_buf[32];
  pb_bytes_array_t *resp_bytes = (pb_bytes_array_t *)resp_buf;
  
  if(arg==NULL) return false;
  
  memset(&response, 0, sizeof(pmbus_Response));
  
  address = *((uint8_t *)arg);
  
  //set response callback
  response.data.funcs.encode=encode_response_callback;
  response.data.arg = resp_bytes;
  
  //set command callback
  command.type = pmbus_Command_Type_READ;
  command.responseLen=2;
  
  command.code=pmbus_CommandCode_READ_VIN;
  if(execute_command(stream, address, &command, &response, field)!=STATUS_OK) return false;
  
  command.code=pmbus_CommandCode_READ_VOUT;
  if(execute_command(stream, address, &command, &response, field)!=STATUS_OK) return false;
  
  command.code=pmbus_CommandCode_READ_IOUT;
  if(execute_command(stream, address, &command, &response, field)!=STATUS_OK) return false;
  
  command.code=pmbus_CommandCode_READ_TEMPERATURE_1;
  if(execute_command(stream, address, &command, &response, field)!=STATUS_OK) return false;
  
  command.code=pmbus_CommandCode_READ_DUTY_CYCLE;
  if(execute_command(stream, address, &command, &response, field)!=STATUS_OK) return false;
  
  command.code=pmbus_CommandCode_READ_FREQUENCY;
  if(execute_command(stream, address, &command, &response, field)!=STATUS_OK) return false;
  
  command.code=pmbus_CommandCode_STATUS_WORD;
  if(execute_command(stream, address, &command, &response, field)!=STATUS_OK) return false;
  
  if(!response.has_error) {
    status_word = resp_bytes->bytes[0];
    status_word |= ((0xFFFF & resp_bytes->bytes[1]) << 8); 
    
    command.responseLen=1;
    if(status_word & PMBUS_STATUS_CML_MASK) {
      command.code=pmbus_CommandCode_STATUS_CML;
      if(execute_command(stream, address, &command, &response, field)!=STATUS_OK) return false;
    }
    
    if(status_word & PMBUS_STATUS_TEMPERATURE_MASK) {
      command.code=pmbus_CommandCode_STATUS_TEMPERATURE;
      if(execute_command(stream, address, &command, &response, field)!=STATUS_OK) return false;
    }
    
    if(status_word & PMBUS_STATUS_IOUT_MASK) {
      command.code=pmbus_CommandCode_STATUS_IOUT;
      if(execute_command(stream, address, &command, &response, field)!=STATUS_OK) return false;
    }
    
    if(status_word & PMBUS_STATUS_VOUT_MASK) {
      command.code=pmbus_CommandCode_STATUS_VOUT;
      if(execute_command(stream, address, &command, &response, field)!=STATUS_OK) return false;
    }
    
    if(status_word & PMBUS_STATUS_OTHER_MASK) {
      command.code=pmbus_CommandCode_STATUS_OTHER;
      if(execute_command(stream, address, &command, &response, field)!=STATUS_OK) return false;
    }
    
    if(status_word & PMBUS_STATUS_FANS_MASK) {
      command.code=pmbus_CommandCode_STATUS_FANS_1_2;
      if(execute_command(stream, address, &command, &response, field)!=STATUS_OK) return false;
    
      command.code=pmbus_CommandCode_STATUS_FANS_3_4;
      if(execute_command(stream, address, &command, &response, field)!=STATUS_OK) return false;
    }
    
    if(status_word & PMBUS_STATUS_MFR_MASK) {
      command.code=pmbus_CommandCode_STATUS_MFR_SPECIFIC;
      if(execute_command(stream, address, &command, &response, field)!=STATUS_OK) return false;
    }
    
    if(status_word & PMBUS_STATUS_INPUT_MASK) {
      command.code=pmbus_CommandCode_STATUS_INPUT;
      if(execute_command(stream, address, &command, &response, field)!=STATUS_OK) return false;
    }
    
  }
  
  return true;
}


static int8_t execute_command(pb_ostream_t *stream, uint8_t address, pmbus_Command *command, pmbus_Response *response, const pb_field_t *field) {
  int8_t status;
  twi_package_t rx_package;
  pb_bytes_array_t *resp_bytes = (pb_bytes_array_t *)response->data.arg;
  pb_bytes_array_t *req_bytes;
  
  response->code = command->code;
  response->has_error=false;
  
  rx_package.chip = address;
  rx_package.addr_length = 1;
  rx_package.addr[0] = command->code;
  
  //send write command
  if(command->type == pmbus_Command_Type_WRITE) {
    req_bytes = (pb_bytes_array_t *)command->data.arg;
    rx_package.buffer = req_bytes->bytes;
    rx_package.length = req_bytes->size;
    
    status=twi_master_write(SENSOR_TWI, &rx_package);
    if(status != STATUS_OK) {
      //set error response
      resp_bytes->size=0;
      response->has_error = true;
      response->error = pmbus_Response_Error_IO_ERR;
      return status;
    }
    
    if(command->responseLen==0) {
      return STATUS_OK;
    }
    
    _delay_ms(1);
  }
  
  if(command->responseLen>0) {
    //send read command
    rx_package.buffer=resp_bytes->bytes;
    rx_package.length=command->responseLen; 
  
    status=twi_master_read(SENSOR_TWI, &rx_package);
    
    if(status == STATUS_OK) {
      //set length of response data
      resp_bytes->size=rx_package.length;
    }
    else {
      //set error response
      resp_bytes->size=0;
      response->has_error = true;
      response->error = pmbus_Response_Error_IO_ERR;
      return status;
    }
  
    //write the message to the msg field in the envelope
    if (!pb_encode_tag_for_field(stream, field))
      return ERR_IO_ERROR;
  
    //serialize the response
    if (!pb_enc_submessage(stream, field, response))
        return ERR_IO_ERROR;
    
    _delay_ms(1);
  }
  return STATUS_OK;
}


static bool encode_response_callback(pb_ostream_t *stream, const pb_field_t *field, const void *arg) {
  pb_bytes_array_t *buf=(pb_bytes_array_t *)arg;
  
  if(arg==NULL) return false;
  if(buf->size==0) return true;
  
  //write the message to the msg field in the envelope
  if (!pb_encode_tag_for_field(stream, field))
    return false;
  
  if (!pb_enc_bytes(stream, field, buf))
    return false;
  
  return true;
}  