#include "utils.h"

static bool encode_env_callback(pb_ostream_t *stream, const pb_field_t *field, const void *arg);
static bool decode_env_callback(pb_istream_t *stream, const pb_field_t *field, void *arg);

/**
 * \b decode_envelope
 *
 * Decode an Envelope protobuf stream. The message data will be stored
 * in pb_istream_t *msg_stream on return.
 *
 * @param[in] env Pointer to envelope.
 * @param[in] stream Protobuf data input stream.
 *
 * @return Success
 */
bool decode_envelope(iop_Envelope *env, pb_istream_t *stream, pb_istream_t *msg_stream) {
  //set msg callback
  env->msg.funcs.decode=decode_env_callback;
  env->msg.arg=msg_stream;
  
  //decode envelope
  if (!pb_decode(stream, iop_Envelope_fields, env)) {
    env->msg.arg=NULL;
    return false;
  }  
  return true;
}

//use this callback to decode the envelope and the message
static bool decode_env_callback(pb_istream_t *stream, const pb_field_t *field, void *arg) {
  pb_istream_t *msg_stream=(pb_istream_t *)arg;
  uint8_t *ptr = (uint8_t *)stream->state;
  
  msg_stream->bytes_left=stream->bytes_left;
  msg_stream->state=stream->state;
  
  stream->state=ptr+stream->bytes_left;
  stream->bytes_left=0;
  
  return true;
}

/**
 * \b encode_envelope
 *
 * Encode an Envelope as protobuf stream. The message data must
 * be available in env.msg.arg as a pb_bytes_array_t.
 *
 * @param[in] stream Protobuf data output stream.
 * @param[in] env Pointer to envelope.
 *
 * @return Success
 */
bool encode_envelope(pb_ostream_t *stream, iop_Envelope *env, pb_istream_t *msg_stream) {
  
  //set msg callback
  env->msg.funcs.encode=encode_env_callback;
  env->msg.arg=msg_stream;
  
  //serialize the envelope
  if (!pb_encode(stream, iop_Envelope_fields, env)) {
    env->msg.arg=NULL;
    return false;
  }
  
  return true;
}




static bool encode_env_callback(pb_ostream_t *stream, const pb_field_t *field, const void *arg) {
  pb_istream_t *msg_stream=(pb_istream_t *)arg;
  
  if(arg==NULL) return false;
  
  //write the message to the msg field in the envelope
  if (!pb_encode_tag_for_field(stream, field))
    return false;
  
  if (!pb_encode_string(stream, msg_stream->state, msg_stream->bytes_left))
    return false;
  
  return true;
}



int8_t calc_hash(uint8_t *hash, uint8_t *key_buffer, uint8_t *key, iop_Envelope *env, io_input_stream_t *src) {
  uint8_t *ptr;
  memset(key_buffer, 0, SHA1_BLOCK_BYTES);
  
  ptr=key_buffer;
  memcpy(ptr, key, AES_KEY_SIZE);
  ptr+=AES_KEY_SIZE;
  if(env->has_src) {
    memcpy(ptr, &(env->src), sizeof(env->msgCode));
  }
  ptr+=sizeof(env->src);
  if(env->has_dst) {
    memcpy(ptr, &(env->dst), sizeof(env->msgCode));
  }
  ptr+=sizeof(env->dst);
  memcpy(ptr, &(env->msgCode), sizeof(env->msgCode));
  ptr+=sizeof(env->msgCode);
  memcpy(ptr, &(env->nonce), sizeof(env->nonce));
  ptr+=sizeof(env->nonce);
  memcpy(ptr, env->iv.bytes, env->iv.size); 
  ptr+=env->iv.size;
  
  return hmac_sha1(hash, key_buffer, src);
}


char *trim(char *str)
{
  char *end;

  // Trim leading space
  while(isspace(*str)) str++;

  if(*str == 0)  // All spaces?
    return str;

  // Trim trailing space
  end = str + strlen(str) - 1;
  while(end > str && isspace(*end)) end--;

  // Write new null terminator
  *(end+1) = 0;

  return str;
}

/*
uint8_t isspace(char c) {
    switch (c) {
        case ' ':
        case '\n':
        case '\t':
        case '\f':
        case '\r':
            return 1;
            break;
        default:
            return 0;
            break;
    }
}
*/

/*
 * get_free_memory (void)
 *
 * Function: Get the number of free memory bytes
 *
 * Returns the amount of remaining memory (bytes)
 */
uint16_t get_free_memory(void)
{
  uint16_t free_memory;
  char* res[90];
  int i=0;
 
  for(;;)
  {
    res[i]= (char*) calloc(100,sizeof(char));
    if(res[i]==NULL)
    {
      free_memory = i*100;
      break;
    }
    i++;
  }
  for(i-=1;i>=0;i--)
  {
    free(res[i]);
    res[i]=NULL;
  }
  return free_memory;
}


