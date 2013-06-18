/*
 * hmac_sha1.c
 *
 * Created: 12/12/2012 7:55:33 PM
 *  Author: Administrator
 */ 
#include "hmac_sha1.h"

#define IPAD 0x36
#define OPAD 0x5C

int8_t hmac_sha1(void* dest, uint8_t* key_buffer, io_input_stream_t *src){
  SHA1Context *ctx;
  uint8_t i;
  uint16_t l;
  
  for (i=0; i<SHA1_BLOCK_BYTES; ++i){
    key_buffer[i] ^= IPAD;
  }
  
  ctx=(SHA1Context *)mem_safe_malloc(sizeof(SHA1Context));
  if(ctx==NULL) {
    return ERR_NO_MEMORY;
  }
  
  SHA1Reset(ctx);
  SHA1Input(ctx, (const uint8_t *)key_buffer, SHA1_BLOCK_BYTES);
  while(src->bytes_left>0) {
    l=src->bytes_left>SHA1_BYTES ? SHA1_BYTES : (uint16_t)src->bytes_left;
    src->read(src, dest, l);
    SHA1Input(ctx, (const uint8_t *)dest, l);
  }
  if(SHA1Result(ctx, (uint8_t *)dest)!=STATUS_OK) {
    mem_safe_free(ctx);
    return ERR_BAD_DATA;
  }
  
  //since buffer still contains key xor ipad
  for (i=0; i<SHA1_BLOCK_BYTES; ++i){
    key_buffer[i] ^= IPAD ^ OPAD;
  }
  
  SHA1Reset(ctx);
  SHA1Input(ctx, (const uint8_t *)key_buffer, SHA1_BLOCK_BYTES);
  SHA1Input(ctx, dest, SHA1_BYTES);
  if(SHA1Result(ctx, (uint8_t *)dest)!=STATUS_OK) {
    mem_safe_free(ctx);
    return ERR_BAD_DATA;
  }
  
  mem_safe_free(ctx);
  
  return STATUS_OK;
}
