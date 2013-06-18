/*
 * hmac_sha1.h
 *
 * Created: 12/12/2012 7:53:45 PM
 *  Author: Administrator
 */ 
#ifndef HMAC_SHA1_H_
#define HMAC_SHA1_H_

#include <stdint.h>
#include <string.h>
#include "sha1.h"
#include "io_stream.h"

int8_t hmac_sha1(void* dest, uint8_t* key_buffer, io_input_stream_t *src);


#endif /* HMAC_SHA1_H_ */