/*
 * io_stream.h
 *
 * Created: 3/9/2012 3:45:33 PM
 *  Author: Administrator
 */ 
#ifndef IO_STREAM_H_
#define IO_STREAM_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include "status_codes.h"
#include "atom.h"
#include "atomqueue.h"
#include "pb_decode.h"

typedef struct _io_output_stream_ io_output_stream_t;
typedef struct _io_input_stream_ io_input_stream_t;

struct _io_output_stream_ {
  int8_t (*write)(io_output_stream_t *stream, uint8_t *data, uint16_t len);
  void *state;
  int32_t timeout;
  uint32_t bytes_written;
  uint32_t max_len;
};


struct _io_input_stream_ {
  int8_t (*read)(io_input_stream_t *stream, uint8_t *data, uint16_t len);
  void *state;
  int32_t timeout;
  uint32_t bytes_left;
};


io_output_stream_t io_output_stream_from_buffer(uint8_t *buf, uint32_t len);
io_input_stream_t io_input_stream_from_buffer(uint8_t *buf, uint32_t len);
io_output_stream_t io_output_stream_from_queue(ATOM_QUEUE *queue);
io_input_stream_t io_input_stream_from_queue(ATOM_QUEUE *queue);
pb_istream_t io_to_pb_istream(io_input_stream_t *source);

#endif /* IO_STREAM_H_ */