/*
 * io.c
 *
 * Created: 3/11/2012 3:37:16 PM
 *  Author: Administrator
 */ 
#include "io_stream.h"

static int8_t io_write_buf(io_output_stream_t *stream, uint8_t *data, uint16_t len);
static int8_t io_read_buf(io_input_stream_t *stream, uint8_t *data, uint16_t len);
static int8_t io_write_queue(io_output_stream_t *stream, uint8_t *data, uint16_t len);
static int8_t io_read_queue(io_input_stream_t *stream, uint8_t *data, uint16_t len);
static bool io_read_pb(pb_istream_t *stream, uint8_t *data, size_t len);

io_output_stream_t io_output_stream_from_buffer(uint8_t *buf, uint32_t len) {
  io_output_stream_t stream;
  stream.timeout=0;
  stream.write=io_write_buf;
  stream.state=buf;
  stream.bytes_written=0;
  stream.max_len=len;
  return stream;
}

io_input_stream_t io_input_stream_from_buffer(uint8_t *buf, uint32_t len) {
  io_input_stream_t stream;
  stream.timeout=0;
  stream.read=io_read_buf;
  stream.state=buf;
  stream.bytes_left=len;
  return stream;
}


io_output_stream_t io_output_stream_from_queue(ATOM_QUEUE *queue) {
  io_output_stream_t stream;
  stream.write=io_write_queue;
  stream.state=queue;
  stream.timeout=-1;
  stream.bytes_written=0;
  stream.max_len=queue->max_num_msgs;
  return stream;
}


io_input_stream_t io_input_stream_from_queue(ATOM_QUEUE *queue) {
  io_input_stream_t stream;
  stream.read=io_read_queue;
  stream.state=queue;
  stream.timeout=-1;
  stream.bytes_left=queue->num_msgs_stored;
  return stream;
}

pb_istream_t io_to_pb_istream(io_input_stream_t *source) {
  pb_istream_t stream;
  stream.callback=io_read_pb;
  stream.bytes_left=(size_t)source->bytes_left;
  stream.state=source;
  return stream;
}

static bool io_read_pb(pb_istream_t *stream, uint8_t *data, size_t len) {
  io_input_stream_t *source = (io_input_stream_t *)stream->state;
  source->read(source, data, len);
  stream->bytes_left=source->bytes_left;
  return true;
}

static int8_t io_write_buf(io_output_stream_t *stream, uint8_t *data, uint16_t len) {
  uint8_t *dst = (uint8_t*)stream->state;
  memcpy(dst, data, len);
  stream->state = dst + len;
  stream->bytes_written+=len;
  return STATUS_OK;
}

static int8_t io_read_buf(io_input_stream_t *stream, uint8_t *data, uint16_t len) {
  uint8_t *src = (uint8_t*)stream->state;
  memcpy(data, src, len);
  stream->state = src + len;
  stream->bytes_left-=len;
  return STATUS_OK;
}

static int8_t io_write_queue(io_output_stream_t *stream, uint8_t *data, uint16_t len) {
  ATOM_QUEUE *dst = (ATOM_QUEUE *)stream->state;
  for(;len!=0;len--) {
    if(atomQueuePut(dst, stream->timeout, data++)!=ATOM_OK) {
      return ERR_IO_ERROR;
    }
  }
  stream->bytes_written+=len;
  return STATUS_OK;
}

static int8_t io_read_queue(io_input_stream_t *stream, uint8_t *data, uint16_t len) {
  ATOM_QUEUE *src = (ATOM_QUEUE *)stream->state;
  for(;len!=0;len--) {
    if(atomQueueGet(src, stream->timeout, data++)!=ATOM_OK) {
      return ERR_IO_ERROR;
    }
  }
  stream->bytes_left=src->num_msgs_stored;
  return STATUS_OK;
}