#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "pb_decode.h"
#include "pb_encode.h"
#include "iop.pb.h"
#include "mem.h"
#include "sha1.h"
#include "hmac_sha1.h"
#include "aes.h"

#ifdef __cplusplus
extern "C" { 
#endif
    int8_t calc_hash(uint8_t *hash, uint8_t *key_buffer, uint8_t *key, iop_Envelope *env, io_input_stream_t *src);
    bool decode_envelope(iop_Envelope *env, pb_istream_t *src, pb_istream_t *msg_stream);
    bool encode_envelope(pb_ostream_t *stream, iop_Envelope *env, pb_istream_t *msg_stream);
    
    char *trim(char *str);
    //uint8_t isspace(char c);
    uint16_t get_free_memory(void);
  
#ifdef __cplusplus
}
#endif
