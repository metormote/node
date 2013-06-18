#include "base64.h"

/*
** Translation Table as described in RFC1113
*/
static const char cb64[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

//url safe version
//static const char cb64[] PROGMEM = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";


/*
** encodeblock
**
** encode 3 8-bit binary bytes as 4 '6-bit' characters
*/
static void encodeblock( uint8_t *in, char *out, int len )
{
    *out++ = pgm_read_byte( cb64 + (in[0] >> 2) );
    *out++ = pgm_read_byte( cb64 + (((in[0] & 0x03) << 4) | ((in[1] & 0xf0) >> 4)) );
    *out++ = (char) (len > 1 ? pgm_read_byte( cb64 + (((in[1] & 0x0f) << 2) | ((in[2] & 0xc0) >> 6)) ) : '=');
    *out = (char) (len > 2 ? pgm_read_byte( cb64 + (in[2] & 0x3f) ) : '=');
}

/*
** encode
**
** base64 encode adding padding as per spec.
*/
void base64_encode(uint8_t* crypt, uint32_t length, char* enc)
{
    uint8_t in[3];
    char out[4];
    uint32_t i, j, len;
    
    for( j=0; j < length/3; j++ ) {
      for( i = 0; i < 3; i++ ) {
        in[i] = crypt[3*j+i];
      }
      encodeblock( in, out, 3 );
      for( i = 0; i < 4; i++ ) {
        enc[j*4+i]=out[i];
      }
    }
    
    len = length % 3;
    if(len) {
      for( i = 0; i < 3; i++ ) {
          if( i < len ) {
            in[i] = crypt[3*j+i];
          }
          else {
            in[i] = 0;
          }
      }
      
      encodeblock( in, out, len );
      for( i = 0; i < 4; i++ ) {
        enc[j*4+i]=out[i];
      }
      j++;
    }
    
    enc[4*j]='\0';
}

/*
** Translation Table to decode
*/
static const char cd64[] PROGMEM = "|$$$}rstuvwxyz{$$$$$$$>?@ABCDEFGHIJKLMNOPQRSTUVW$$$$$$XYZ[\\]^_`abcdefghijklmnopq";

//url safe version
//static const char cd64[] PROGMEM = "$$|$$rstuvwxyz{$$$$$$$>?@ABCDEFGHIJKLMNOPQRSTUVW$$$$}$XYZ[\\]^_`abcdefghijklmnopq";

/*
** decodeblock
**
** decode 4 '6-bit' characters into 3 8-bit binary bytes
*/
static void decodeblock( char *in, uint8_t *out )
{   
    *out++ = (uint8_t) (in[0] << 2 | in[1] >> 4);
    *out++ = (uint8_t) (in[1] << 4 | in[2] >> 2);
    *out = (uint8_t) (((in[2] << 6) & 0xc0) | in[3]);
}

/*
** decode
**
** decode a base64 encoded buffer
*/
uint32_t base64_decode(char* enc, uint8_t* crypt)
{
    char in[4], v;
    uint8_t out[3];
    int i, j, k, len;
    
    k=0;
    j=0;
    while( enc[j] != '\0' ) {
        for( len = 0, i = 0; i < 4 && enc[j] != '\0'; i++ ) {
            v = 0;
            while( enc[j] != '\0' && v == 0 ) {
                v = (char) enc[j++];
                v = (char) ((v < 43 || v > 122) ? 0 : pgm_read_byte( cd64 + (v - 43) ));
                if( v ) {
                    v = (char) ((v == '$') ? 0 : v - 61);
                }
            }
            if( v ) {
                len++;
                in[ i ] = (char) (v - 1);
            }
            else {
                in[i] = 0;
            }
        }
        if( len ) {
            decodeblock( in, out );
            for( i = 0; i < len - 1; i++ ) {
                crypt[k++]=out[i];
            }
        }
    }
    return k;
}


