#include <stdint.h>
#include <avr/pgmspace.h>


#ifdef __cplusplus
extern "C" { 
#endif

    void base64_encode(uint8_t* crypt, uint32_t length, char* enc);
    uint32_t base64_decode(char* enc, uint8_t* crypt);

#ifdef __cplusplus
}
#endif



