/*
 * crypto.h
 *
 * Created: 9/2/2011 11:02:48 AM
 *  Author: Administrator
 */ 
#ifndef CRYPTO_H_
#define CRYPTO_H_

#include <stdint.h>
#include <stdbool.h>
#include <aes.h>
#include <status_codes.h>

#include "atom.h"
#include "atomsem.h"
#include "atommutex.h"

#define CRYPTO_TIMEOUT  1*SYSTEM_TICKS_PER_SEC
#define CRYPTO_MUTEX_TIMEOUT  5*SYSTEM_TICKS_PER_SEC


#ifdef __cplusplus
extern "C" { 
#endif

void crypto_init(uint8_t *key);
void crypto_init_iv(uint8_t *iv);
int8_t crypto_start_encrypt(uint8_t *iv);
int8_t crypto_encrypt(uint8_t *buf, uint16_t *len);
int8_t crypto_encrypt_block(uint8_t *p, uint8_t len);
int8_t crypto_start_decrypt(void);
int8_t crypto_decrypt_block(uint8_t *p, uint8_t *iv, uint8_t *len, bool last);
int8_t crypto_decrypt(uint8_t *buf, uint8_t *iv, uint16_t *len);
void crypto_final(void);

#ifdef __cplusplus
}
#endif

#endif /* CRYPTO_H_ */