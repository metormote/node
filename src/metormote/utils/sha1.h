/*
 *  sha1.h
 *
 *  Copyright (C) 1998, 2009
 *  Paul E. Jones <paulej@packetizer.com>
 *  All Rights Reserved
 *
 *  Description:
 *      This class implements the Secure Hashing Standard as defined
 *      in FIPS PUB 180-1 published April 17, 1995.
 *
 *      Many of the variable names in the SHA1Context, especially the
 *      single character names, were used because those were the names
 *      used in the publication.
 *
 *      Please read the file sha1.c for more information.
 *
 */

#ifndef _SHA1_H_
#define _SHA1_H_

#include <stdint.h>
#include <stdbool.h>
#include <status_codes.h>
#include "mem.h"

#define SHA1_BITS         160
#define SHA1_BYTES        20
#define SHA1_BLOCK_BITS   512
#define SHA1_BLOCK_BYTES  64


/* 
 *  This structure will hold context information for the hashing
 *  operation
 */
typedef struct SHA1Context
{
    uint32_t Message_Digest[SHA1_BYTES/4]; /* Message Digest (output)          */
    uint32_t Length_Low;        /* Message length in bits           */
    uint32_t Length_High;       /* Message length in bits           */
    uint8_t Message_Block[SHA1_BLOCK_BYTES]; /* 512-bit message blocks      */
    uint8_t Message_Block_Index;    /* Index into message block array   */
    bool Computed;               /* Is the digest computed?          */
    bool Corrupted;              /* Is the message digest corrupted?  */
} SHA1Context;

/*
 *  Function Prototypes
 */
void SHA1Reset(SHA1Context *);
int8_t SHA1Result(SHA1Context *, uint8_t *dest);
void SHA1Input(SHA1Context *, const uint8_t *, uint16_t);

#endif
