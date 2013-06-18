/*
 * crypto.c
 *
 * Created: 9/2/2011 11:02:58 AM
 *  Author: Administrator
 */ 
#include "crypto.h"

static uint8_t *key;

/**
 * \brief Key used when the AES shall decrypt.
 *
 * This last subkey is a modified version of the key.
 */
static uint8_t lastsubkey[AES_DATA_SIZE];

/* Semaphore used for crypto finished notifications*/
//static ATOM_SEM aes_sem;

/* Mutex used to protect from concurrent access to crypto module*/
static ATOM_MUTEX aes_mutex;

//forward declarations
static void aes_isr_handler(void);
static bool aes_lastsubkey_generate(t_key key, t_key last_sub_key);



void crypto_init(uint8_t *k) {
  key=k;
  
  //create semaphore used for crypto notifications
  //atomSemCreate(&aes_sem, 0);
  
  //create mutex
  atomMutexCreate(&aes_mutex);
  
  aes_lastsubkey_generate(key, lastsubkey);
    
  /* Set AES interrupt callback function. */
  aes_set_callback(aes_isr_handler);

}

void crypto_init_iv(uint8_t *iv) {
  uint8_t i;
  uint32_t r;
  
  for(i=0;i<4;i++) {
    r=random();
    iv[4*i]=(uint8_t)r;
    iv[4*i+1]=(uint8_t)(r >> 8);
    iv[4*i+2]=(uint8_t)(r >> 16);
    iv[4*i+3]=(uint8_t)(r >> 24);
  }
}

  
int8_t crypto_start_encrypt(uint8_t *iv) {
  
  if(atomMutexGet(&aes_mutex, CRYPTO_MUTEX_TIMEOUT)!=ATOM_OK) {
    return ERR_BUSY;
  }
  
  /* Before using the AES it is recommended to do an AES software reset to
    * put the module in known state, in case other parts of the code has
    * accessed the AES module. */
  aes_software_reset();

  /* Load initial vector into AES state memory. */
  aes_write_inputdata(iv);
    
  /* Set AES encryption of a single block in auto mode. */
  aes_configure(AES_ENCRYPT, AES_AUTO, AES_XOR_ON);
    
  //reset the semaphore to zero
  //atomSemResetCount(&aes_sem, 0);
  
  return STATUS_OK;
}


//encrypt a block of 16 bytes
//len must be <= 16
int8_t crypto_encrypt_block(uint8_t *p, uint8_t len) {
  uint8_t k;
  
  if(len<AES_DATA_SIZE) {
    //add the PKCS5Padding
    p+=len;
    for(k=len;k<AES_DATA_SIZE;k++) {
      *p++=AES_DATA_SIZE-len;
    }
    p-=AES_DATA_SIZE;
  }
      
  /* Load key into AES key memory. */
  aes_set_key(key);

  /* Load data into AES state memory. */
  aes_write_inputdata(p);
        
  /* Enable the AES low level interrupt. 
   * Note that this must be set to low level interrupt to avoid 
   * possible deadlock situations. */
  //aes_isr_configure(AES_INTLVL_LO);
  
  //wait on semaphore until encryption finished
  //if(atomSemGet(&aes_sem, CRYPTO_TIMEOUT)!=ATOM_OK) return ERR_TIMEOUT;
  
  do {
    /* Wait until AES is finished or an error occurs. */
  } while ((AES.STATUS & (AES_SRIF_bm | AES_ERROR_bm) ) == 0);

  /* If not error. */
  if ((AES.STATUS & AES_ERROR_bm) == 0) {
    AES.STATUS = AES_SRIF_bm;
  } else {
    AES.STATUS = AES_ERROR_bm;
    return ERR_IO_ERROR;
  }
  
  aes_read_outputdata(p);
  return STATUS_OK;
}

/* 
 * Encrypt data buffer
 * The buffer will contain the encrypted data on return
 */
int8_t crypto_encrypt(uint8_t *buf, uint16_t *len) {
  uint8_t block_count, block_len;
  uint8_t *p=buf;
  uint16_t l=0;
  uint8_t lmod=(uint8_t)(*len % AES_DATA_SIZE);
  
  for(block_count=(*len)/AES_DATA_SIZE+1;block_count!=0;block_count--) {
    block_len=(uint8_t)(block_count>1?AES_DATA_SIZE:lmod);
    
    //encrypt
    if(crypto_encrypt_block(p, block_len)!=STATUS_OK) {
      return ERR_TIMEOUT;
    }   
    l+=AES_DATA_SIZE;
    p+=AES_DATA_SIZE;
  }
  *len=l;
  return STATUS_OK;
}


int8_t crypto_start_decrypt() {
  
  if(atomMutexGet(&aes_mutex, CRYPTO_MUTEX_TIMEOUT)!=ATOM_OK) {
    return ERR_BUSY;
  }
  
  /* Before using the AES it is recommended to do an AES software reset to
    * put the module in known state, in case other parts of the code has
    * accessed the AES module. */
  aes_software_reset();

  //reset the semaphore to zero
  //atomSemResetCount(&aes_sem, 0);
  
  return STATUS_OK;
}

    
int8_t crypto_decrypt_block(uint8_t *block, uint8_t *iv, uint8_t *len, bool last) {
  uint8_t i,j;
  uint8_t *p;

  /* Set AES decryption of a single block in auto mode. */
  aes_configure(AES_DECRYPT, AES_AUTO, AES_XOR_OFF);

  /* Load key into AES key memory. */
  aes_set_key(lastsubkey);

  /* Load data into AES state memory. */
  aes_write_inputdata(block);
  
  /* Enable the AES low level interrupt. */
  //aes_isr_configure(AES_INTLVL_LO);
  
  //wait on semaphore until decryption finishes
  //if(atomSemGet(&aes_sem, CRYPTO_TIMEOUT)!=ATOM_OK) 
  //  return ERR_TIMEOUT;
  
  do {
    /* Wait until AES is finished or an error occurs. */
  } while ((AES.STATUS & (AES_SRIF_bm | AES_ERROR_bm) ) == 0);

  /* If not error. */
  if ((AES.STATUS & AES_ERROR_bm) == 0) {
    AES.STATUS = AES_SRIF_bm;
  } else {
    AES.STATUS = AES_ERROR_bm;
    return ERR_IO_ERROR;
  }
          
  /* When CBC is used the answer must be xored with the previous cipher text
    * or the initialization vector to reconstruct the plaintext. */
  /* Set AES decryption of a single block in manual mode. */
  aes_configure(AES_DECRYPT, AES_MANUAL, AES_XOR_ON);
  aes_write_inputdata(iv);
  
  //save the cryptotext for next round CBC
  p=block;
  for(j=0;j<AES_DATA_SIZE;j++) {
    iv[j]=*p;
    p++;
  }
  
  //read out the result
  aes_read_outputdata(block);
  
  //if last crypto block then remove PKCS5Padding
  //p is pointing after the last element of the buffer
  if(last) {
    p--;
    j=*p;
    if(j<=AES_DATA_SIZE) {
      for(i=0;i<j;i++) {
        *p--='\0';
      }
      *len=AES_DATA_SIZE-j;
      return STATUS_OK;
    }
  }
  
  *len=AES_DATA_SIZE;
  return STATUS_OK;
}


/* 
 * Decrypt data buffer
 * The buffer will contain the decrypted data on return.
 * The len parameter will be modified to contain the true length of the decrypted data.
 * Note that the IV vector will be modified.
 */
int8_t crypto_decrypt(uint8_t *buf, uint8_t *iv, uint16_t *len) {
  uint8_t block_count, block_len;
  uint8_t *p=buf;
  uint16_t l=0;
  
  for(block_count=(*len)/AES_DATA_SIZE;block_count!=0;block_count--) {
    //decrypt block
    if(crypto_decrypt_block(p, iv, &block_len, block_count==1)!=STATUS_OK) {
      return ERR_TIMEOUT;
    }   
    l+=block_len;
    p+=AES_DATA_SIZE;
  }
  //return the true data length
  *len=l;
  
  return STATUS_OK;
}

/* Release the mutex */
void crypto_final() {
  atomMutexPut(&aes_mutex);
}


/**
 * \b aes_isr_handler
 *
 * Callback for crypto module.
 *
 * @return None
 */
static void aes_isr_handler(void) {
    atomIntEnter();
  
    /* Disable the AES interrupt. */
    aes_isr_configure(AES_INTLVL_OFF);
  
    //atomSemPut(&aes_sem);
  
    atomIntExit(FALSE, TRUE);
}

/**
 * \brief Generate AES sub key
 *
 * \note Get AES sub key by encryption of dummy data.
 *
 * \param key           Pointer to AES key input.
 * \param last_sub_key  Pointer to AES sub key output.
 *
 */
static bool aes_lastsubkey_generate(t_key key, t_key last_sub_key)
{
    bool keygen_ok;
    uint8_t i;
    uint8_t * temp_last_sub_key;
    
    /* Before using the AES it is recommended to do an AES software reset to
     * put the module in known state, in case other parts of your code has
     * accessed the AES module. */
    aes_software_reset();

    /* Set AES encryption of a single block in manual mode. */
    aes_configure(AES_ENCRYPT, AES_MANUAL, AES_XOR_OFF);

    /* Load key into AES key memory. */
    aes_set_key(key);

    /* Load dummy data into AES state memory. */
    for (i = 0; i < AES_DATA_SIZE; i++) {
        AES.STATE = 0x00;
    }

    /* Start encryption. */
    aes_start();

    do {
        /* Wait until AES is finished or an error occurs. */
    } while ((AES.STATUS & (AES_SRIF_bm | AES_ERROR_bm) ) == 0);

    /* If not error. */
    if ((AES.STATUS & AES_ERROR_bm) == 0) {
        /* Store the last subkey. */
        temp_last_sub_key = last_sub_key;
        for (i = 0; i < AES_DATA_SIZE; i++) {
            *(temp_last_sub_key++) = AES.KEY;
        }
        AES.STATUS = AES_SRIF_bm;
        keygen_ok = true;
    } else {
        AES.STATUS = AES_ERROR_bm;
        keygen_ok = false;
    }
    return keygen_ok;
}
