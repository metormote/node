/*
 * mem.h
 *
 * Created: 10/13/2011 1:37:46 PM
 *  Author: Administrator
 */ 

#ifndef MEM_H_
#define MEM_H_

#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <interrupt.h>
#include "atom.h"
#include "atommutex.h"
//#include "pmic.h"
#include "conf_board.h"

#ifdef __cplusplus
extern "C" { 
#endif

void mem_init(void);
void *mem_safe_calloc(size_t nele, size_t size);
void *mem_safe_malloc(size_t size);
void mem_safe_free(void *ptr);


#ifdef __cplusplus
}
#endif

#endif /* MEM_H_ */