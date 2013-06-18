/*
 * mem.c
 *
 * Created: 10/26/2011 10:35:16 AM
 *  Author: Administrator
 */ 
#include "mem.h"


static ATOM_MUTEX mem_mutex;

void mem_init(void) {
  atomMutexCreate(&mem_mutex);
}

void *mem_safe_calloc(size_t nele, size_t size) {
  void *ptr;
  //irqflags_t flags=cpu_irq_save();
  size_t len = nele * size;
  //pmic_disable_level(PMIC_LVL_LOW);
  atomMutexGet(&mem_mutex, 0);
  ptr=malloc(len);
  if(ptr>(void *)(STACKEND-len)) {
    free(ptr);
    ptr=NULL;
  }
  atomMutexPut(&mem_mutex);
  //pmic_enable_level(PMIC_LVL_LOW);
  //cpu_irq_restore(flags);
  memset(ptr, 0, len);
  return ptr;
}

void *mem_safe_malloc(size_t size) {
  void *ptr;
  //irqflags_t flags=cpu_irq_save();
  //pmic_disable_level(PMIC_LVL_LOW);
  atomMutexGet(&mem_mutex, 0);
  ptr=malloc(size);
  if(ptr>(void *)(STACKEND-size)) {
    free(ptr);
    ptr=NULL;
  }
  atomMutexPut(&mem_mutex);
  //pmic_enable_level(PMIC_LVL_LOW);
  //cpu_irq_restore(flags);
  return ptr;
}

void mem_safe_free(void *ptr) {
  //irqflags_t flags=cpu_irq_save();
  //pmic_disable_level(PMIC_LVL_LOW);
  atomMutexGet(&mem_mutex, 0);
  free(ptr);
  atomMutexPut(&mem_mutex);
  //pmic_enable_level(PMIC_LVL_LOW);
  //cpu_irq_restore(flags);
  return;
}