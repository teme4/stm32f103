/**************************************************************************************************
 * @file sync.hpp
 * @author Yakovlev Vladislav (y.yakovlev@quanttelecom.ru)
 * @brief Реализует примитивы синхронизации, как описано в документе ARM.
 * DHT0008A (ID081709) «Примитивы синхронизации ARM» и ARM v7-M
 * Справочное руководство по архитектуре.
 * @version 1.0
 * @date 09.07.2023
 *
 * @copyright Copyright (c) 2023
 *
 *************************************************************************************************/
#ifndef __CORE_CM3_SYNC_HPP_
#define __CORE_CM3_SYNC_HPP_

#include "stm32f1xx.h"

/* DMB is supported on CM0 */
 void __dmb();

/**************************************************************************************************
 * @brief Эксклюзивные инструкции по загрузке и хранению, т.е определяются только на
 *CM3 или CM4
 ***************************************************************************************************/
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
#define MUTEX_LOCKED   1
#define MUTEX_UNLOCKED 0

using mutex_t = uint32_t;


inline uint32_t __ldrex(volatile uint32_t *addr) __attribute__((always_inline));
inline uint32_t __strex(uint32_t val, volatile uint32_t *addr) __attribute__((always_inline));

/*---Удобные функции --------------------------------------------------------------*/
/*Здесь мы реализуем несколько простых примитивов синхронизации.*/


void mutex_lock(mutex_t *m);//  __attribute__((always_inline));
void mutex_unlock(mutex_t *m);//  __attribute__((always_inline));
mutex_t mutex_trylock(mutex_t *m);//  __attribute__((always_inline));


#endif

#endif//__CORE_CM3_SYNC_HPP_
