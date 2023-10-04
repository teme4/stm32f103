#include "sync.hpp"

void __dmb()
{
	__asm__ volatile ("dmb");
}

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)

uint32_t __ldrex(volatile uint32_t *addr)
{
    uint32_t res;
    __asm__ volatile("ldrex %0, [%1]" : "=r"(res) : "r"(addr));
    return res;
}

uint32_t __strex(uint32_t val, volatile uint32_t *addr)
{
    uint32_t res;
    __asm__ volatile("strex %0, %2, [%1]" : "=&r"(res) : "r"(addr), "r"(val));
    return res;
}

/*---Удобные функции --------------------------------------------------------------*/
/*Здесь мы реализуем несколько простых примитивов синхронизации.*/


void mutex_lock(mutex_t *m)
{
    uint32_t status{0};
    do
    {
        /*ждем, пока мьютекс не будет разблокирован.*/
        while (__ldrex(m) != MUTEX_UNLOCKED) {}

        /*Пытаемся получить его.*/
        status = __strex(MUTEX_LOCKED, m);

        /* Did we get it? If not then try again. */
    } while (status != 0);

    /*Выполнить инструкцию Data Memory Barrier!*/
    __dmb();
}

void mutex_unlock(mutex_t *m)
{
    /*Убедитесь, что доступ к защищенному ресурсу завершен*/
    __dmb();

    /*Освобождаем блокировку.*/
    *m = MUTEX_UNLOCKED;
}

/**************************************************************************************************
 * @brief Попытаться заблокировать мьютекс
 * @param [inout] m
 * @return MUTEX_LOCKED уже заблокирован или в STREX произошла ошибка
 * @return MUTEX_UNLOCKED свободен
 *************************************************************************************************/
mutex_t mutex_trylock(mutex_t *m)
{
	uint32_t status = 0;
	mutex_t old_lock = __ldrex(m); // get mutex value
	// set mutex
	status = __strex(MUTEX_LOCKED, m);
	if(status == 0) __dmb();
	else old_lock = MUTEX_LOCKED;
	return old_lock;
}

#endif
