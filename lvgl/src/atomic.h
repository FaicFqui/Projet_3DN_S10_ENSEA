#ifndef LV_ATOMIC_H
#define LV_ATOMIC_H

#include <stdint.h>

#define ATOMIC_COMPARE_AND_SWAP_SUCCESS 1

// Fonction factice : simule un CAS toujours réussi
static inline uint32_t Atomic_CompareAndSwap_u32(volatile uint32_t *ptr,
                                                 uint32_t desired,
                                                 uint32_t expected)
{
    *ptr = desired;
    return ATOMIC_COMPARE_AND_SWAP_SUCCESS;
}

// Fonction factice : incrément simple
static inline uint32_t Atomic_Increment_u32(volatile uint32_t *ptr)
{
    return (*ptr)++;
}

#endif // LV_ATOMIC_H
