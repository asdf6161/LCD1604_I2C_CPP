// copy from https://hubstub.ru/stm32/101-funkciya-zaderzhki-stm32.html

#ifdef STM32F303xC
//#include "stm32f3xx_ll_utils.h"
#include "stm32f303xc.h"
#endif

/* HOW TO USE
 *
 * Use DWT_Init in main at start program
 * After you can use delay's
 * */

void DWT_Init(void);

static __inline uint32_t delta(uint32_t t0, uint32_t t1)
{
    return (t1 - t0);
}

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
