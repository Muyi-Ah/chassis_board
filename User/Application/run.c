#include "run.h"
#include "bsp_led.h"
#include "bsp_time.h"

void run(void)
{   
    led_full_toggle();
    Delay_ms(500); // 延时500毫秒
}