#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrfx_twim.h>
#include <nrfx_gpiote.h>
#include "../drivers/BNO086.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// Function to be called when first work item is processed
void work_handler1(struct k_work *work)
{
    LOG_INF("Work handler 1 is running\n");
}

// Function to be called when second work item is processed
void work_handler2(struct k_work *work)
{
    LOG_INF("Work handler 2 is running\n");
}

// Function to be called when third work item is processed
void work_handler3(struct k_work *work)
{
    LOG_INF("Work handler 3 is running\n");
}

K_WORK_DEFINE(my_work1, work_handler1);
K_WORK_DEFINE(my_work2, work_handler2);
K_WORK_DELAYABLE_DEFINE(my_work3, work_handler3);

int main(void)
{

    bno086_reset();

    
    bno086_init();

    bno086_start_stream();

    LOG_INF("Hello, world!\n");

    while(1){
        k_sleep(K_MSEC(10));
    }

    return 0;
}