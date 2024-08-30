#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

K_THREAD_STACK_DEFINE(my_thread_stack, 1024);
struct k_thread my_thread;
volatile bool stop_thread = false;

void my_thread_func(void *arg1, void *arg2, void *arg3)
{
    while (!stop_thread)
    {
        printk("Thread is running...\n");
        k_sleep(K_MSEC(1000));
    }
}

void start_thread()
{
    printk("Starting thread...\n");
    stop_thread = false;
    k_thread_create(&my_thread, my_thread_stack, K_THREAD_STACK_SIZEOF(my_thread_stack),
                    my_thread_func, NULL, NULL, NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
}

void stop_thread_func()
{
    printk("Stopping thread...\n");
    stop_thread = true;
    k_thread_abort(&my_thread);
}

int main(void)
{
    start_thread();
    k_sleep(K_SECONDS(5)); // Let the thread run for 5 seconds
    stop_thread_func();
    return 0;
}