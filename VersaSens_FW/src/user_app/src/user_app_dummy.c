#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>


LOG_MODULE_REGISTER(user_app_dummy, LOG_LEVEL_INF);


void user_app_dummy_thread(){

    while(1){
        k_sleep(K_MSEC(1000));
        LOG_INF("Hello from the app thread!");
    }
}