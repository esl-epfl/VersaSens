#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/usb/usb_device.h>
#include <time.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <nrfx_gpiote.h>
#include "MAX77658.h"
#include "versa_ble.h"
#include "twim_inst.h"
#include "storage.h"
#include "ADS1298.h"
#include "MAX30001.h"
#include "versa_api.h"
#include <start_app_api.h>

#include <zephyr/devicetree.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <ff.h>


/* Include the user app here */
#include "user_app_dummy.h"


int main(void)
{
    versa_init();
    // enable_auto_connect();
    versa_config();

    versa_start_led_thread();
    versa_start_mode_thread();



    /* Call the initialization of the user application thread and pass the user thread */
    init_user_app_thread(user_app_dummy_thread, 12);


    while (1)
    {
        k_sleep(K_MSEC(1000));
    }
}
