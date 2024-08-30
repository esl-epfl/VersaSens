#include "T5838.h"
#include <zephyr/kernel.h>
#include "opus.h"
#include "opus_types.h"
#include "opus_private.h"
#include <zephyr/types.h>
#include "../drivers/flash.h"
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <time.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include "versa_ble.h"
#include "MAX86178.h"



                
int main(void)
{
    // k_sleep(K_MSEC(3500));
    // time_t rawtime;
    // struct tm * timeinfo;
    // int64_t uptime_ms;
    // int64_t time_ms;

    // // Get the current time in seconds
    // time(&rawtime);
    // timeinfo = localtime(&rawtime);

    // // Get the uptime in milliseconds
    // uptime_ms = k_uptime_get();

    // // Calculate the current time in milliseconds
    // time_ms = (rawtime * 1000) + (uptime_ms % 1000);

    // printk("Current local time: %02d:%02d:%02d.%03d\n",
    //        timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, (int)(time_ms % 1000));

    start_ble();
    flash_erase_all();

    k_sleep(K_MSEC(1000));

    flash_init();
    char *dir = "";
    k_sleep(K_MSEC(10));
    k_sleep(K_MSEC(10));
    usb_enable(NULL);

    // // Get a pointer to the CDC ACM device
    // struct device *dev = device_get_binding("CDC_ACM_0");
    // if (!dev) {
    //     // Handle error
    //     printk("Error: CDC ACM device not found\n");
    // }

    // // Write data to the USB interface
    // char data[] = "Hello, USB!";
    // int ret = usb_write(dev, data, sizeof(data), NULL);
    // if (ret != sizeof(data)) {
    //     // Handle error
    //     printk("Error: Failed to write data to USB interface %d\n", ret);
    // }

    // k_sleep(K_MSEC(300000));

    twim_inst_init();
    mlx90632_init();
    k_sleep(K_MSEC(100));
    mlx90632_reset();
    k_sleep(K_MSEC(100));
    mlx90632_config();
    k_sleep(K_MSEC(1000));

    bno086_reset();
    bno086_init();

    flash_open_file(FLASH_CREATE);

    bno086_start_stream();

    t5838_init();
    t5838_start();
    t5838_start_saving();
    k_sleep(K_MSEC(20));
    mlx90632_start_continuous_read();
    printk("Hello, world\n");
    k_sleep(K_MSEC(6000));
    t5838_stop_saving();
    t5838_stop();
    k_sleep(K_MSEC(500));
    bno086_stop_stream();
    k_sleep(K_MSEC(500));
    mlx90632_stop_continuous_read();
    k_sleep(K_MSEC(1000));
    flash_close_file();
    k_sleep(K_MSEC(1000));

    lsdir(dir);

    while(1)
    {
        k_sleep(K_MSEC(1000));
    }

    return 0;
}