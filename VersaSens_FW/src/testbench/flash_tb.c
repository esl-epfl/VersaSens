#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include "../drivers/flash.h"
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_msc.h>

int main(void)
{
    printk("Hello, world\n");
    flash_init();
    uint8_t data_read[100];
    char data_write[100] = "Hello, world\n";
    size_t size = 100;
    char *dir = "Test1";
    k_sleep(K_MSEC(10));
    flash_erase_all();
    k_sleep(K_MSEC(10));
    flash_create_dir(dir);
    lsdir(dir);
    for(int i = 0; i < 100; i++)
    {
        k_sleep(K_MSEC(10));
        versa_flash_write(data_write, size);
    }
    lsdir(dir);
    int ret = usb_enable(NULL);
    if (ret != 0)
    {
        printk("usb_enable failed\n");
    }
    else
    {
        printk("usb_enable success\n");
    }
    return 0;

}