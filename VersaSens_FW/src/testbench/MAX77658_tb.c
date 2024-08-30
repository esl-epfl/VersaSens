#include "../drivers/twim_inst.h"
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

int main(void)
{
    twim_inst_init();
    MAX77658Initialize();
    printk("TWIM BUSY: %d\n", twim_is_busy());
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    MAX77658WriteSequentialRegs(0x00, data, 8);
    printk("TWIM BUSY: %d\n", twim_is_busy());
    printk("Hello, world\n");
    k_sleep(K_MSEC(1000));
    printk("TWIM BUSY: %d\n", twim_is_busy());
    printk("transfer succeeded: %d\n", twim_transfer_succeeded());
    return 0;
}