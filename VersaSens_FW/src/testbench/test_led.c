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

uint16_t data_read[1];

#include <zephyr/devicetree.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <ff.h>


char data_write_FS[12] = "Hello World!";

int main(void)
{
    versa_init();
    // enable_auto_connect();
    versa_config();

    versa_start_led_thread();
    versa_start_mode_thread();

    // nrf_gpio_cfg_output(45);
    // nrf_gpio_pin_clear(45);

    // start_ble();
    // k_sleep(K_MSEC(1000));
    // printk("BLE started\n");
    // twim_inst_init();
    // k_sleep(K_MSEC(1000));
    // printk("TWIM initialized\n");
    // MAX77658_init();

    // storage_init();
    // k_sleep(K_MSEC(500));
    // erase_file();
    // k_sleep(K_MSEC(500));

    // // usb_enable(NULL);
    
    // bno086_reset();
    // bno086_init();

    // k_sleep(K_MSEC(1000));

    // MAX77658_write_8bit(REG_CNFG_SBB1_A_ADDR, 0x34);  // 1.8V
    // MAX77658_write_8bit(REG_CNFG_SBB1_B_ADDR, 0b00000100);
    // MAX77658_write_8bit(REG_CNFG_SBB2_B_ADDR, 0b00000000);
    // MAX77658_write_8bit(REG_CNFG_SBB2_A_ADDR, 0x70);  // 3.3V

    // ADS1298_init();
    // k_sleep(K_MSEC(100));
    // ADS1298_config();

    // storage_open_file(STORAGE_CREATE);
    // ADS1298_init_interrupt();
    // nrf_gpio_cfg_output(45);
    // nrf_gpio_pin_set(45);
    // ADS1298_start_thread();
    // bno086_start_stream();

    // versa_sensor_start();

    // k_sleep(K_SECONDS(30));

    // versa_sensor_stop();    

    // storage_close_file();
    // ADS1298_stop_thread();
    // bno086_stop_stream();

    // MAX30001_init();
    // k_sleep(K_MSEC(100));

    // storage_open_file(STORAGE_CREATE);
    // MAX30001_start_thread();

    // k_sleep(K_SECONDS(30));

    // MAX30001_stop_thread();
    // storage_close_file();

    data_read[0] = 0x0000;

    while (1)
    {
        k_sleep(K_MSEC(1000));
        // MAX77658_read_16bit(REG_RepSOC_ADDR, data_read, 1);
        // MAX77658_read_16bit(REG_Temp_ADDR, data_read, 1);
        // MAX77658_read_16bit(REG_Config_ADDR, data_read, 1);
        // data_read[0] = 0xFFFF;
        // MAX77658_read_8bit(REG_CNFG_SBB1_A_ADDR, data_read);
        // k_sleep(K_MSEC(100));
        // set_battery_data(data_read);

        // k_sleep(K_MSEC(1000));
        // data_read[0] = 0xFFFF;
        // MAX77658_read_16bit(REG_RepSOC_ADDR, data_read, 1);
        // set_battery_data(data_read); 
        // MAX77658_read_16bit(REG_Temp_ADDR, data_read, 1);
        // receive_sensor_data(data_read, 2);
    }
}
