/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : versa_api.c                                                     **
** version  : 1                                                            **
** date     : DD/MM/YY                                                     **
**                                                                         **
*****************************************************************************
**                                                                         **
** Copyright (c) EPFL                                                      **
** All rights reserved.                                                    **
**                                                                         **
*****************************************************************************
	 
VERSION HISTORY:
----------------
Version     : 1
Date        : 10/02/2021
Revised by  : Benjamin Duc
Description : Original version.


*/

/***************************************************************************/
/***************************************************************************/

/**
* @file   versa_api.c
* @date   DD/MM/YY
* @brief  This is the main header of versa_api.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _VERSA_API_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

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
#include "versa_config.h"
#include "T5838.h"
#include "MLX90632.h"
#include "MAX86178.h"
#include "sensors_list.h"
#include "versa_time.h"

#include <zephyr/devicetree.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <ff.h>

LOG_MODULE_REGISTER(versa_api, LOG_LEVEL_INF);

/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                        TYPEDEFS AND STRUCTURES                           */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                      PROTOTYPES OF LOCAL FUNCTIONS                       */
/**                                                                        **/
/****************************************************************************/

void LED_thread_func(void *arg1, void *arg2, void *arg3);

void mode_thread_func(void *arg1, void *arg2, void *arg3);

void new_module_thread_func(void *arg1, void *arg2, void *arg3);

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED VARIABLES                             */
/**                                                                        **/
/****************************************************************************/

int vconf_ads1298_en = VCONF_ADS1298_EN;
int vconf_max30001_en = VCONF_MAX30001_EN;
int vconf_bno086_en = VCONF_BNO086_EN;
int vconf_max86178_en = VCONF_MAX86178_EN;
int vconf_mlx90632_en = VCONF_MLX90632_EN;
int vconf_t5838_en = VCONF_T5838_EN;
int vconf_max77658_en = VCONF_MAX77658_EN;

int vconf_ads1298_fs = VCONF_ADS1298_FS;
int vconf_ads1298_gain = VCONF_ADS1298_GAIN;

int vconf_max30001_mode = VCONF_MAX30001_MODE;

/****************************************************************************/
/**                                                                        **/
/*                            GLOBAL VARIABLES                              */
/**                                                                        **/
/****************************************************************************/

/*! Thread stack and instance */
K_THREAD_STACK_DEFINE(LED_thread_stack, 256);
struct k_thread LED_thread;

/*! Thread stack and instance */
K_THREAD_STACK_DEFINE(mode_thread_stack, 2048);
struct k_thread mode_thread;

/*! Thread stack and instance */
K_THREAD_STACK_DEFINE(new_module_thread_stack, 2048);
struct k_thread new_module_thread;

int mode = MODE_IDLE;
K_MUTEX_DEFINE(mode_mutex);

bool sensor_started = false;
bool new_module_thread_stop = false;

bool auto_disconnect = false;

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

int versa_init(void)
{
    //set reset pin to output
    nrf_gpio_cfg_output(RST_N_PIN);
    //set reset pin to low
    nrf_gpio_pin_clear(RST_N_PIN);
    k_sleep(K_MSEC(100));
    //set reset pin to high
    nrf_gpio_pin_set(RST_N_PIN);
    k_sleep(K_MSEC(100));

    int ret;

    start_ble();
    twim_inst_init();
    MAX77658_init();
    storage_init();
    k_sleep(K_MSEC(500));
    // erase_file();
    k_sleep(K_MSEC(500));

    nrf_gpio_cfg_output(START_PIN);
    nrf_gpio_pin_clear(START_PIN);



    if(vconf_max30001_en)
    {
        ret = sensors_list[MAX30001_ID].init();
        if (ret == -2 & auto_disconnect)
        {
            vconf_max30001_en = 0;
        }
    }

    k_sleep(K_MSEC(50));

    if(vconf_max86178_en)
    {
        ret = sensors_list[MAX86178_ID].init();
        if (ret == -2 & auto_disconnect)
        {
            vconf_max86178_en = 0;
            vconf_t5838_en = 0;
            vconf_mlx90632_en = 0;
        }
    }

    k_sleep(K_MSEC(50));

    if(vconf_bno086_en)
    {
        sensors_list[BNO086_ID].init();
    }

    k_sleep(K_MSEC(50));

    if(vconf_t5838_en)
    {
        sensors_list[T5838_ID].init();
    }
    
    k_sleep(K_MSEC(50));

    if(vconf_ads1298_en)
    {
        ret = sensors_list[ADS1298_ID].init();
        if (ret == -2 & auto_disconnect)
        {
            vconf_ads1298_en = 0;
        }
    }

    k_sleep(K_MSEC(50));

    if(vconf_mlx90632_en)
    {
        sensors_list[MLX90632_ID].init();
    }

    LOG_INF("Versa API sensors initialized\n");
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int enable_auto_connect(void)
{
    new_module_thread_stop = false;
    auto_disconnect = true;
    k_tid_t new_module_thread_id = k_thread_create(&new_module_thread, new_module_thread_stack, K_THREAD_STACK_SIZEOF(new_module_thread_stack),
                                             new_module_thread_func, NULL, NULL, NULL, -1, 0, K_NO_WAIT);
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int disable_auto_connect(void)
{
    new_module_thread_stop = true;
    auto_disconnect = false;
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int versa_config(void)
{
    if(vconf_ads1298_en)
    {
        if (sensors_list[ADS1298_ID].config != NULL) {
            sensors_list[ADS1298_ID].config();
        }
    }

    k_sleep(K_MSEC(50));

    if(vconf_max30001_en)
    {
        if (sensors_list[MAX30001_ID].config != NULL) {
            sensors_list[MAX30001_ID].config();
        }
    }

    k_sleep(K_MSEC(50));

    if(vconf_mlx90632_en)
    {
        if (sensors_list[MLX90632_ID].config != NULL) {
            sensors_list[MLX90632_ID].config();
        }
    }

    k_sleep(K_MSEC(50));

    if(vconf_max86178_en)
    {
        if (sensors_list[MAX86178_ID].config != NULL) {
            sensors_list[MAX86178_ID].config();
        }
    }

    LOG_INF("Versa API sensors configured\n");
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int versa_sensor_start(void)
{
    storage_open_file(STORAGE_CREATE);

    nrf_gpio_pin_set(START_PIN);

    sync_time();

    k_sleep(K_MSEC(10));

    if(vconf_max77658_en)
    {
        MAX77658_start_continuous_read();
    }

    if(vconf_ads1298_en)
    {
        sensors_list[ADS1298_ID].start_continuous();
    }

    if(vconf_max30001_en)
    {
        sensors_list[MAX30001_ID].start_continuous();
    }

    if(vconf_bno086_en)
    {
        sensors_list[BNO086_ID].start_continuous();
    }

    if(vconf_t5838_en)
    {
        sensors_list[T5838_ID].start_continuous();
    }

    if(vconf_mlx90632_en)
    {
        sensors_list[MLX90632_ID].start_continuous();
    }

    if(vconf_max86178_en)
    {
        sensors_list[MAX86178_ID].start_continuous();
    }

    LOG_INF("Versa API sensors started\n");
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int versa_sensor_stop(void)
{
    nrf_gpio_pin_clear(START_PIN);

    if(vconf_max77658_en)
    {
        MAX77658_stop_continuous_read();
    }

    if(vconf_ads1298_en)
    {
        sensors_list[ADS1298_ID].stop_continuous();
    }

    if(vconf_max30001_en)
    {
        sensors_list[MAX30001_ID].stop_continuous();
    }

    if(vconf_bno086_en)
    {
        sensors_list[BNO086_ID].stop_continuous();
    }

    if(vconf_t5838_en)
    {
        sensors_list[T5838_ID].stop_continuous();
    }

    if(vconf_mlx90632_en)
    {
        sensors_list[MLX90632_ID].stop_continuous();
    }

    if(vconf_max86178_en)
    {
        sensors_list[MAX86178_ID].stop_continuous();
    }

    storage_close_file();

    LOG_INF("Versa API sensors stopped\n");
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int versa_start_led_thread(void)
{
    k_tid_t LED_thread_id = k_thread_create(&LED_thread, LED_thread_stack, K_THREAD_STACK_SIZEOF(LED_thread_stack),
                                           LED_thread_func, NULL, NULL, NULL, 2, 0, K_NO_WAIT);

    LOG_INF("Versa API LED thread started\n");
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int versa_start_mode_thread(void)
{
    k_tid_t mode_thread_id = k_thread_create(&mode_thread, mode_thread_stack, K_THREAD_STACK_SIZEOF(mode_thread_stack),
                                             mode_thread_func, NULL, NULL, NULL, 1, 0, K_NO_WAIT);
    return 0;
}

/****************************************************************************/
/****************************************************************************/

void versa_set_mode(int new_mode)
{
    k_mutex_lock(&mode_mutex, K_FOREVER);
    mode = new_mode;
    k_mutex_unlock(&mode_mutex);
}

/****************************************************************************/
/****************************************************************************/

int versa_get_mode(void)
{
    k_mutex_lock(&mode_mutex, K_FOREVER);
    int current_mode = mode;
    k_mutex_unlock(&mode_mutex);
    return current_mode;
}

/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

void LED_thread_func(void *arg1, void *arg2, void *arg3)
{
    nrf_gpio_cfg_output(GREEN_LED_PIN);
    nrf_gpio_pin_clear(GREEN_LED_PIN);

    nrf_gpio_cfg_output(RED_LED_PIN);
    nrf_gpio_pin_clear(RED_LED_PIN);

    nrf_gpio_cfg_output(YELLOW_LED_PIN);
    nrf_gpio_pin_clear(YELLOW_LED_PIN);

    while (1)
    {
        if (get_temperature_high())
        {
            nrf_gpio_pin_set(GREEN_LED_PIN);
            nrf_gpio_pin_clear(RED_LED_PIN);
            nrf_gpio_pin_set(YELLOW_LED_PIN);
        }
        else if (get_write_failed())
        {
            nrf_gpio_pin_set(GREEN_LED_PIN);
            nrf_gpio_pin_set(RED_LED_PIN);
            nrf_gpio_pin_set(YELLOW_LED_PIN);
        }
        else if (versa_get_mode() == MODE_STORE)
        {
            nrf_gpio_pin_set(GREEN_LED_PIN);
            nrf_gpio_pin_toggle(RED_LED_PIN);
            nrf_gpio_pin_set(YELLOW_LED_PIN);
        }
        else if (versa_get_mode() == MODE_STREAM)
        {
            nrf_gpio_pin_set(GREEN_LED_PIN);
            nrf_gpio_pin_set(RED_LED_PIN);
            nrf_gpio_pin_toggle(YELLOW_LED_PIN);
        }
        else if (get_battery_charging())
        {
            nrf_gpio_pin_toggle(GREEN_LED_PIN);
            nrf_gpio_pin_set(RED_LED_PIN);
            nrf_gpio_pin_set(YELLOW_LED_PIN);
        }
        else if (get_battery_low())
        {
            nrf_gpio_pin_set(GREEN_LED_PIN);
            nrf_gpio_pin_set(RED_LED_PIN);
            nrf_gpio_pin_clear(YELLOW_LED_PIN);
        }
        else if (versa_get_mode() == MODE_IDLE)
        {
            nrf_gpio_pin_clear(GREEN_LED_PIN);
            nrf_gpio_pin_set(RED_LED_PIN);
            nrf_gpio_pin_set(YELLOW_LED_PIN);
        }


        k_sleep(K_MSEC(500));
    }
}

/****************************************************************************/
/****************************************************************************/

void mode_thread_func(void *arg1, void *arg2, void *arg3)
{
    nrf_gpio_cfg_input(MODE_IDLE_PIN, GPIO_PIN_CNF_PULL_Pulldown);
    nrf_gpio_cfg_input(MODE_STORE_PIN, GPIO_PIN_CNF_PULL_Pulldown);
    nrf_gpio_cfg_input(MODE_STREAM_PIN, GPIO_PIN_CNF_PULL_Pulldown);

    while (1)
    {
        
        if (nrf_gpio_pin_read(MODE_IDLE_PIN) > 0 | (nrf_gpio_pin_read(MODE_STORE_PIN)==0 & nrf_gpio_pin_read(MODE_STREAM_PIN)==0))
        {
            disable_stream_data();
            if(sensor_started)
            {
                sensor_started = false;
                versa_sensor_stop();
            }
            versa_set_mode(MODE_IDLE);
        }
        else if (nrf_gpio_pin_read(MODE_STORE_PIN) > 0 | (nrf_gpio_pin_read(MODE_IDLE_PIN)==0 & nrf_gpio_pin_read(MODE_STREAM_PIN)==0))
        {
            disable_stream_data();
            if(!sensor_started)
            {
                sensor_started = true;
                versa_sensor_start();
            }
            versa_set_mode(MODE_STORE);
        }
        else if (nrf_gpio_pin_read(MODE_STREAM_PIN) > 0 | (nrf_gpio_pin_read(MODE_IDLE_PIN)==0 & nrf_gpio_pin_read(MODE_STORE_PIN)==0))
        {
            enable_stream_data();
            if(!sensor_started)
            {
                sensor_started = true;
                versa_sensor_start();
            }
            versa_set_mode(MODE_STREAM);
        }

        k_sleep(K_MSEC(200));
    }
}

/****************************************************************************/
/****************************************************************************/

void new_module_thread_func(void *arg1, void *arg2, void *arg3){
    int ret;
    while(!new_module_thread_stop){
        k_sleep(K_MSEC(3000));
        // check for each inactive sensor if it is enabled
        if(!vconf_ads1298_en){
            ret = sensors_list[ADS1298_ID].init();
            if (ret != -2)
            {
                vconf_ads1298_en = 1;
                sensors_list[ADS1298_ID].config();
            }
        }
        if(!vconf_max30001_en){
            ret = sensors_list[MAX30001_ID].init();
            if (ret != -2)
            {
                vconf_max30001_en = 1;
                sensors_list[MAX30001_ID].config();
            }
        }
        if(!vconf_max86178_en){
            ret = sensors_list[MAX86178_ID].init();
            if (ret != -2)
            {
                vconf_max86178_en = 1;
                sensors_list[MAX86178_ID].config();

                vconf_t5838_en = 1;
                sensors_list[T5838_ID].init();

                vconf_mlx90632_en = 1;
                sensors_list[MLX90632_ID].init();
                sensors_list[MLX90632_ID].config();
            }
        }
        if(!vconf_bno086_en){
            vconf_bno086_en = 1;
            sensors_list[BNO086_ID].init();
        }
        if(!vconf_max77658_en){
            vconf_max77658_en = 1;
        }
    }
}

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/