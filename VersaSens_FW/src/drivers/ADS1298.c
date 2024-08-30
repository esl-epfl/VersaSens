/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : ADS1298.c                                                   **
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
* @file   ADS1298.c
* @date   DD/MM/YY
* @brief  This is the main header of ADS1298.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _ADS1298_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include <stdlib.h>
#include "ADS1298.h"
#include <nrfx_spim.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrfx_gpiote.h>
#include "versa_time.h"
#include "versa_ble.h"
#include "storage.h"
#include "versa_config.h"
#include "spim_inst.h"

/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(ADS1298, LOG_LEVEL_INF);

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

/**
 * @brief Function to handle the ADS1298 thread
 * 
 * @param arg1 A pointer to the first argument passed to the thread.
 * @param arg2 A pointer to the second argument passed to the thread.
 * @param arg3 A pointer to the third argument passed to the thread.
 */
void ADS1298_thread_func(void *arg1, void *arg2, void *arg3);

/**
 * @brief Function to reconfigure the SPIM instance for the ADS1298
 * 
 * @return int 0 if successful, -1 otherwise
 */
int ADS1298_reconfig(void);

/**
 * @brief Function to handle the interrupt from the ADS1298
 * 
 * @param pin The pin that triggered the interrupt
 * @param trigger The trigger that triggered the interrupt
 * @param context The context of the interrupt
 */
void ADS1298_interrupt_handler(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void *context);

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED VARIABLES                             */
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/*                            GLOBAL VARIABLES                              */
/**                                                                        **/
/****************************************************************************/

/*! spim instance */
static nrfx_spim_t ads_spim_inst = NRFX_SPIM_INSTANCE(ADS_INST_IDX);

/*! Buffer for the SPI transfer */
uint8_t ads_tx_buf[20];

/*! Flag to check if the last transfer was successful */
bool ads_last_transfer_succeeded = false;

/*! Semaphore to notify the thread that new data is ready */
K_SEM_DEFINE(new_data_rdy, 0, 1);

/*! Thread stack and instance */
K_THREAD_STACK_DEFINE(ADS1298_thread_stack, 1024);
struct k_thread ADS1298_thread;

/*! Flag to stop the thread */
volatile bool ADS1298_stop_thread_flag = false;

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

int ADS1298_init(void)
{
    int ret_val = 0;

    // Initialize the SPIM instance
    nrfx_spim_config_t ads_spim_config = NRFX_SPIM_DEFAULT_CONFIG(ADS_SCK_PIN, ADS_MOSI_PIN, ADS_MISO_PIN, ADS_SS_PIN);
    ads_spim_config.mode = NRF_SPIM_MODE_1;
    ads_spim_config.frequency = NRFX_MHZ_TO_HZ(4);

    nrfx_err_t err_code = nrfx_spim_init(&ads_spim_inst, &ads_spim_config, NULL, NULL);
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("twim_init failed with error code: %d\n", err_code);
        ret_val = -1;
    }

    k_sleep(K_MSEC(100));

    // Reset the ADS1298
    ADS1298_send_cmd(RESET_CMD);
    k_sleep(K_MSEC(10));
    ADS1298_send_cmd(START_CMD);
    k_sleep(K_MSEC(10));
    ADS1298_send_cmd(WAKEUP_CMD);
    k_sleep(K_MSEC(10));

    // stop the continuous data transfer
    ADS1298_send_cmd(SDATAC_CMD);
    k_sleep(K_MSEC(300));

    // check if the ADS1298 is present
    if(ADS1298_check_present() != 0)
    {
        return -2;
    }

    ADS1298_init_interrupt();

    LOG_INF("ADS1298_init");
    return ret_val;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_check_present(void)
{
    uint8_t data[3];
    data[2] = 0;

    // Check if the ADS1298 is present by reading the ID register
    int err_code = ADS1298_read_reg(REG_ID_Addr, data);
    if (err_code != 0)
    {
        LOG_ERR("ADS1298_read_reg failed with error code: %d\n", err_code);
        return -1;
    }

    if (data[2] != 0x92)
    {
        LOG_ERR("ADS1298 not present\n");
        return -1;
    }

    LOG_INF("ADS1298 present");
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_send_cmd(uint8_t cmd)
{
    k_sem_take(&spi_sem, K_FOREVER);
    int err_code;

    // set the transfer descriptor
    ads_tx_buf[0] = cmd;
    nrfx_spim_xfer_desc_t spi_desc = NRFX_SPIM_XFER_TX(ads_tx_buf, 1);

    // reconfigure the SPIM instance
    ADS1298_reconfig();

    err_code = nrfx_spim_xfer(&ads_spim_inst, &spi_desc, 0);
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("nrfx_spim_xfer failed");
        return err_code;
    }

    k_sem_give(&spi_sem);

    return 0;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_read_reg(uint8_t addr, uint8_t *data)
{
    k_sem_take(&spi_sem, K_FOREVER);
    int err_code;

    // set the transfer descriptor
    ads_tx_buf[0] = addr | 0x20;
    ads_tx_buf[1] = 0x00;
    ads_tx_buf[2] = 0x00;
    nrfx_spim_xfer_desc_t spi_config = NRFX_SPIM_XFER_TRX(ads_tx_buf, 3, data, 3);

    // reconfigure the SPIM instance
    ADS1298_reconfig();

    err_code = nrfx_spim_xfer(&ads_spim_inst, &spi_config, 0);
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("nrfx_spim_xfer failed");
        return err_code;
    }

    k_sem_give(&spi_sem);

    return 0;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_write_reg(uint8_t addr, uint8_t data)
{
    k_sem_take(&spi_sem, K_FOREVER);
    int err_code;

    // set the transfer descriptor
    ads_tx_buf[0] = addr | 0x40;
    ads_tx_buf[1] = 0x00;
    ads_tx_buf[2] = data;
    nrfx_spim_xfer_desc_t spi_config = NRFX_SPIM_XFER_TX(ads_tx_buf, 3);

    // reconfigure the SPIM instance
    ADS1298_reconfig();

    err_code = nrfx_spim_xfer(&ads_spim_inst, &spi_config, 0);
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("nrfx_spim_xfer failed");
        return err_code;
    }

    k_sem_give(&spi_sem);

    return 0;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_read_Nreg(uint8_t addr, uint8_t *data, uint8_t num_reg)
{
    k_sem_take(&spi_sem, K_FOREVER);
    int err_code;

    // set the transfer descriptor
    ads_tx_buf[0] = addr | 0x20;
    ads_tx_buf[1] = num_reg;
    nrfx_spim_xfer_desc_t spi_config = NRFX_SPIM_XFER_TRX(ads_tx_buf, 2, data, 2+num_reg);

    // reconfigure the SPIM instance
    ADS1298_reconfig();

    err_code = nrfx_spim_xfer(&ads_spim_inst, &spi_config, 0);
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("nrfx_spim_xfer failed");
        return err_code;
    }

    k_sem_take(&spi_sem, K_FOREVER);

    return 0;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_write_Nreg(uint8_t addr, uint8_t *data, uint8_t num_reg)
{
    k_sem_take(&spi_sem, K_FOREVER);
    int err_code;

    // set the transfer descriptor
    ads_tx_buf[0] = addr | 0x40;
    ads_tx_buf[1] = num_reg;
    memcpy(&ads_tx_buf[2], data, num_reg);
    nrfx_spim_xfer_desc_t spi_config = NRFX_SPIM_XFER_TX(ads_tx_buf, num_reg + 2);

    // reconfigure the SPIM instance
    ADS1298_reconfig();

    err_code = nrfx_spim_xfer(&ads_spim_inst, &spi_config, 0);
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("nrfx_spim_xfer failed");
        return err_code;
    }

    k_sem_give(&spi_sem);

    return 0;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_read_data(uint8_t *data)
{
    k_sem_take(&spi_sem, K_FOREVER);
    int err_code;

    // set the transfer descriptor
    ads_tx_buf[0] = RDATA_CMD;
    nrfx_spim_xfer_desc_t spi_config = NRFX_SPIM_XFER_TRX(ads_tx_buf, 1, data, 1+27);

    // reconfigure the SPIM instance
    ADS1298_reconfig();

    err_code = nrfx_spim_xfer(&ads_spim_inst, &spi_config, 0);
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("nrfx_spim_xfer failed");
        return err_code;
    }

    k_sem_give(&spi_sem);

    return 0;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_init_interrupt(void)
{
    nrfx_err_t err; 

    // Initialize the GPIOTE driver
    if(!nrfx_gpiote_is_init())
    {
        err = nrfx_gpiote_init(0);
        if (err != NRFX_SUCCESS)
        {
            LOG_ERR("nrfx_gpiote_init failed with error code: %d\n", err);
            return err;
        }
    }

    // Allocate a GPIOTE channel
    uint8_t ads_channel;
    err = nrfx_gpiote_channel_alloc(&ads_channel);
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("nrfx_gpiote_channel_alloc failed with error code: %d\n", err);
        return err;
    }

    // Configure the GPIOTE channel
    static const nrfx_gpiote_input_config_t input_config = {
		.pull = NRF_GPIO_PIN_PULLUP,
	};
	const nrfx_gpiote_trigger_config_t trigger_config = {
		.trigger = NRFX_GPIOTE_TRIGGER_HITOLO,
		.p_in_channel = &ads_channel,
	};
	static const nrfx_gpiote_handler_config_t handler_config = {
		.handler = ADS1298_interrupt_handler,
	};

    err = nrfx_gpiote_input_configure(ADS_INT_PIN,
									  &input_config,
									  &trigger_config,
									  &handler_config);

    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("nrfx_gpiote_input_configure failed with error code: %d\n", err);
        return err;
    }

    // Enable the GPIOTE channel
    nrfx_gpiote_trigger_enable(ADS_INT_PIN, true);

    return 0;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_start_thread(void)
{
    ADS1298_stop_thread_flag = false;

    // Start the thread
    k_thread_create(&ADS1298_thread, ADS1298_thread_stack, K_THREAD_STACK_SIZEOF(ADS1298_thread_stack),
                    ADS1298_thread_func, NULL, NULL, NULL, ADS1298_PRIO, 0, K_NO_WAIT);
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_stop_thread(void)
{
    ADS1298_stop_thread_flag = true;
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int ADS1298_config(void)
{
    ADS1298_REG reg;
    reg.REG_CONFIG1.b.HR = 0b0;
    reg.REG_CONFIG1.b.DAISY_EN = 0b0;
    reg.REG_CONFIG1.b.CLK_EN = 0b0;
    reg.REG_CONFIG1.b._reserved_3 = 0b0;
    reg.REG_CONFIG1.b._reserved_4 = 0b0;
    reg.REG_CONFIG1.b.DR = vconf_ads1298_fs;
    ADS1298_write_reg(REG_CONFIG1_Addr, reg.REG_CONFIG1.w);
    reg.REG_CONFIG2.b._reserved_7 = 0b0;
    reg.REG_CONFIG2.b._reserved_6 = 0b0;
    reg.REG_CONFIG2.b.WCT_CHOP = 0b0;
    reg.REG_CONFIG2.b.INT_TEST = 0b1;
    reg.REG_CONFIG2.b._reserved_3 = 0b0;
    reg.REG_CONFIG2.b.TEST_AMP = 0b0;
    reg.REG_CONFIG2.b.TEST_FREQ = 0b0;
    ADS1298_write_reg(REG_CONFIG2_Addr, reg.REG_CONFIG2.w);
    reg.REG_CONFIG3.b.PD_REFBUF = 0b1;
    reg.REG_CONFIG3.b._reserved_6 = 0b1;
    reg.REG_CONFIG3.b.VREF_4V = 0b1;
    reg.REG_CONFIG3.b.RLD_MEAS = 0b0;
    reg.REG_CONFIG3.b.RLDREF_INT = 0b1;
    reg.REG_CONFIG3.b.PD_RLD = 0b1;
    reg.REG_CONFIG3.b.RLD_LOFF_SENS = 0b0;
    reg.REG_CONFIG3.b.RLD_STAT = 0b0;
    ADS1298_write_reg(REG_CONFIG3_Addr, reg.REG_CONFIG3.w);
    reg.REG_LOFF.b.COMP_TH = 0b000;
    reg.REG_LOFF.b.VLEAD_OFF_EN = 0b0;
    reg.REG_LOFF.b.ILEAD_OFF = 0b00;
    reg.REG_LOFF.b.FLEAD_OFF = 0b00;
    ADS1298_write_reg(REG_LOFF_Addr, reg.REG_LOFF.w);
    reg.REG_CH1SET.b.PD1 = 0b0;
    reg.REG_CH1SET.b.GAIN1 = vconf_ads1298_gain;
    reg.REG_CH1SET.b._reserved_3 = 0b0;
    reg.REG_CH1SET.b.MUX1 = 0b000;
    ADS1298_write_reg(REG_CH1SET_Addr, reg.REG_CH1SET.w);
    reg.REG_CH2SET.b.PD2 = 0b0;
    reg.REG_CH2SET.b.GAIN2 = vconf_ads1298_gain;
    reg.REG_CH2SET.b._reserved_3 = 0b0;
    reg.REG_CH2SET.b.MUX2 = 0b000;
    ADS1298_write_reg(REG_CH2SET_Addr, reg.REG_CH2SET.w);
    reg.REG_CH3SET.b.PD3 = 0b0;
    reg.REG_CH3SET.b.GAIN3 = vconf_ads1298_gain;
    reg.REG_CH3SET.b._reserved_3 = 0b0;
    reg.REG_CH3SET.b.MUX3 = 0b000;
    ADS1298_write_reg(REG_CH3SET_Addr, reg.REG_CH3SET.w);
    reg.REG_CH4SET.b.PD4 = 0b0;
    reg.REG_CH4SET.b.GAIN4 = vconf_ads1298_gain;
    reg.REG_CH4SET.b._reserved_3 = 0b0;
    reg.REG_CH4SET.b.MUX4 = 0b000;
    ADS1298_write_reg(REG_CH4SET_Addr, reg.REG_CH4SET.w);
    reg.REG_CH5SET.b.PD5 = 0b0;
    reg.REG_CH5SET.b.GAIN5 = vconf_ads1298_gain;
    reg.REG_CH5SET.b._reserved_3 = 0b0;
    reg.REG_CH5SET.b.MUX5 = 0b000;
    ADS1298_write_reg(REG_CH5SET_Addr, reg.REG_CH5SET.w);
    reg.REG_CH6SET.b.PD6 = 0b0;
    reg.REG_CH6SET.b.GAIN6 = vconf_ads1298_gain;
    reg.REG_CH6SET.b._reserved_3 = 0b0;
    reg.REG_CH6SET.b.MUX6 = 0b000;
    ADS1298_write_reg(REG_CH6SET_Addr, reg.REG_CH6SET.w);
    reg.REG_CH7SET.b.PD7 = 0b0;
    reg.REG_CH7SET.b.GAIN7 = vconf_ads1298_gain;
    reg.REG_CH7SET.b._reserved_3 = 0b0;
    reg.REG_CH7SET.b.MUX7 = 0b000;
    ADS1298_write_reg(REG_CH7SET_Addr, reg.REG_CH7SET.w);
    reg.REG_CH8SET.b.PD8 = 0b0;
    reg.REG_CH8SET.b.GAIN8 = vconf_ads1298_gain;
    reg.REG_CH8SET.b._reserved_3 = 0b0;
    reg.REG_CH8SET.b.MUX8 = 0b000;
    ADS1298_write_reg(REG_CH8SET_Addr, reg.REG_CH8SET.w);

    return 0;
}


/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

void ADS1298_thread_func(void *arg1, void *arg2, void *arg3)
{
    ADS1298_StorageFormat storage_datas[ADS1298_CONSEC_MEAS];
    ADS1298_StorageFormat ble_datas[ADS1298_CONSEC_MEAS];
    uint8_t data[28];
    int err_code;
    int storage_count = 0; // Counter for storage measurements
    int ble_count = 0; // Counter for BLE measurements
    uint8_t index = 0;

    while (!ADS1298_stop_thread_flag)
    {
        // Wait for the new data to be ready
        k_sem_take(&new_data_rdy, K_FOREVER);

        struct time_values current_time = get_time_values();
        uint32_t rawtime_bin = current_time.rawtime_s_bin;
        uint16_t time_ms_bin = current_time.time_ms_bin;

        err_code = ADS1298_read_data(data);
        if (err_code != 0)
        {
            LOG_ERR("ADS1298_read_data failed with error code: %d\n", err_code);
            nrf_gpio_cfg_output(27);
            nrf_gpio_pin_clear(27);
        }

        storage_datas[storage_count].header = 0xDDDD;
        storage_datas[storage_count].rawtime_bin = rawtime_bin;
        storage_datas[storage_count].time_ms_bin = time_ms_bin;
        storage_datas[storage_count].len = 25;
        storage_datas[storage_count].index = index++;
        memcpy(storage_datas[storage_count].measurements, &data[4], 24);

        // Subsampling logic for BLE
        if (index % VCONF_ADS1298_SUBSAMPLING_FACTOR == 0)
        {
            ble_datas[ble_count] = storage_datas[storage_count];
            ble_count++;

            // Send data to BLE if we have collected enough measurements
            if (ble_count == ADS1298_CONSEC_MEAS)
            {
                receive_sensor_data((uint8_t *)&ble_datas, sizeof(ble_datas));
                ble_count = 0; // Reset the BLE counter
            }
        }

        storage_count++;

        // If we have ADS1298_CONSEC_MEAS measurements, write them to storage
        if (storage_count == ADS1298_CONSEC_MEAS)
        {
            int ret = storage_write_to_fifo((uint8_t *)&storage_datas, sizeof(storage_datas));
            storage_count = 0; // Reset the storage counter
        }
    }

    k_thread_abort(k_current_get());
}


/****************************************************************************/
/****************************************************************************/

int ADS1298_reconfig(void)
{
    // Reconfigure the SPIM instance for the ADS1298
    nrfx_spim_config_t ads_spim_config = NRFX_SPIM_DEFAULT_CONFIG(ADS_SCK_PIN, ADS_MOSI_PIN, ADS_MISO_PIN, ADS_SS_PIN);
    ads_spim_config.mode = NRF_SPIM_MODE_1;
    ads_spim_config.frequency = NRFX_MHZ_TO_HZ(4);

    nrfx_err_t err_code = nrfx_spim_reconfigure(&ads_spim_inst, &ads_spim_config);
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("twim_reconfigure failed with error code: %d\n", err_code);
        return -1;
    }

    LOG_INF("ADS1298_reconfig");
    return 0;
}

/****************************************************************************/
/****************************************************************************/

void ADS1298_interrupt_handler(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void *context)
{
    // Notify the thread that new data is ready
    k_sem_give(&new_data_rdy);
}

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/