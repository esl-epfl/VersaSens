/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : MAX77658.c                                                   **
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
* @file   MAX77658.c
* @date   DD/MM/YY
* @brief  This is the main header of MAX77658.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _MAX77658_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include <stdlib.h>
#include "MAX77658.h"
#include <nrfx_twim.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "twim_inst.h"
#include "storage.h"
#include "versa_ble.h"
#include "versa_time.h"

/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(MAX77658, LOG_LEVEL_INF);

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

void max77658_thread_func(void *arg1, void *arg2, void *arg3);

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

/*! I2C Instance pointer*/
static nrfx_twim_t *I2cInstancePtr;

uint8_t tx_buffer_pmu[MAX_SIZE_TRANSFER + 1];

bool MAX77658_last_transfer_succeeded = false;

volatile MAX77658_REG MAX77658_registers;
volatile MAX77658_FG_REG MAX77658_FG_registers;

bool MAX77658_cont_read = false;

MAX77658_StorageFormat MAX77658_Storage;

/*! Thread stack and instance */
K_THREAD_STACK_DEFINE(MAX77658_thread_stack, 1024);
struct k_thread MAX77658_thread;

bool battery_charging = false;
bool battery_low = false;
bool temperature_high = false;

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/


/**
 * @brief Function for handling TWIM driver events.
 *
 * @param[in] p_event   Event information structure.
 * @param[in] p_context General purpose parameter set during initialization of the TWIM.
 *                      This parameter can be used to pass additional information to the
 *                      handler function. In this example @p p_context is used to pass address
 *                      of TWI transfer descriptor structure.
 */
static void MAX77658_handler(nrfx_twim_evt_t const * p_event, void * p_context)
{
    // printk("TWIM event: %d\n", p_event->type);
    switch (p_event->type)
    {
        case NRFX_TWIM_EVT_DONE:
            MAX77658_last_transfer_succeeded = true;
            break;
        case NRFX_TWIM_EVT_ADDRESS_NACK:
            LOG_ERR("TWIM address NACK\n");
            MAX77658_last_transfer_succeeded = false;
            break;
        case NRFX_TWIM_EVT_DATA_NACK:
            LOG_ERR("TWIM data NACK\n");
            MAX77658_last_transfer_succeeded = false;
            break;
        case NRFX_TWIM_EVT_OVERRUN:
            LOG_ERR("TWIM overrun\n");
            MAX77658_last_transfer_succeeded = false;
            break;
        case NRFX_TWIM_EVT_BUS_ERROR:
            LOG_ERR("TWIM bus error\n");
            MAX77658_last_transfer_succeeded = false;
            break;
        default:
            break;
    }
}

/*****************************************************************************
*****************************************************************************/


int MAX77658_read_8bit(uint8_t addr, uint8_t *data){
    k_sem_take(&I2C_sem, K_FOREVER);
    tx_buffer_pmu[0] = addr;
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(PMU_MAIN_ADDR, tx_buffer_pmu, 1, data, 1);
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

    k_sem_give(&I2C_sem);

    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_write_8bit(uint8_t addr, uint8_t data){
    k_sem_take(&I2C_sem, K_FOREVER);
    tx_buffer_pmu[0] = addr;
    tx_buffer_pmu[1] = data;
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(PMU_MAIN_ADDR, tx_buffer_pmu, 2);
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

    k_sem_give(&I2C_sem);

    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_read_8bit_seq(uint8_t start_addr, uint8_t *data, uint8_t num_bytes) 
{
    k_sem_take(&I2C_sem, K_FOREVER);
    tx_buffer_pmu[0] = start_addr;
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(PMU_MAIN_ADDR, tx_buffer_pmu, 1, data, num_bytes);
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

    k_sem_give(&I2C_sem);

    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_write_8bit_seq(uint8_t start_addr, uint8_t *data, uint8_t num_bytes) 
{
    if (num_bytes > MAX_SIZE_TRANSFER) {
        return -1;
    }

    k_sem_take(&I2C_sem, K_FOREVER);

    tx_buffer_pmu[0] = start_addr;
    memcpy(&tx_buffer_pmu[1], data, num_bytes);
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(PMU_MAIN_ADDR, tx_buffer_pmu, num_bytes+1);
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);

    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

    k_sem_give(&I2C_sem);

    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_write_16bit(uint8_t start_address, uint16_t *data, size_t num_words)
{
    if (num_words > MAX_SIZE_TRANSFER/2) {
        return -1;
    }

    k_sem_take(&I2C_sem, K_FOREVER);

    tx_buffer_pmu[0] = start_address;

    // Fill the buffer with data, LSB first
    for (size_t i = 0; i < num_words; i++) {
        tx_buffer_pmu[i*2 + 1] = data[i] & 0xFF;  // LSB
        tx_buffer_pmu[i*2 + 2] = data[i] >> 8;    // MSB
    }

    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(PMU_FUEL_GAUGE_ADDR, tx_buffer_pmu, num_words*2 + 1);
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);

    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

    k_sem_give(&I2C_sem);

    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_read_16bit(uint8_t start_address, uint16_t *data, size_t num_words)
{
    k_sem_take(&I2C_sem, K_FOREVER);
    tx_buffer_pmu[0] = start_address;
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(PMU_FUEL_GAUGE_ADDR, tx_buffer_pmu, 1, (uint8_t *)data, num_words*2);
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);

    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}

    k_sem_give(&I2C_sem);

    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_init(void){
    nrfx_twim_t *I2cInstPtr = twim_get_instance();
    I2cInstancePtr=I2cInstPtr;

    MAX77658_REG reg;
    MAX77658_FG_REG fg_reg;

    reg.REG_CNFG_SBB2_B.b.OP_MODE = 0b00;
    reg.REG_CNFG_SBB2_B.b.IP_SBB2 = 0b00;
    reg.REG_CNFG_SBB2_B.b.ADE_SBB2 = 0b0;
    reg.REG_CNFG_SBB2_B.b.EN_SBB2 = 0b100;
    MAX77658_write_8bit(REG_CNFG_SBB2_B_ADDR, reg.REG_CNFG_SBB2_B.w);
    reg.REG_CNFG_CHG_E.b.CHG_CC = 0x3F;             //300mA
    reg.REG_CNFG_CHG_E.b.T_FAST_CHG = 0b01;         //3h
    MAX77658_write_8bit(REG_CNFG_CHG_E_ADDR, reg.REG_CNFG_CHG_E.w);
    reg.REG_CNFG_CHG_F.b.CHG_CC_JEITA = 0x3F;       //300mA
    reg.REG_CNFG_CHG_F.b._reserved_1 = 0b0;
    MAX77658_write_8bit(REG_CNFG_CHG_F_ADDR, reg.REG_CNFG_CHG_F.w);
    reg.REG_CNFG_CHG_G.b.CHG_CV = 0x18;             //4.20V
    reg.REG_CNFG_CHG_G.b.USBS = 0b0;                //USBS not suspended
    reg.REG_CNFG_CHG_G.b.FUS_M = 0b1;
    MAX77658_write_8bit(REG_CNFG_CHG_G_ADDR, reg.REG_CNFG_CHG_G.w);
    reg.REG_CNFG_CHG_H.b.CHG_CV_JEITA = 0x18;       //4.20V
    reg.REG_CNFG_CHG_H.b.SYS_BAT_PRT = 0b1;
    reg.REG_CNFG_CHG_H.b.CHR_TH_EN = 0b1;
    MAX77658_write_8bit(REG_CNFG_CHG_H_ADDR, reg.REG_CNFG_CHG_H.w);
    reg.REG_CNFG_CHG_B.b.VCHGIN_MIN = 0b000;        //4.0V
    reg.REG_CNFG_CHG_B.b.ICHGIN_LIM = 0b000;        //475mA
    reg.REG_CNFG_CHG_B.b.I_PQ = 0b0;                //10%
    reg.REG_CNFG_CHG_B.b.CHG_EN = 0b1;              //Enable charger
    MAX77658_write_8bit(REG_CNFG_CHG_B_ADDR, reg.REG_CNFG_CHG_B.w);
    uint16_t capa = 0x01A4;
    MAX77658_write_16bit(REG_DesignCap_ADDR, &capa, 1);
    uint16_t IchgTerm = 0x0070;
    MAX77658_write_16bit(REG_IChgTerm_ADDR, &IchgTerm, 1);
    uint16_t Vempty = 0x9600;
    MAX77658_write_16bit(REG_VEmpty_ADDR, &Vempty, 1);

    k_thread_create(&MAX77658_thread, MAX77658_thread_stack, K_THREAD_STACK_SIZEOF(MAX77658_thread_stack),
                    max77658_thread_func, NULL, NULL, NULL, MAX77658_PRIO, 0, K_NO_WAIT);

    printk("MAX77658_init\n");
    return 0;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_config(void){
    printk("MAX77658_configure\n");
    return 0;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_start_continuous_read(void)
{
    MAX77658_cont_read = true;

    return 0;
}

/*****************************************************************************
*****************************************************************************/

int MAX77658_stop_continuous_read(void)
{
    MAX77658_cont_read = false;

    return 0;
}

/*****************************************************************************
*****************************************************************************/

bool get_battery_charging(void)
{
    return battery_charging;
}

/*****************************************************************************
*****************************************************************************/

bool get_temperature_high(void)
{
    return temperature_high;
}

/*****************************************************************************
*****************************************************************************/

bool get_battery_low(void)
{
    return battery_low;
}

/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

void max77658_thread_func(void *arg1, void *arg2, void *arg3)
{
    MAX77658_Storage.header = 0x8888;
    uint8_t len = 9;
    uint8_t index = 0;
    uint16_t data_read[4];
    uint8_t data_read_8bit[1];
    while(1)
    {
        struct time_values current_time = get_time_values();
        uint32_t time_s_bin = current_time.rawtime_s_bin;
        uint16_t time_ms_bin = current_time.time_ms_bin;

        MAX77658_read_16bit(REG_Temp_ADDR, data_read, 3);
        MAX77658_read_16bit(REG_RepSOC_ADDR, &data_read[3], 1);
        MAX77658_read_8bit(REG_STAT_CHG_B_ADDR, data_read_8bit);

        if(data_read_8bit[0] & 0b00000010){
            battery_charging = true;
        }
        else{
            battery_charging = false;
        }

        set_battery_data(&data_read[1]);

        uint8_t temp = ((uint8_t*)data_read)[1];

        if(temp > 50){
            temperature_high = true;
        }
        else{
            temperature_high = false;
        }

        uint16_t voltage = data_read[1];

        if(voltage < 42240){
            battery_low = true;
        }
        else{
            battery_low = false;
        }

        if(MAX77658_cont_read){

            MAX77658_Storage.time_s_bin = time_s_bin;
            MAX77658_Storage.time_ms_bin = time_ms_bin;
            MAX77658_Storage.len = len;
            MAX77658_Storage.index = index++;
            MAX77658_Storage.temperature = data_read[0];
            MAX77658_Storage.voltage = data_read[1];
            MAX77658_Storage.current = data_read[2];
            MAX77658_Storage.soc = data_read[3];

            storage_write_to_fifo((uint8_t *)&MAX77658_Storage, sizeof(MAX77658_Storage));
            receive_sensor_data((uint8_t *)&MAX77658_Storage, sizeof(MAX77658_Storage));

        }

        k_sleep(K_MSEC(200));
    }
    k_thread_abort(k_current_get());
}

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/