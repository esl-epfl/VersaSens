/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : MAX30001.c                                                   **
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
* @file   MAX30001.c
* @date   DD/MM/YY
* @brief  This is the main header of MAX30001.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _MAX30001_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include <stdlib.h>
#include "MAX30001.h"
#include <nrfx_spim.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrfx_gpiote.h>
#include "versa_time.h"
#include "versa_config.h"
#include "storage.h"
#include "versa_ble.h"
#include "spim_inst.h"

/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(MAX30001, LOG_LEVEL_INF);

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

void MAX30001_thread_func(void *arg1, void *arg2, void *arg3);

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
static nrfx_spim_t max_spim_inst = NRFX_SPIM_INSTANCE(MAX_INST_IDX);

uint8_t max_tx_buf[20];

bool max_last_transfer_succeeded = false;
bool max_new_data = false;

/*! Thread stack and instance */
K_THREAD_STACK_DEFINE(MAX30001_thread_stack, 1024);
struct k_thread MAX30001_thread;

volatile bool MAX30001_stop_thread_flag = false;

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

int MAX30001_init(void)
{
    int ret_val = 0;
    nrfx_spim_config_t max_spim_config = NRFX_SPIM_DEFAULT_CONFIG(MAX_SCK_PIN, MAX_MOSI_PIN, MAX_MISO_PIN, MAX_SS_PIN);

    nrfx_err_t err_code = nrfx_spim_init(&max_spim_inst, &max_spim_config, NULL, NULL);
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("spim_init failed with error code: %d\n", err_code);
        ret_val = -1;
    }

    k_sleep(K_MSEC(100));

    if(MAX30001_check_present() != 0)
    {
        ret_val = -2;
    }

    LOG_INF("MAX30001_init");
    return ret_val;

}

/****************************************************************************/
/****************************************************************************/

int MAX30001_check_present(void)
{
    uint8_t data_read[4];
    data_read[1] = 0x00;
    MAX30001_normal_read(REG_INFO_Addr, data_read);
    k_sleep(K_MSEC(10));
    MAX30001_normal_read(REG_INFO_Addr, data_read);
    k_sleep(K_MSEC(10));
    MAX30001_normal_read(REG_INFO_Addr, data_read);
    if ((uint8_t)(data_read[1] & 0xF0) == 0x50)
    {
        LOG_INF("MAX30001_check_present: OK");
        return 0;
    }
    else
    {
        LOG_ERR("MAX30001_check_present: NOK");
        return 1;
    }

}

/****************************************************************************/
/****************************************************************************/

int MAX30001_reconfig(void)
{
    nrfx_spim_config_t max_spim_config = NRFX_SPIM_DEFAULT_CONFIG(MAX_SCK_PIN, MAX_MOSI_PIN, MAX_MISO_PIN, MAX_SS_PIN);

    nrfx_err_t err_code = nrfx_spim_reconfigure(&max_spim_inst, &max_spim_config);
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("twim_reconfigure failed with error code: %d\n", err_code);
        return -1;
    }

    // LOG_INF("MAX30001_reconfig");
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int MAX30001_normal_read(uint8_t addr, uint8_t *data)
{
    k_sem_take(&spi_sem, K_FOREVER);
    max_tx_buf[0] = addr<<1 | 0x01;
    nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_XFER_TRX(max_tx_buf, 1, data, 4); 

    MAX30001_reconfig();

    nrfx_err_t err_code = nrfx_spim_xfer(&max_spim_inst, &xfer, 0);
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("twim_xfer failed with error code: %d\n", err_code);
        return -1;
    }

    k_sem_give(&spi_sem);

    return 0;
}

/****************************************************************************/
/****************************************************************************/

int MAX30001_normal_write(uint8_t addr, uint8_t *data)
{
    k_sem_take(&spi_sem, K_FOREVER);
    max_tx_buf[0] = addr<<1 | 0x00;
    
    // Reverse the order of bytes
    for (int i = 0; i < 3; i++) {
        max_tx_buf[3 - i] = data[i];
    }


    nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_XFER_TX(max_tx_buf, 4);

    MAX30001_reconfig();

    nrfx_err_t err_code = nrfx_spim_xfer(&max_spim_inst, &xfer, 0);
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("twim_xfer failed with error code: %d\n", err_code);
        return -1;
    }

    k_sem_give(&spi_sem);

    return 0;
}

/****************************************************************************/
/****************************************************************************/

int MAX30001_burst_read(uint8_t addr, uint8_t *data, uint8_t len)
{
    k_sem_take(&spi_sem, K_FOREVER);
    max_tx_buf[0] = addr<<1 | 0x01;

    nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_XFER_TRX(max_tx_buf, 1, data, 1+len*3);

    MAX30001_reconfig();

    nrfx_err_t err_code = nrfx_spim_xfer(&max_spim_inst, &xfer, 0);
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("twim_xfer failed with error code: %d\n", err_code);
        return -1;
    }

    k_sem_give(&spi_sem);

    return 0;
}

/****************************************************************************/
/****************************************************************************/

void MAX30001_start_thread(void)
{
    uint8_t conf[3];
    conf[0] = 0x00;
    conf[1] = 0x00;
    conf[2] = 0x00;
    MAX30001_normal_write(REG_FIFO_RST_Addr, conf);

    MAX30001_stop_thread_flag = false;
    k_thread_create(&MAX30001_thread, MAX30001_thread_stack, K_THREAD_STACK_SIZEOF(MAX30001_thread_stack),
                    MAX30001_thread_func, NULL, NULL, NULL, MAX30001_PRIO, 0, K_NO_WAIT);
}

/****************************************************************************/
/****************************************************************************/

void MAX30001_stop_thread(void)
{
    MAX30001_stop_thread_flag = true;
}

/****************************************************************************/
/****************************************************************************/

void MAX30001_config(void)
{
    MAX30001_REG reg;

    /****************************************************************************/
    /**                                EEG MODE                                **/
    /****************************************************************************/

    if(vconf_max30001_mode == VCONF_MAX30001_MODE_ECG)
    {
        reg.REG_CNFG_GEN.b.EN_ULP_LON = 0b00;
        reg.REG_CNFG_GEN.b.FMSTR = 0b00;
        reg.REG_CNFG_GEN.b.EN_ECG = 0b1;
        reg.REG_CNFG_GEN.b.EN_BIOZ = 0b0;
        reg.REG_CNFG_GEN.b.EN_PACE = 0b0;
        reg.REG_CNFG_GEN.b.EN_BLOFF = 0b00;
        reg.REG_CNFG_GEN.b.EN_DCLOFF = 0b00;
        reg.REG_CNFG_GEN.b.DCLOFF_IPOL = 0b0;
        reg.REG_CNFG_GEN.b.IMAG = 0b000;
        reg.REG_CNFG_GEN.b.VTH = 0b00;
        reg.REG_CNFG_GEN.b.EN_RBIAS = 0b00;
        reg.REG_CNFG_GEN.b.RBIASV = 0b01;
        reg.REG_CNFG_GEN.b.RBIASP = 0b0;
        reg.REG_CNFG_GEN.b.RBIASN = 0b0;
        MAX30001_normal_write(REG_CNFG_GEN_Addr, reg.REG_CNFG_GEN.w);

        reg.REG_CNFG_CAL.b.EN_VCAL = 0b1;
        reg.REG_CNFG_CAL.b.VMODE = 0b1;
        reg.REG_CNFG_CAL.b.VMAG = 0b1;
        reg.REG_CNFG_CAL.b.FCAL = 0b100;
        reg.REG_CNFG_CAL.b.FIFTY = 0b0;
        reg.REG_CNFG_CAL.b.THIGH = 0b0000000000;
        MAX30001_normal_write(REG_CNFG_CAL_Addr, reg.REG_CNFG_CAL.w);

        reg.REG_CNFG_ECG.b.ECG_RATE = 0b10;     //128sps
        reg.REG_CNFG_ECG.b.ECG_GAIN = 0b10;     //80V/V
        reg.REG_CNFG_ECG.b.ECG_DHPF = 0b1;      //0.50Hz
        reg.REG_CNFG_ECG.b.ECG_DLPF = 0b01;     //40Hz
        MAX30001_normal_write(REG_CNFG_ECG_Addr, reg.REG_CNFG_ECG.w);

        reg.REG_CNFG_EMUX.b.ECG_POL = 0b0;
        reg.REG_CNFG_EMUX.b.ECG_OPENP = 0b0;
        reg.REG_CNFG_EMUX.b.ECG_OPENN = 0b0;
        reg.REG_CNFG_EMUX.b.ECG_CALP_SEL = 0b00;
        reg.REG_CNFG_EMUX.b.ECG_CALN_SEL = 0b00;
        MAX30001_normal_write(REG_CNFG_EMUX_Addr, reg.REG_CNFG_EMUX.w);

        reg.REG_SYNCH[0] = 0x00;
        reg.REG_SYNCH[1] = 0x00;
        reg.REG_SYNCH[2] = 0x00;
        MAX30001_normal_write(REG_SYNCH_Addr, reg.REG_SYNCH);
    }

    /****************************************************************************/
    /**                             EEG + BIOZ MODE                            **/
    /****************************************************************************/

    else if(vconf_max30001_mode == VCONF_MAX30001_MODE_ECG_BIOZ)
    {
        reg.REG_CNFG_GEN.b.EN_ULP_LON = 0b00;
        reg.REG_CNFG_GEN.b.FMSTR = 0b00;
        reg.REG_CNFG_GEN.b.EN_ECG = 0b1;
        reg.REG_CNFG_GEN.b.EN_BIOZ = 0b1;
        reg.REG_CNFG_GEN.b.EN_PACE = 0b0;
        reg.REG_CNFG_GEN.b.EN_BLOFF = 0b00;
        reg.REG_CNFG_GEN.b.EN_DCLOFF = 0b00;
        reg.REG_CNFG_GEN.b.DCLOFF_IPOL = 0b0;
        reg.REG_CNFG_GEN.b.IMAG = 0b000;
        reg.REG_CNFG_GEN.b.VTH = 0b00;
        reg.REG_CNFG_GEN.b.EN_RBIAS = 0b00;
        reg.REG_CNFG_GEN.b.RBIASV = 0b01;
        reg.REG_CNFG_GEN.b.RBIASP = 0b0;
        reg.REG_CNFG_GEN.b.RBIASN = 0b0;
        MAX30001_normal_write(REG_CNFG_GEN_Addr, reg.REG_CNFG_GEN.w);

        reg.REG_CNFG_CAL.b.EN_VCAL = 0b1;
        reg.REG_CNFG_CAL.b.VMODE = 0b1;
        reg.REG_CNFG_CAL.b.VMAG = 0b1;
        reg.REG_CNFG_CAL.b.FCAL = 0b100;
        reg.REG_CNFG_CAL.b.FIFTY = 0b0;
        reg.REG_CNFG_CAL.b.THIGH = 0b0000000000;
        MAX30001_normal_write(REG_CNFG_CAL_Addr, reg.REG_CNFG_CAL.w);

        reg.REG_CNFG_ECG.b.ECG_RATE = 0b10;     //128sps
        reg.REG_CNFG_ECG.b.ECG_GAIN = 0b10;     //80V/V
        reg.REG_CNFG_ECG.b.ECG_DHPF = 0b1;      //0.50Hz
        reg.REG_CNFG_ECG.b.ECG_DLPF = 0b01;     //40Hz
        MAX30001_normal_write(REG_CNFG_ECG_Addr, reg.REG_CNFG_ECG.w);

        reg.REG_CNFG_EMUX.b.ECG_POL = 0b0;
        reg.REG_CNFG_EMUX.b.ECG_OPENP = 0b0;
        reg.REG_CNFG_EMUX.b.ECG_OPENN = 0b0;
        reg.REG_CNFG_EMUX.b.ECG_CALP_SEL = 0b00;
        reg.REG_CNFG_EMUX.b.ECG_CALN_SEL = 0b00;
        MAX30001_normal_write(REG_CNFG_EMUX_Addr, reg.REG_CNFG_EMUX.w);

        reg.REG_CNFG_BIOZ.b.BIOZ_RATE = 0b0;        //64sps
        reg.REG_CNFG_BIOZ.b.BIOZ_AHPF = 0b010;      //800Hz
        reg.REG_CNFG_BIOZ.b.EXT_RBIAS = 0b0;        //Internal Rbias
        reg.REG_CNFG_BIOZ.b.LN_BIOZ = 0b1;          //low noise mode
        reg.REG_CNFG_BIOZ.b.BIOZ_GAIN = 0b01;       //20V/V
        reg.REG_CNFG_BIOZ.b.BIOZ_DHPF = 0b10;       //0.50Hz
        reg.REG_CNFG_BIOZ.b.BIOZ_DLPF = 0b01;       //4Hz
        reg.REG_CNFG_BIOZ.b.BIOZ_FCGEN = 0b0100;    //8000Hz
        reg.REG_CNFG_BIOZ.b.BIOZ_CGMON = 0b0;       //CGMAG disabled
        reg.REG_CNFG_BIOZ.b.BIOZ_CGMAG = 0b100;     //48uA
        reg.REG_CNFG_BIOZ.b.BIOZ_PHOFF = 0b0011;   
        MAX30001_normal_write(REG_CNFG_BIOZ_Addr, reg.REG_CNFG_BIOZ.w);

        reg.REG_CNFG_BMUX.b.BMUX_OPENP = 0b0;
        reg.REG_CNFG_BMUX.b.BMUX_OPENN = 0b0;
        reg.REG_CNFG_BMUX.b.BMUX_CALP_SEL = 0b00;
        reg.REG_CNFG_BMUX.b.BMUX_CALN_SEL = 0b00;
        reg.REG_CNFG_BMUX.b.BMUX_CG_MODE = 0b00;
        reg.REG_CNFG_BMUX.b.BMUX_EN_BIST = 0b0;
        reg.REG_CNFG_BMUX.b.BMUX_RNOM = 0b000;
        reg.REG_CNFG_BMUX.b.BMUX_RMOD = 0b100;
        reg.REG_CNFG_BMUX.b.BMUX_FBIST = 0b00;
        MAX30001_normal_write(REG_CNFG_BMUX_Addr, reg.REG_CNFG_BMUX.w);

        reg.REG_SYNCH[0] = 0x00;
        reg.REG_SYNCH[1] = 0x00;
        reg.REG_SYNCH[2] = 0x00;
        MAX30001_normal_write(REG_SYNCH_Addr, reg.REG_SYNCH);
    }

    return;
}

/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

void MAX30001_thread_func(void *arg1, void *arg2, void *arg3)
{
    uint8_t data_read[4];
    MAX30001_StorageFormat format_datas[CONSEC_SAMPLES];
    int count = 0; // Counter for measurements
    uint8_t index = 0;
    // initialize the measurements
    while (!MAX30001_stop_thread_flag)
    {
        struct time_values current_time = get_time_values();
        uint32_t rawtime_bin = current_time.rawtime_s_bin;
        uint16_t time_ms_bin = current_time.time_ms_bin;

        MAX30001_normal_read(REG_ECG_FIFO_Addr, data_read);

        if(data_read[3] != 0x37)
        {
            format_datas[count].header = 0xEEEE;
            format_datas[count].rawtime_bin = rawtime_bin;
            format_datas[count].time_ms_bin = time_ms_bin;
            format_datas[count].len = 7;
            format_datas[count].index = index++;
            memcpy(format_datas[count].measurements, &data_read[1], 3);
            if (VCONF_MAX30001_MODE == VCONF_MAX30001_MODE_ECG_BIOZ)
            {
                MAX30001_burst_read(REG_BIOZ_FIFO_Addr, data_read, 1);
                memcpy(&format_datas[count].measurements[3], &data_read[1], 3);
            }
            else{
                format_datas[count].measurements[3] = 0;
                format_datas[count].measurements[4] = 0;
                format_datas[count].measurements[5] = 0;
            }
            count++;
        }

        if (count == CONSEC_SAMPLES)
        {
            storage_write_to_fifo((uint8_t *)&format_datas, sizeof(format_datas));
            receive_sensor_data((uint8_t *)&format_datas, sizeof(format_datas));
            count = 0;
        }


        k_sleep(K_MSEC(2));
    }
    k_thread_abort(k_current_get());
}

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/