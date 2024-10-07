/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : MAX86178.c                                                      **
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
* @file   MAX86178.c
* @date   DD/MM/YY
* @brief  This is the main header of MAX86178.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _MAX86178_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/
 
#include <stdlib.h>
#include "MAX86178.h"
#include <nrfx_twim.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include "storage.h"
#include "versa_time.h"
#include "versa_ble.h"
#include <nrfx_gpiote.h>

/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(max86178, LOG_LEVEL_INF);

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
* @brief This function is the thread function that manage the continuous measurement of the MAX86178
*
* @param arg1 : argument 1, not used
* @param arg2 : argument 2, not used
* @param arg3 : argument 3, not used
*
* @retval None
*/
void max86178_thread_func(void *arg1, void *arg2, void *arg3);

/**
 * @brief This function is the interrupt handler for the MAX86178
 * 
 * @param pin : the pin that triggered the interrupt
 * @param trigger : the trigger that triggered the interrupt
 * @param context : the context of the interrupt
 * 
 * @retval None
 */
void max86178_interrupt_handler(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void *context);

/**
 * @brief This function initializes the interrupt of the MAX86178
 * 
 * @retval 0 if no error
 */
int max86178_init_interrupt(void);

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

volatile MAX86178_REG MAX86178_Registers;

/*! I2C Instance pointer*/
static nrfx_twim_t *I2cInstancePtr;

/*! I2C tx and rx buffers */
uint8_t max86178_tx_buffer[MAX_SIZE_TRANSFER + 1];
uint8_t max86178_rx_buffer[RX_BUF_SIZE];

/*! Thread stack and instance */
K_THREAD_STACK_DEFINE(MAX86178_thread_stack, 1024);
struct k_thread MAX86178_thread;

volatile bool MAX86178_stop_thread = false;;

K_SEM_DEFINE(MAX86_new_data_rdy, 0, 1);

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

int max86178_init(void)
{
    /*! Get the I2C instance */
    nrfx_twim_t *I2cInstPtr = twim_get_instance();
    I2cInstancePtr=I2cInstPtr;

    // max86178_init_interrupt();

    if (max86178_check_present() != 0)
    {
        return -2;
    }

    return 0;
}

/****************************************************************************/
/****************************************************************************/

int max86178_check_present(void)
{
    uint8_t data[1];

    int err_code = max86178_read_reg(REG_PART_ID_Addr, data);
    if (err_code != 0)
    {
        LOG_ERR("ADS1298_read_reg failed with error code: %d\n", err_code);
        return -1;
    }

    if (data[0] != 0x43)
    {
        LOG_ERR("ADS1298 not present\n");
        return -1;
    }

    LOG_INF("ADS1298 present");
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int max86178_write_reg(uint8_t addr, uint8_t data)
{
    k_sem_take(&I2C_sem, K_FOREVER);
    int err_code;
    max86178_tx_buffer[0] = addr;
    max86178_tx_buffer[1] = data;

    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(MAX86178_I2C_ADDR, max86178_tx_buffer, 2);
    err_code = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("Error writing to MAX86178 register %d\n", addr);
        return -1;
    }
    k_sem_give(&I2C_sem);
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int max86178_read_reg(uint8_t addr, uint8_t *data)
{
    k_sem_take(&I2C_sem, K_FOREVER);
    int err_code;
    max86178_tx_buffer[0] = addr;

    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(MAX86178_I2C_ADDR, max86178_tx_buffer, 1, data, 1);
    err_code = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}
    if (err_code != NRFX_SUCCESS)
    {
        LOG_ERR("Error reading from MAX86178 register %d\n", addr);
        return -1;
    }
    k_sem_give(&I2C_sem);
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int max86178_write_seq_reg(uint8_t start_addr, const uint8_t *data, uint8_t num_bytes)
{
    /*! Check if the number of bytes is greater than the maximum size */
    if (num_bytes > MAX_SIZE_TRANSFER) {
        return -1;
    }

    k_sem_take(&I2C_sem, K_FOREVER);

    /*! Write the initial address and data in the tx buffer */
    max86178_tx_buffer[0] = start_addr;
    memcpy(max86178_tx_buffer[1], data, num_bytes);

    /*! Create the transfer descriptor */
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(MAX86178_I2C_ADDR, max86178_tx_buffer, num_bytes+1);

    /*! Start the transfer */
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}
    k_sem_give(&I2C_sem);
    return err == NRFX_SUCCESS ? 0 : -1;
}

/****************************************************************************/
/****************************************************************************/

int max86178_read_seq_reg(uint8_t start_addr, uint8_t *data, uint8_t num_bytes)
{
    /*! Check if the number of bytes is greater than the maximum size */
    if (num_bytes > RX_BUF_SIZE) {
        return -1;
    }

    k_sem_take(&I2C_sem, K_FOREVER);

    /*! Write the initial address in the tx buffer */
    max86178_tx_buffer[0] = start_addr;

    /*! Create the transfer descriptor */
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(MAX86178_I2C_ADDR, max86178_tx_buffer, 1, data, num_bytes);

    /*! Start the transfer */
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}
    k_sem_give(&I2C_sem);
    return err == NRFX_SUCCESS ? 0 : -1;
}

/****************************************************************************/
/****************************************************************************/

int max86178_start_thread(void)
{
    MAX86178_REG reg;
    reg.REG_PPG_CONFIG_1.b.MEAS1_EN = 0b1;
    reg.REG_PPG_CONFIG_1.b.MEAS2_EN = 0b1;
    reg.REG_PPG_CONFIG_1.b.MEAS3_EN = 0b1;
    reg.REG_PPG_CONFIG_1.b.MEAS4_EN = 0b1;
    reg.REG_PPG_CONFIG_1.b.MEAS5_EN = 0b0;
    reg.REG_PPG_CONFIG_1.b.MEAS6_EN = 0b0;
    max86178_write_reg(REG_PPG_CONFIG_1_Addr, reg.REG_PPG_CONFIG_1.w);
    MAX86178_stop_thread = false;
    k_thread_create(&MAX86178_thread, MAX86178_thread_stack, K_THREAD_STACK_SIZEOF(MAX86178_thread_stack),
                    max86178_thread_func, NULL, NULL, NULL, MAX86178_PRIO, 0, K_NO_WAIT);
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int max86178_stop_thread(void)
{
    MAX86178_REG reg;
    reg.REG_PPG_CONFIG_1.b.MEAS1_EN = 0b0;
    reg.REG_PPG_CONFIG_1.b.MEAS2_EN = 0b0;
    reg.REG_PPG_CONFIG_1.b.MEAS3_EN = 0b0;
    reg.REG_PPG_CONFIG_1.b.MEAS4_EN = 0b0;
    reg.REG_PPG_CONFIG_1.b.MEAS5_EN = 0b0;
    reg.REG_PPG_CONFIG_1.b.MEAS6_EN = 0b0;
    max86178_write_reg(REG_PPG_CONFIG_1_Addr, reg.REG_PPG_CONFIG_1.w);
    MAX86178_stop_thread = true;
    return 0;
}

/****************************************************************************/
/****************************************************************************/

int max86178_config(void)
{
    MAX86178_REG reg;

    reg.REG_SYSTEM_CONFIG_1.b.DISABLE_I2C = 0b0;
    reg.REG_SYSTEM_CONFIG_1.b.ECG_PPG_TIMING_DATA = 0b1;
    reg.REG_SYSTEM_CONFIG_1.b.BIOZ_PPG_TIMING_DATA = 0b0;
    reg.REG_SYSTEM_CONFIG_1.b.ECG_BIOZ_TIMING_DATA = 0b1;
    reg.REG_SYSTEM_CONFIG_1.b.SHDN = 0b0;
    reg.REG_SYSTEM_CONFIG_1.b.RESET = 0b0;
    max86178_write_reg(REG_SYSTEM_CONFIG_1_Addr, reg.REG_SYSTEM_CONFIG_1.w);

    reg.REG_PLL_CONFIG_1.b.MDIV = 0x01;
    reg.REG_PLL_CONFIG_1.b.PLL_LOCK_WNDW = 0b0;
    reg.REG_PLL_CONFIG_1.b.PLL_EN = 0b1;
    max86178_write_reg(REG_PLL_CONFIG_1_Addr, reg.REG_PLL_CONFIG_1.w);

    reg.REG_PLL_CONFIG_2.b.MDIV = 0xFF;
    max86178_write_reg(REG_PLL_CONFIG_2_Addr, reg.REG_PLL_CONFIG_2.w);

    reg.REG_PLL_CONFIG_3.b.BIOZ_NDIV = 0x01;
    reg.REG_PLL_CONFIG_3.b.BIOZ_KDIV = 0b0000;
    max86178_write_reg(REG_PLL_CONFIG_3_Addr, reg.REG_PLL_CONFIG_3.w);

    reg.REG_PLL_CONFIG_4.b.ECG_NDIV = 0b010;
    reg.REG_PLL_CONFIG_4.b.ECG_FDIV = 0b001;
    max86178_write_reg(REG_PLL_CONFIG_4_Addr, reg.REG_PLL_CONFIG_4.w);

    reg.REG_PLL_CONFIG_5.b.ECG_NDIV = 0x00;
    max86178_write_reg(REG_PLL_CONFIG_5_Addr, reg.REG_PLL_CONFIG_5.w);

    reg.REG_PLL_CONFIG_6.b.REF_CLK_SEL = 0b0;
    reg.REG_PLL_CONFIG_6.b.CLK_FREQ_SEL = 0b1;
    reg.REG_PLL_CONFIG_6.b.CLK_FINE_TUNE = 0b00000;
    max86178_write_reg(REG_PLL_CONFIG_6_Addr, reg.REG_PLL_CONFIG_6.w);

    reg.REG_BIOZ_CONFIG_1.b.BIOZ_DAC_OSR = 0x3;
    reg.REG_BIOZ_CONFIG_1.b.BIOZ_ADC_OSR = 0x5;
    reg.REG_BIOZ_CONFIG_1.b.ECG_BIOZ_BG_EN = 0b1;
    reg.REG_BIOZ_CONFIG_1.b.BIOZ_EN = 0b01;
    max86178_write_reg(REG_BIOZ_CONFIG_1_Addr, reg.REG_BIOZ_CONFIG_1.w);

    reg.REG_BIOZ_CONFIG_2.b.BIOZ_DHPF = 0b01;
    reg.REG_BIOZ_CONFIG_2.b.BIOZ_DLPF = 0b000;
    reg.REG_BIOZ_CONFIG_2.b.EN_BIOZ_THRESH = 0b0;
    max86178_write_reg(REG_BIOZ_CONFIG_2_Addr, reg.REG_BIOZ_CONFIG_2.w);

    reg.REG_BIOZ_CONFIG_3.b.BIOZ_EXT_RES = 0b0;
    reg.REG_BIOZ_CONFIG_3.b.BIOZ_VDRV_MAG = 0b00;
    reg.REG_BIOZ_CONFIG_3.b.BIOZ_IDRV_RGE = 0b00;
    reg.REG_BIOZ_CONFIG_3.b.BIOZ_DRV_MODE = 0b00;
    max86178_write_reg(REG_BIOZ_CONFIG_3_Addr, reg.REG_BIOZ_CONFIG_3.w);

    reg.REG_BIOZ_CONFIG_4.b.EN_UTIL_MODE = 0b0;
    max86178_write_reg(REG_BIOZ_CONFIG_4_Addr, reg.REG_BIOZ_CONFIG_4.w);

    reg.REG_BIOZ_CONFIG_5.b.BIOZ_DC_CODE_SEL = 0b0;
    reg.REG_BIOZ_CONFIG_5.b.BIOZ_DC_DAC_CODE = 0x00;
    max86178_write_reg(REG_BIOZ_CONFIG_5_Addr, reg.REG_BIOZ_CONFIG_5.w);

    reg.REG_BIOZ_CONFIG_6.b.BIOZ_AHPF = 0x5;
    reg.REG_BIOZ_CONFIG_6.b.BIOZ_INA_MODE = 0b0;
    reg.REG_BIOZ_CONFIG_6.b.BIOZ_DM_DIS = 0b0;
    reg.REG_BIOZ_CONFIG_6.b.BIOZ_GAIN = 0b00;
    max86178_write_reg(REG_BIOZ_CONFIG_6_Addr, reg.REG_BIOZ_CONFIG_6.w);

    reg.REG_BIOZ_MUX_CONFIG_1.b.BMUX_RSEL = 0b00;
    reg.REG_BIOZ_MUX_CONFIG_1.b.BMUX_BIST_EN = 0b0;
    reg.REG_BIOZ_MUX_CONFIG_1.b.CONNECT_CAL_ONLY = 0b0;
    reg.REG_BIOZ_MUX_CONFIG_1.b.BIOZ_MUX_EN = 0b1;
    reg.REG_BIOZ_MUX_CONFIG_1.b.BIOZ_CAL_EN = 0b0;
    max86178_write_reg(REG_BIOZ_MUX_CONFIG_1_Addr, reg.REG_BIOZ_MUX_CONFIG_1.w);

    reg.REG_BIOZ_MUX_CONFIG_3.b.BIP_ASSIGN = 0x2;
    reg.REG_BIOZ_MUX_CONFIG_3.b.BIN_ASSIGN = 0x2;
    reg.REG_BIOZ_MUX_CONFIG_3.b.DRVN_ASSIGN = 0x0;
    reg.REG_BIOZ_MUX_CONFIG_3.b.DRVP_ASSIGN = 0x0;
    max86178_write_reg(REG_BIOZ_MUX_CONFIG_3_Addr, reg.REG_BIOZ_MUX_CONFIG_3.w);

    reg.REG_FR_CLOCK_DIVIDER_MSB.b.FR_CLK_DIV = 0x01;
    max86178_write_reg(REG_FR_CLOCK_DIVIDER_MSB_Addr, reg.REG_FR_CLOCK_DIVIDER_MSB.w);

    reg.REG_FR_CLOCK_DIVIDER_LSB.b.FR_CLK_DIV = 0x00;
    max86178_write_reg(REG_FR_CLOCK_DIVIDER_LSB_Addr, reg.REG_FR_CLOCK_DIVIDER_LSB.w);

    reg.REG_PPG_CONFIG_2.b.PPG1_PWRDN = 0b0;
    reg.REG_PPG_CONFIG_2.b.PPG2_PWRDN = 0b1;
    reg.REG_PPG_CONFIG_2.b.PPG_SYNC_MODE = 0b0;
    max86178_write_reg(REG_PPG_CONFIG_2_Addr, reg.REG_PPG_CONFIG_2.w);

    reg.REG_PPG_CONFIG_3.b.MEAS1_CONFIG_SEL = 0b0;
    reg.REG_PPG_CONFIG_3.b.COLLECT_RAW_DATA = 0b0;
    reg.REG_PPG_CONFIG_3.b.ALC_DISABLE = 0b0;
    reg.REG_PPG_CONFIG_3.b.SMP_AVE = 0b1;
    max86178_write_reg(REG_PPG_CONFIG_3_Addr, reg.REG_PPG_CONFIG_3.w);

    reg.REG_PPG_CONFIG_4.b.PROX_AUTO = 0b0;
    reg.REG_PPG_CONFIG_4.b.PROX_DATA_EN = 0b0;
    max86178_write_reg(REG_PPG_CONFIG_4_Addr, reg.REG_PPG_CONFIG_4.w);
    
    reg.REG_ECG_CONFIG_1.b.ECG_DEC_RATE = 0x3;
    reg.REG_ECG_CONFIG_1.b.ECG_EN = 0b1;
    max86178_write_reg(REG_ECG_CONFIG_1_Addr, reg.REG_ECG_CONFIG_1.w);

    reg.REG_ECG_CONFIG_2.b.ECG_IPOL = 0b0;
    reg.REG_ECG_CONFIG_2.b.ECG_PGA_GAIN = 0b000;
    reg.REG_ECG_CONFIG_2.b.ECG_INA_RGE = 0b00;
    reg.REG_ECG_CONFIG_2.b.ECG_PGA_GAIN = 0b01;
    max86178_write_reg(REG_ECG_CONFIG_2_Addr, reg.REG_ECG_CONFIG_2.w);

    reg.REG_ECG_CONFIG_3.b.ECH_IMP_HIGH = 0b0;
    reg.REG_ECG_CONFIG_3.b.ECG_AUTO_REC = 0b1;
    reg.REG_ECG_CONFIG_3.b.ECG_MUX_SEL = 0b00;
    max86178_write_reg(REG_ECG_CONFIG_3_Addr, reg.REG_ECG_CONFIG_3.w);

    reg.REG_ECG_CONFIG_4.b.EN_ECG_FAST_REC = 0b00;
    reg.REG_ECG_CONFIG_4.b.ECG_FAST_REC_THRESHOLD = 0x3D;
    max86178_write_reg(REG_ECG_CONFIG_4_Addr, reg.REG_ECG_CONFIG_4.w);

    reg.REG_ECG_LEAD_BIAS_CONFIG_1.b.ECG_RBIAS_VALUE = 0x1;
    reg.REG_ECG_LEAD_BIAS_CONFIG_1.b.EN_ECG_RBIASP = 0b1;
    reg.REG_ECG_LEAD_BIAS_CONFIG_1.b.EN_ECG_RBIASN = 0b1;
    max86178_write_reg(REG_ECG_LEAD_BIAS_CONFIG_1_Addr, reg.REG_ECG_LEAD_BIAS_CONFIG_1.w);

    reg.REG_ECG_LEAD_DETECT_CONFIG_1.b.EN_ECG_LON = 0b0;
    reg.REG_ECG_LEAD_DETECT_CONFIG_1.b.EN_ECG_LOFF = 0b1;
    reg.REG_ECG_LEAD_DETECT_CONFIG_1.b.ECG_LOFF_MODE = 0b0;
    reg.REG_ECG_LEAD_DETECT_CONFIG_1.b.ECG_LOFF_FREQ = 0b000;
    max86178_write_reg(REG_ECG_LEAD_DETECT_CONFIG_1_Addr, reg.REG_ECG_LEAD_DETECT_CONFIG_1.w);

    reg.REG_ECG_LEAD_DETECT_CONFIG_2.b.ECG_LOFF_IPOL = 0b0;
    reg.REG_ECG_LEAD_DETECT_CONFIG_2.b.ECG_LOFF_IMAG = 0x4;
    reg.REG_ECG_LEAD_DETECT_CONFIG_2.b.ECG_LOFF_THRESH = 0x5;
    max86178_write_reg(REG_ECG_LEAD_DETECT_CONFIG_2_Addr, reg.REG_ECG_LEAD_DETECT_CONFIG_2.w);

    reg.REG_RESPIRATION_CONFIG_1.b.CG_LPF_DUTY = 0x3;
    reg.REG_RESPIRATION_CONFIG_1.b.CG_CHOP_CLK = 0x0;
    reg.REG_RESPIRATION_CONFIG_1.b.CG_MODE = 0x2;
    reg.REG_RESPIRATION_CONFIG_1.b.RESP_EN = 0b1;
    max86178_write_reg(REG_RESPIRATION_CONFIG_1_Addr, reg.REG_RESPIRATION_CONFIG_1.w);


    //reg.REG_PPG_CONFIG_1.b.MEAS1_EN = 0b1;
    //reg.REG_PPG_CONFIG_1.b.MEAS2_EN = 0b1;
    //reg.REG_PPG_CONFIG_1.b.MEAS3_EN = 0b1;
    //reg.REG_PPG_CONFIG_1.b.MEAS4_EN = 0b1;
    //reg.REG_PPG_CONFIG_1.b.MEAS5_EN = 0b0;
    //reg.REG_PPG_CONFIG_1.b.MEAS6_EN = 0b0;
    //max86178_write_reg(REG_PPG_CONFIG_1_Addr, reg.REG_PPG_CONFIG_1.w);

    reg.REG_MEAS1_SELECTS.b.MEAS1_AMB = 0b0;
    reg.REG_MEAS1_SELECTS.b.MEAS1_DRVA = 0b000;
    reg.REG_MEAS1_SELECTS.b.MEAS1_DRVB = 0b000;
    max86178_write_reg(REG_MEAS1_SELECTS_Addr, reg.REG_MEAS1_SELECTS.w);

    reg.REG_MEAS1_CONFIG_1.b.MEAS1_AVER = 0b001;
    reg.REG_MEAS1_CONFIG_1.b.MEAS1_TINT = 0b11;
    reg.REG_MEAS1_CONFIG_1.b.MEAS1_FILT_SEL = 0b0;
    reg.REG_MEAS1_CONFIG_1.b.MEAS1_FILT2_SEL = 0b1;
    reg.REG_MEAS1_CONFIG_1.b.MEAS1_SINC3_SEL = 0b0;
    max86178_write_reg(REG_MEAS1_CONFIG_1_Addr, reg.REG_MEAS1_CONFIG_1.w);

    reg.REG_MEAS1_CONFIG_2.b.MEAS1_PPG1_ADC_RGE = 0b11;
    reg.REG_MEAS1_CONFIG_2.b.MEAS1_PPG2_ADC_RGE = 0b11;
    max86178_write_reg(REG_MEAS1_CONFIG_2_Addr, reg.REG_MEAS1_CONFIG_2.w);

    reg.REG_PHOTODIODE_BIAS.b.PD1_BIAS = 0b01;
    reg.REG_PHOTODIODE_BIAS.b.PD2_BIAS = 0b01;
    reg.REG_PHOTODIODE_BIAS.b.PD3_BIAS = 0b01;
    reg.REG_PHOTODIODE_BIAS.b.PD4_BIAS = 0b01;
    max86178_write_reg(REG_PHOTODIODE_BIAS_Addr, reg.REG_PHOTODIODE_BIAS.w); 
    
    reg.REG_MEAS1_CONFIG_4.b.MEAS1_LED_RGE = 0b01;
    reg.REG_MEAS1_CONFIG_4.b.MEAS1_LED_SETLNG = 0b01;
    reg.REG_MEAS1_CONFIG_4.b.MEAS1_PD_SETLNG = 0b01;
    max86178_write_reg(REG_MEAS1_CONFIG_4_Addr, reg.REG_MEAS1_CONFIG_4.w);

    reg.REG_MEAS1_LEDA_CURRENT.b.MEAS1_DRVA_PA = 0x08;
    max86178_write_reg(REG_MEAS1_LEDA_CURRENT_Addr, reg.REG_MEAS1_LEDA_CURRENT.w);

    reg.REG_MEAS1_LEDB_CURRENT.b.MEAS1_DRVB_PA = 0x00;
    max86178_write_reg(REG_MEAS1_LEDB_CURRENT_Addr, reg.REG_MEAS1_LEDB_CURRENT.w);

    reg.REG_MEAS1_CONFIG_5.b.MEAS1_PD1_SEL = 0b10;
    reg.REG_MEAS1_CONFIG_5.b.MEAS1_PD2_SEL = 0b00;
    reg.REG_MEAS1_CONFIG_5.b.MEAS1_PD3_SEL = 0b00;
    reg.REG_MEAS1_CONFIG_5.b.MEAS1_PD4_SEL = 0b00;
    max86178_write_reg(REG_MEAS1_CONFIG_5_Addr, reg.REG_MEAS1_CONFIG_5.w);

    reg.REG_MEAS2_SELECTS.b.MEAS2_AMB = 0b0;
    reg.REG_MEAS2_SELECTS.b.MEAS2_DRVA = 0b000;
    reg.REG_MEAS2_SELECTS.b.MEAS2_DRVB = 0b001;
    max86178_write_reg(REG_MEAS2_SELECTS_Addr, reg.REG_MEAS2_SELECTS.w);

    reg.REG_MEAS2_CONFIG_1.b.MEAS2_AVER = 0b100;
    reg.REG_MEAS2_CONFIG_1.b.MEAS2_TINT = 0b11;
    reg.REG_MEAS2_CONFIG_1.b.MEAS2_FILT_SEL = 0b0;
    reg.REG_MEAS2_CONFIG_1.b.MEAS2_FILT2_SEL = 0b1;
    reg.REG_MEAS2_CONFIG_1.b.MEAS2_SINC3_SEL = 0b0;
    max86178_write_reg(REG_MEAS2_CONFIG_1_Addr, reg.REG_MEAS2_CONFIG_1.w);

    reg.REG_MEAS2_CONFIG_2.b.MEAS2_PPG1_ADC_RGE = 0b11;
    reg.REG_MEAS2_CONFIG_2.b.MEAS2_PPG2_ADC_RGE = 0b11;
    max86178_write_reg(REG_MEAS2_CONFIG_2_Addr, reg.REG_MEAS2_CONFIG_2.w);

    reg.REG_MEAS2_CONFIG_3.b.MEAS2_PPG1_DACOFF = 0x00;
    reg.REG_MEAS2_CONFIG_3.b.MEAS2_PPG2_DACOFF = 0x00;
    max86178_write_reg(REG_MEAS2_CONFIG_3_Addr, reg.REG_MEAS2_CONFIG_3.w);

    reg.REG_MEAS2_CONFIG_4.b.MEAS2_LED_RGE = 0b01;
    reg.REG_MEAS2_CONFIG_4.b.MEAS2_LED_SETLNG = 0b01;
    reg.REG_MEAS2_CONFIG_4.b.MEAS2_PD_SETLNG = 0b01;
    max86178_write_reg(REG_MEAS2_CONFIG_4_Addr, reg.REG_MEAS2_CONFIG_4.w);

    reg.REG_MEAS2_LEDA_CURRENT.b.MEAS2_DRVA_PA = 0x00;
    max86178_write_reg(REG_MEAS2_LEDA_CURRENT_Addr, reg.REG_MEAS2_LEDA_CURRENT.w);

    reg.REG_MEAS2_LEDB_CURRENT.b.MEAS2_DRVB_PA = 0x08;
    max86178_write_reg(REG_MEAS2_LEDB_CURRENT_Addr, reg.REG_MEAS2_LEDB_CURRENT.w);

    reg.REG_MEAS2_CONFIG_5.b.MEAS2_PD1_SEL = 0b10;
    reg.REG_MEAS2_CONFIG_5.b.MEAS2_PD2_SEL = 0b00;
    reg.REG_MEAS2_CONFIG_5.b.MEAS2_PD3_SEL = 0b00;
    reg.REG_MEAS2_CONFIG_5.b.MEAS2_PD4_SEL = 0b00;
    max86178_write_reg(REG_MEAS2_CONFIG_5_Addr, reg.REG_MEAS2_CONFIG_5.w);

    reg.REG_MEAS3_SELECTS.b.MEAS3_AMB = 0b0;
    reg.REG_MEAS3_SELECTS.b.MEAS3_DRVA = 0b010;
    reg.REG_MEAS3_SELECTS.b.MEAS3_DRVB = 0b000;
    max86178_write_reg(REG_MEAS3_SELECTS_Addr, reg.REG_MEAS3_SELECTS.w);

    reg.REG_MEAS3_CONFIG_1.b.MEAS3_AVER = 0b001;
    reg.REG_MEAS3_CONFIG_1.b.MEAS3_TINT = 0b11;
    reg.REG_MEAS3_CONFIG_1.b.MEAS3_FILT_SEL = 0b0;
    reg.REG_MEAS3_CONFIG_1.b.MEAS3_FILT2_SEL = 0b1;
    reg.REG_MEAS3_CONFIG_1.b.MEAS3_SINC3_SEL = 0b0;
    max86178_write_reg(REG_MEAS3_CONFIG_1_Addr, reg.REG_MEAS3_CONFIG_1.w);

    reg.REG_MEAS3_CONFIG_2.b.MEAS3_PPG1_ADC_RGE = 0b11;
    reg.REG_MEAS3_CONFIG_2.b.MEAS3_PPG2_ADC_RGE = 0b11;
    max86178_write_reg(REG_MEAS3_CONFIG_2_Addr, reg.REG_MEAS3_CONFIG_2.w);

    reg.REG_MEAS3_CONFIG_4.b.MEAS3_LED_RGE = 0b01;
    reg.REG_MEAS3_CONFIG_4.b.MEAS3_LED_SETLNG = 0b01;
    reg.REG_MEAS3_CONFIG_4.b.MEAS3_PD_SETLNG = 0b01;
    max86178_write_reg(REG_MEAS3_CONFIG_4_Addr, reg.REG_MEAS3_CONFIG_4.w);

    reg.REG_MEAS3_LEDA_CURRENT.b.MEAS3_DRVA_PA = 0x08;//0x3C;
    max86178_write_reg(REG_MEAS3_LEDA_CURRENT_Addr, reg.REG_MEAS3_LEDA_CURRENT.w);

    reg.REG_MEAS3_LEDB_CURRENT.b.MEAS3_DRVB_PA = 0x00;
    max86178_write_reg(REG_MEAS3_LEDB_CURRENT_Addr, reg.REG_MEAS3_LEDB_CURRENT.w);

    reg.REG_MEAS3_CONFIG_5.b.MEAS3_PD1_SEL = 0b10;
    reg.REG_MEAS3_CONFIG_5.b.MEAS3_PD2_SEL = 0b00;
    reg.REG_MEAS3_CONFIG_5.b.MEAS3_PD3_SEL = 0b00;
    reg.REG_MEAS3_CONFIG_5.b.MEAS3_PD4_SEL = 0b00;
    max86178_write_reg(REG_MEAS3_CONFIG_5_Addr, reg.REG_MEAS3_CONFIG_5.w);

    reg.REG_MEAS4_SELECTS.b.MEAS4_AMB = 0b1;
    reg.REG_MEAS4_SELECTS.b.MEAS4_DRVA = 0b000;
    reg.REG_MEAS4_SELECTS.b.MEAS4_DRVB = 0b000;
    max86178_write_reg(REG_MEAS4_SELECTS_Addr, reg.REG_MEAS4_SELECTS.w);

    reg.REG_MEAS4_CONFIG_1.b.MEAS4_AVER = 0b001;
    reg.REG_MEAS4_CONFIG_1.b.MEAS4_TINT = 0b11;
    reg.REG_MEAS4_CONFIG_1.b.MEAS4_FILT_SEL = 0b0;
    reg.REG_MEAS4_CONFIG_1.b.MEAS4_FILT2_SEL = 0b1;
    reg.REG_MEAS4_CONFIG_1.b.MEAS4_SINC3_SEL = 0b0;
    max86178_write_reg(REG_MEAS4_CONFIG_1_Addr, reg.REG_MEAS4_CONFIG_1.w);

    reg.REG_MEAS4_CONFIG_2.b.MEAS4_PPG1_ADC_RGE = 0b11;
    reg.REG_MEAS4_CONFIG_2.b.MEAS4_PPG2_ADC_RGE = 0b11;
    max86178_write_reg(REG_MEAS4_CONFIG_2_Addr, reg.REG_MEAS4_CONFIG_2.w);

    reg.REG_MEAS4_CONFIG_5.b.MEAS4_PD1_SEL = 0b10;
    reg.REG_MEAS4_CONFIG_5.b.MEAS4_PD2_SEL = 0b00;
    reg.REG_MEAS4_CONFIG_5.b.MEAS4_PD3_SEL = 0b00;
    reg.REG_MEAS4_CONFIG_5.b.MEAS4_PD4_SEL = 0b00;
    max86178_write_reg(REG_MEAS4_CONFIG_5_Addr, reg.REG_MEAS4_CONFIG_5.w);

    reg.REG_INTERRUPT1_ENABLE_1.b.A_FULL_EN1 = 0b1;
    reg.REG_INTERRUPT1_ENABLE_1.b.ACL_OVF_EN1 = 0b0;
    reg.REG_INTERRUPT1_ENABLE_1.b.EXP_OVF_EN1 = 0b0;
    reg.REG_INTERRUPT1_ENABLE_1.b.FIFO_DATA_RDY_EN1 = 0b0;
    reg.REG_INTERRUPT1_ENABLE_1.b.PPG_FRAME_RDY_EN1 = 0b0;
    reg.REG_INTERRUPT1_ENABLE_1.b.PPG_THRESH1_HILO_EN1 = 0b0;
    reg.REG_INTERRUPT1_ENABLE_1.b.PPG_THRESH2_HILO_EN1 = 0b0;
    max86178_write_reg(REG_INTERRUPT1_ENABLE_1_Addr, reg.REG_INTERRUPT1_ENABLE_1.w);

    reg.REG_OUTPUT_PIN_CONFIG.b.INT1_OCFG = 0b00;
    reg.REG_OUTPUT_PIN_CONFIG.b.INT2_OCFG = 0b00;
    reg.REG_OUTPUT_PIN_CONFIG.b.TRIG_OCFG = 0b00;
    max86178_write_reg(REG_OUTPUT_PIN_CONFIG_Addr, reg.REG_OUTPUT_PIN_CONFIG.w);

    //reg.REG_PPG_CONFIG_1.b.MEAS1_EN = 0b0;
    //reg.REG_PPG_CONFIG_1.b.MEAS2_EN = 0b0;
    //reg.REG_PPG_CONFIG_1.b.MEAS3_EN = 0b0;
    //reg.REG_PPG_CONFIG_1.b.MEAS4_EN = 0b0;
    //reg.REG_PPG_CONFIG_1.b.MEAS5_EN = 0b0;
    //reg.REG_PPG_CONFIG_1.b.MEAS6_EN = 0b0;
    //max86178_write_reg(REG_PPG_CONFIG_1_Addr, reg.REG_PPG_CONFIG_1.w);

    reg.REG_THRESHOLD_MEAS_SEL.b.THRESH1_MEAS_SEL = 0b00;
    reg.REG_THRESHOLD_MEAS_SEL.b.THRESH2_MEAS_SEL = 0b00;
    max86178_write_reg(REG_THRESHOLD_MEAS_SEL_Addr, reg.REG_THRESHOLD_MEAS_SEL.w);



    return 0;
}

/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

void max86178_thread_func(void *arg1, void *arg2, void *arg3)
{
    MAX86178_StorageFormat MAX86178_Storage;
    MAX86178_Storage.header = 0x9999;
    uint8_t index = 0;
    uint8_t data_read[MAX86178_FIFO_READ_SIZE];
    uint8_t status;
    int count = 0;
    while(!MAX86178_stop_thread)
    {
        // k_sem_take(&MAX86_new_data_rdy, K_FOREVER);

        max86178_read_seq_reg(REG_FIFO_DATA_REGISTER_Addr, data_read, MAX86178_FIFO_READ_SIZE);

        struct time_values current_time = get_time_values();
        uint32_t time_s_bin = current_time.rawtime_s_bin;
        uint16_t time_ms_bin = current_time.time_ms_bin;

        MAX86178_Storage.time_s_bin = time_s_bin;
        MAX86178_Storage.time_ms_bin = time_ms_bin;
        MAX86178_Storage.len = MAX86178_FIFO_READ_SIZE+1;
        MAX86178_Storage.index = index++;
        memcpy(MAX86178_Storage.data, data_read, MAX86178_FIFO_READ_SIZE);

        storage_write_to_fifo((uint8_t *)&MAX86178_Storage, sizeof(MAX86178_Storage));
        receive_sensor_data((uint8_t *)&MAX86178_Storage, sizeof(MAX86178_Storage));

        k_sleep(K_MSEC(25));

    }
}

/****************************************************************************/
/****************************************************************************/

void max86178_interrupt_handler(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void *context)
{
    k_sem_give(&MAX86_new_data_rdy);
}

/****************************************************************************/
/****************************************************************************/

int max86178_init_interrupt(void)
{
    nrfx_err_t err; 
    if(!nrfx_gpiote_is_init())
    {
        err = nrfx_gpiote_init(0);
        if (err != NRFX_SUCCESS)
        {
            LOG_ERR("nrfx_gpiote_init failed with error code: %d\n", err);
            return err;
        }
    }

    uint8_t max86_channel;
    err = nrfx_gpiote_channel_alloc(&max86_channel);
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("nrfx_gpiote_channel_alloc failed with error code: %d\n", err);
        return err;
    }

    static const nrfx_gpiote_input_config_t input_config = {
		.pull = NRF_GPIO_PIN_PULLUP,
	};
	const nrfx_gpiote_trigger_config_t trigger_config = {
		.trigger = NRFX_GPIOTE_TRIGGER_LOW,
		.p_in_channel = &max86_channel,
	};
	static const nrfx_gpiote_handler_config_t handler_config = {
		.handler = max86178_interrupt_handler,
	};

    err = nrfx_gpiote_input_configure(MAX86178_INT_PIN,
									  &input_config,
									  &trigger_config,
									  &handler_config);

    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("nrfx_gpiote_input_configure failed with error code: %d\n", err);
        return err;
    }

    nrfx_gpiote_trigger_enable(MAX86178_INT_PIN, true);

    return 0;
}

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/