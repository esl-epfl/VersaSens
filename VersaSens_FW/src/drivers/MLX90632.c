/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : MLX90632.c                                                   **
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
* @file   MLX90632.c
* @date   DD/MM/YY
* @brief  This is the main storage_format of MLX90632.c
*
* Here typically goes a more extensive explanation of what the storage_format
* defines.
*/

#define _MLX90632_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include <stdlib.h>
#include "MLX90632.h"
#include <nrfx_twim.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include "storage.h"
#include "versa_time.h"
#include "versa_ble.h"



/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(mlx90632, LOG_LEVEL_INF);

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
* @brief This function is the thread function that continuously reads the temperature
*
* @param arg1 : argument 1, not used
* @param arg2 : argument 2, not used
* @param arg3 : argument 3, not used
*
* @retval None
*/
void mlx90632_thread_func(void *arg1, void *arg2, void *arg3);

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED VARIABLES                             */
/**                                                                        **/
/****************************************************************************/

volatile MLX90632_MATH_PARAM math_param;
volatile MLX90632_REG mlx_reg;
volatile MLX90632_RAM_DATA ram_data; 

/*! Last cycle position */
int last_CycPos = 0;

float AmbientTemperature;
float ObjectTemperature;

/****************************************************************************/
/**                                                                        **/
/*                            GLOBAL VARIABLES                              */
/**                                                                        **/
/****************************************************************************/

/*! I2C Instance pointer*/
static nrfx_twim_t *I2cInstancePtr;

/*! I2C tx and rx buffers */
uint8_t tx_buffer[MAX_SIZE_TRANSFER + 2];
uint8_t rx_buffer[RX_BUF_SIZE];

/*! Thread stack and instance */
K_THREAD_STACK_DEFINE(MLX90632_thread_stack, 1024);
struct k_thread MLX90632_thread;

volatile bool MLX90632_stop_thread = false;;

bool MLX90632_writing = false;

MLX90632_StorageFormat format;

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

int mlx90632_read_reg(uint16_t addr, uint8_t *data, uint8_t reg_size)
{
    k_sem_take(&I2C_sem, K_FOREVER);
    /*! Write the address in the tx buffer */
    tx_buffer[0] = (addr >> 8) & 0xFF;
    tx_buffer[1] = addr & 0xFF;

    /*! Create the transfer descriptor */
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(TEMP_SENSOR_I2C_ADDR, tx_buffer, 2, data, reg_size);

    /*! Start the transfer */
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}
    k_sem_give(&I2C_sem);
    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int mlx90632_write_reg(uint16_t addr, uint16_t data)
{
    k_sem_take(&I2C_sem, K_FOREVER);
    /*! Write the address and data in the tx buffer */
    tx_buffer[0] = (addr >> 8) & 0xFF;
    tx_buffer[1] = addr & 0xFF;
    tx_buffer[2] = (data >> 8) & 0xFF;
    tx_buffer[3] = data & 0xFF;

    /*! Create the transfer descriptor */
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(TEMP_SENSOR_I2C_ADDR, tx_buffer, 4);

    /*! Start the transfer */
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}
    k_sem_give(&I2C_sem);
    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int mlx90632_reset(void)
{
    k_sem_take(&I2C_sem, K_FOREVER);
    /*! Write the reset command in the tx buffer */
    tx_buffer[0] = 0x30;
    tx_buffer[1] = 0x05;
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x06;

    /*! Create the transfer descriptor */
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(TEMP_SENSOR_I2C_ADDR, tx_buffer, 4);

    /*! Start the transfer */
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}
    k_sem_give(&I2C_sem);
    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int mlx90632_read_seq_reg(uint16_t start_addr, uint8_t *data, uint8_t num_bytes) 
{
    k_sem_take(&I2C_sem, K_FOREVER);
    /*! Write the initial address in the tx buffer */
    tx_buffer[0] = (start_addr >> 8) & 0xFF;
    tx_buffer[1] = start_addr & 0xFF;

    /*! Create the transfer descriptor */
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TXRX(TEMP_SENSOR_I2C_ADDR, tx_buffer, 2, data, num_bytes);

    /*! Start the transfer */
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}
    k_sem_give(&I2C_sem);
    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int mlx90632_write_seq_reg(uint16_t start_addr, const uint8_t *data, uint8_t num_bytes) 
{
    k_sem_take(&I2C_sem, K_FOREVER);
    /*! Check if the number of bytes is greater than the maximum size */
    if (num_bytes > MAX_SIZE_TRANSFER) {
        return -1;
    }

    /*! Write the initial address and data in the tx buffer */
    tx_buffer[0] = (start_addr >> 8) & 0xFF;
    tx_buffer[1] = start_addr & 0xFF;
    memcpy(&tx_buffer[2], data, num_bytes);

    /*! Create the transfer descriptor */
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(TEMP_SENSOR_I2C_ADDR, tx_buffer, num_bytes+2);

    /*! Start the transfer */
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}
    k_sem_give(&I2C_sem);
    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int mlx90632_eeprom_unlock(void)
{
    k_sem_take(&I2C_sem, K_FOREVER);
    /*! Write the unlock command in the tx buffer */
    tx_buffer[0] = 0x30;
    tx_buffer[1] = 0x05;
    tx_buffer[2] = 0x55;
    tx_buffer[3] = 0x4C;

    /*! Create the transfer descriptor */
    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(TEMP_SENSOR_I2C_ADDR, tx_buffer, 4);

    /*! Start the transfer */
    nrfx_err_t err = nrfx_twim_xfer(I2cInstancePtr, &xfer, 0);
    while(nrfx_twim_is_busy(I2cInstancePtr) == true){k_sleep(K_USEC(10));}
    k_sem_give(&I2C_sem);
    return err == NRFX_SUCCESS ? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int mlx90632_init(void)
{
    /*! Get the I2C instance */
    nrfx_twim_t *I2cInstPtr = twim_get_instance();
    I2cInstancePtr=I2cInstPtr;

    /*! Initialize the temperature computation parameters */
    math_param.TA0 = 25;
    math_param.TO0 = 25;
    math_param.e = 1;
    math_param.Fa = 0;
    math_param.Fb = 0;
    math_param.Ga = 0;
    math_param.Gb = 0;
    math_param.Ha = 0;
    math_param.Hb = 0;
    math_param.TAdut = 0;
    math_param.TOdut = 25;
    math_param.Ea = 0;
    math_param.Eb = 0;

    LOG_INF("MLX90632_init\n");
    return 0;
}

/*****************************************************************************
*****************************************************************************/

void log_float(float f)
{
    int sign = (f < 0) ? -1 : 1;
    f = fabs(f);
    int i = (int)f;
    int d = (int)((f - i) * 1000000000);
    LOG_INF("%c%d.%09d", sign < 0 ? '-' : ' ', i, d);
}

/*****************************************************************************
*****************************************************************************/

int mlx90632_config(void)
{
    /*! Configure the temperature computation parameters with the value from the EEPROM */
    uint8_t reg_16bit[2];
    uint8_t reg_32bit[4];
    mlx90632_read_seq_reg(EEPROM_EE_Fa_LSW_Addr, reg_32bit, 4);
    wait_for_twim_transfer();
    int32_t Fa = (reg_32bit[2] << 24) | (reg_32bit[3] << 16) | (reg_32bit[0] << 8) | reg_32bit[1];
    math_param.Fa = (float)Fa * powf(2.0, -46);
    mlx90632_read_seq_reg(EEPROM_EE_Fb_LSW_Addr, reg_32bit, 4);
    wait_for_twim_transfer();
    int32_t Fb = (reg_32bit[2] << 24) | (reg_32bit[3] << 16) | (reg_32bit[0] << 8) | reg_32bit[1];
    math_param.Fb = (float)Fb * powf(2.0, -36);
    mlx90632_read_seq_reg(EEPROM_EE_Ga_LSW_Addr, reg_32bit, 4);
    wait_for_twim_transfer();
    int32_t Ga = (reg_32bit[2] << 24) | (reg_32bit[3] << 16) | (reg_32bit[0] << 8) | reg_32bit[1];
    math_param.Ga = (float)Ga * powf(2.0, -36);
    mlx90632_read_seq_reg(EEPROM_EE_Gb_Addr, reg_16bit, 2);
    wait_for_twim_transfer();
    int16_t Gb = (reg_16bit[0] << 8) | reg_16bit[1];
    math_param.Gb = (float)Gb * powf(2.0, -10);
    mlx90632_read_seq_reg(EEPROM_EE_Ha_Addr, reg_16bit, 2);
    wait_for_twim_transfer();
    int16_t Ha = (reg_16bit[0] << 8) | reg_16bit[1];
    math_param.Ha = (float)Ha * powf(2.0, -14);
    mlx90632_read_seq_reg(EEPROM_EE_Hb_Addr, reg_16bit, 2);
    wait_for_twim_transfer();
    int16_t Hb = (reg_16bit[0] << 8) | reg_16bit[1];
    math_param.Hb = (float)Hb * powf(2.0, -10);
    mlx90632_read_seq_reg(EEPROM_EE_Ea_LSW_Addr, reg_32bit, 4);
    wait_for_twim_transfer();
    int32_t Ea = (reg_32bit[2] << 24) | (reg_32bit[3] << 16) | (reg_32bit[0] << 8) | reg_32bit[1];
    math_param.Ea = (float)Ea * powf(2.0, -16);
    mlx90632_read_seq_reg(EEPROM_EE_Eb_LSW_Addr, reg_32bit, 4);
    wait_for_twim_transfer();
    int32_t Eb = (reg_32bit[2] << 24) | (reg_32bit[3] << 16) | (reg_32bit[0] << 8) | reg_32bit[1];
    math_param.Eb = (float)Eb * powf(2.0, -8);
    mlx90632_read_seq_reg(EEPROM_EE_Ka_Addr, reg_16bit, 2);
    wait_for_twim_transfer();
    int16_t Ka = (reg_16bit[0] << 8) | reg_16bit[1];
    math_param.Ka = (float)Ka * powf(2.0, -10);
    mlx90632_read_seq_reg(EEPROM_EE_P_O_LSW_Addr, reg_32bit, 4);
    wait_for_twim_transfer();
    int32_t P_O = (reg_32bit[2] << 24) | (reg_32bit[3] << 16) | (reg_32bit[0] << 8) | reg_32bit[1];
    math_param.P_O = (float)P_O * powf(2.0, -8);
    mlx90632_read_seq_reg(EEPROM_EE_P_T_LSW_Addr, reg_32bit, 4);
    wait_for_twim_transfer();
    int32_t P_T = (reg_32bit[2] << 24) | (reg_32bit[3] << 16) | (reg_32bit[0] << 8) | reg_32bit[1];
    math_param.P_T = (float)P_T * powf(2.0, -44);
    mlx90632_read_seq_reg(EEPROM_EE_P_G_LSW_Addr, reg_32bit, 4);
    wait_for_twim_transfer();
    int32_t P_G = (reg_32bit[2] << 24) | (reg_32bit[3] << 16) | (reg_32bit[0] << 8) | reg_32bit[1];
    math_param.P_G = (float)P_G * powf(2.0, -20);
    mlx90632_read_seq_reg(EEPROM_EE_P_R_LSW_Addr, reg_32bit, 4);
    wait_for_twim_transfer();
    int32_t P_R = (reg_32bit[2] << 24) | (reg_32bit[3] << 16) | (reg_32bit[0] << 8) | reg_32bit[1];
    math_param.P_R = (float)P_R * powf(2.0, -8);

    LOG_INF("mlx90632_config\n");
    return 0;
}

/*****************************************************************************
*****************************************************************************/

void mlx90632_temp_comp(void)
{
    /*! Pre-calculations */
    float VR_TA = ram_data.RAM_9 + math_param.Gb * ram_data.RAM_6 / 12;
    float AMB = (ram_data.RAM_6/12)/VR_TA * powf(2.0, 19);
    float S;
    if (last_CycPos == 1)
    {
        S = (ram_data.RAM_4 + ram_data.RAM_5) / 2;
    }
    else if (last_CycPos == 2)
    {
        S = (ram_data.RAM_7 + ram_data.RAM_8) / 2;
    }
    else
    {
        return;
    }
    float VR_TO = ram_data.RAM_9 + math_param.Ka * ram_data.RAM_6 / 12;
    float S_TO = (S / 12) / VR_TO * powf(2.0, 19);

    /*! Ambient temperature calculation*/
    AmbientTemperature = math_param.P_O + (AMB - math_param.P_R) / math_param.P_G + math_param.P_T * powf(AMB - math_param.P_R, 2);

    /*! Object temperature calculation*/
    math_param.TAdut = (AMB - math_param.Eb) / math_param.Ea + 25;

    for (int i = 0; i < 3; i++)
    {
        math_param.TOdut = powf(S_TO / (math_param.e * math_param.Fa * math_param.Ha * 
                        (1 + math_param.Ga * (math_param.TOdut - math_param.TO0) + 
                        math_param.Fb * (math_param.TAdut - math_param.TA0))) + 
                        powf(math_param.TAdut + 273.15, 4), 1./4.) - 273.15 - math_param.Hb;
    }

    ObjectTemperature = math_param.TOdut;
}

/*****************************************************************************
*****************************************************************************/

int mlx90632_start_continuous_read(void)
{
    MLX90632_stop_thread = false;
    /*! Start the thread responsible for reading the measurement */
    k_thread_create(&MLX90632_thread, MLX90632_thread_stack, K_THREAD_STACK_SIZEOF(MLX90632_thread_stack),
                    mlx90632_thread_func, NULL, NULL, NULL, MLX90632_PRIO, 0, K_NO_WAIT);
    return 0;
}

/*****************************************************************************
*****************************************************************************/

int mlx90632_stop_continuous_read(void)
{
    MLX90632_stop_thread = true;
    return 0;
}


/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

void mlx90632_thread_func(void *arg1, void *arg2, void *arg3)
{
    format.header = 0xBBBB;
    uint8_t len = 9;
    uint8_t index = 0;
    
    while (!MLX90632_stop_thread)
    {
        /*! Read the status register */
        mlx90632_read_reg(REG_STATUS_Addr, rx_buffer, 2);
        k_sleep(K_MSEC(10));
        mlx_reg.REG_STATUS.w = (rx_buffer[0] << 8) | rx_buffer[1];

        /*! Check if new data is available */
        if (mlx_reg.REG_STATUS.b.NewD == 1)
        {   
            /*! Clear the new data flag */
            mlx_reg.REG_STATUS.b.NewD = 0;

            struct time_values current_time = get_time_values();
            uint32_t rawtime_bin = current_time.rawtime_s_bin;
            uint16_t time_ms_bin = current_time.time_ms_bin;

            /*! Write the status register */
            mlx90632_write_reg(REG_STATUS_Addr, mlx_reg.REG_STATUS.w);
            k_sleep(K_MSEC(10));

            bool ConsecutiveData = false;

            /*! Read the RAM data corresponding to the current cycle */
            uint16_t ram_addr = mlx_reg.REG_STATUS.b.CycPos*3 + RAM_1_Addr;
            mlx90632_read_seq_reg(ram_addr, rx_buffer, 6);
            k_sleep(K_MSEC(10));
            if (mlx_reg.REG_STATUS.b.CycPos == 1)
            {
                /*! Check if the last cycle position was 2 */
                if (last_CycPos == 2)
                {
                    ConsecutiveData = true;
                }
                ram_data.RAM_4 = (rx_buffer[0] << 8) | rx_buffer[1];
                ram_data.RAM_5 = (rx_buffer[2] << 8) | rx_buffer[3];
                ram_data.RAM_6 = (rx_buffer[4] << 8) | rx_buffer[5];
                last_CycPos = 1;
            }
            else
            {
                /*! Check if the last cycle position was 1 */
                if (last_CycPos == 1)
                {
                    ConsecutiveData = true;
                }
                ram_data.RAM_7 = (rx_buffer[0] << 8) | rx_buffer[1];
                ram_data.RAM_8 = (rx_buffer[2] << 8) | rx_buffer[3];
                ram_data.RAM_9 = (rx_buffer[4] << 8) | rx_buffer[5];
                last_CycPos = 2;
            }

            if (ConsecutiveData)
            {
                /*! Compute the temperature */
                mlx90632_temp_comp();
                log_float(AmbientTemperature);
                log_float(ObjectTemperature); 

                format.rawtime_bin = rawtime_bin;
                format.time_ms_bin = time_ms_bin;
                format.len = len;
                format.index = index++;
                format.AmbientTemperature = AmbientTemperature;
                format.ObjectTemperature = ObjectTemperature;


                int ret = storage_write_to_fifo((uint8_t *)&format, sizeof(format));
                if (ret != 0)
                {
                    LOG_ERR("Error writing to flash\n");
                }

                receive_sensor_data((uint8_t *)&format, sizeof(format));
            }
        }
        else 
        {
            LOG_INF("No new data\n");
        }
        k_sleep(K_MSEC(100));
    }
    k_thread_abort(k_current_get());
}


/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/
