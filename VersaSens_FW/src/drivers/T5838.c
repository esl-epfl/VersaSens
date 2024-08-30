/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : T5838.c                                                      **
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
* @file   T5838.c
* @date   DD/MM/YY
* @brief  This is the main header of T5838.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _T5838_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include "storage.h"
#include <nrfx_uarte.h>
#include <stdlib.h>
#include "T5838.h"
#include <nrfx_pdm.h>
#include <nrfx_clock.h>
#include <zephyr/irq.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "versa_time.h"

#include "opus.h"
#include "opus_types.h"
#include "opus_private.h"

/****************************************************************************/
/**                                                                        **/
/*                        DEFINITIONS AND MACROS                            */
/**                                                                        **/
/****************************************************************************/

LOG_MODULE_REGISTER(t5838, LOG_LEVEL_INF);

#define PDM_BUFFER_SIZE        240

#define T5838_PDM(idx)	       DT_NODELABEL(pdm##idx)

/****************************************************************************/
/**                                                                        **/
/*                        TYPEDEFS AND STRUCTURES                           */
/**                                                                        **/
/****************************************************************************/

struct {
    int16_t t5838_frame1[PDM_BUFFER_SIZE];
    int16_t t5838_frame2[PDM_BUFFER_SIZE];
    int16_t t5838_frame3[PDM_BUFFER_SIZE];
}t5838_frames;

/****************************************************************************/
/**                                                                        **/
/*                      PROTOTYPES OF LOCAL FUNCTIONS                       */
/**                                                                        **/
/****************************************************************************/

/**
* @brief This function is the thread function that saves the frames to the fifo buffer
*
* @param arg1 : argument 1, not used
* @param arg2 : argument 2, not used
* @param arg3 : argument 3, not used
*
* @retval None
*/
void t5838_save_thread_func(void *arg1, void *arg2, void *arg3);

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED VARIABLES                             */
/**                                                                        **/
/****************************************************************************/

int16_t *t5838_frame = NULL;

bool t5838_frame1_new = false;
bool t5838_frame2_new = false;
bool t5838_frame3_new = false;

/****************************************************************************/
/**                                                                        **/
/*                            GLOBAL VARIABLES                              */
/**                                                                        **/
/****************************************************************************/

K_THREAD_STACK_DEFINE(T5838_save_thread_stack, 100000);
struct k_thread T5838_save_thread;

volatile bool T5838_save_stop_thread = false;

T5838_StorageFormat T5838_storage;

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

static void pdm_handler(nrfx_pdm_evt_t const *event)
{
    /*! Check if a buffer is requested */
    if (event->buffer_requested) {
        if (t5838_frame == t5838_frames.t5838_frame1)
        {
            t5838_frame = t5838_frames.t5838_frame2;
        }
        else if (t5838_frame == t5838_frames.t5838_frame2)
        {
            t5838_frame = t5838_frames.t5838_frame3;
        }
        else if (t5838_frame == t5838_frames.t5838_frame3 || t5838_frame == NULL)
        {
            t5838_frame = t5838_frames.t5838_frame1;
        }

        if(event->buffer_released == t5838_frames.t5838_frame1)
        {
            t5838_frame1_new = true;
        }
        else if(event->buffer_released == t5838_frames.t5838_frame2)
        {
            t5838_frame2_new = true;
        }
        else if(event->buffer_released == t5838_frames.t5838_frame3)
        {
            t5838_frame3_new = true;
        }

        nrfx_pdm_buffer_set(t5838_frame, PDM_BUFFER_SIZE);
    }
}

/*****************************************************************************
*****************************************************************************/

int t5838_set_freq(uint32_t freq)
{
    nrfx_err_t status;
    (void)status; 

    /*! Get the default PDM configuration */
    nrfx_pdm_config_t pdm_config = NRFX_PDM_DEFAULT_CONFIG(PDM_CLK_PIN, PDM_DATA_PIN);
    pdm_config.clock_freq = NRF_PDM_FREQ_1000K / 1000.0 * freq;

    /*! Initialize the PDM peripheral */
    status = nrfx_pdm_reconfigure(&pdm_config);

    return (status == NRFX_SUCCESS)? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int t5838_start(void)
{
    nrfx_err_t status;
    (void)status; 

    /*! Start the PDM peripheral */
    status = nrfx_pdm_start();

    t5838_start_saving();

    return (status == NRFX_SUCCESS)? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int t5838_stop(void)
{
    nrfx_err_t status;
    (void)status; 

    t5838_stop_saving();

    /*! Stop the PDM peripheral */
    status = nrfx_pdm_stop();

    return (status == NRFX_SUCCESS)? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int t5838_init(void)
{
    nrfx_err_t status;
    (void)status; 

    /*! f_out = 12288000 */
    /*! FREQ_VALUE = 2^16 * ((12 * f_out / 32M) - 4) */
    uint16_t freq_value = 39846;

    /*! Set the audio clock frequency */
    nrfx_clock_enable();
    nrfx_clock_start(NRF_CLOCK_DOMAIN_HFCLKAUDIO);
    nrfx_clock_hfclkaudio_config_set(freq_value);

    /*! Set the PDM configuration */
    nrfx_pdm_config_t pdm_config = NRFX_PDM_DEFAULT_CONFIG(PDM_CLK_PIN, PDM_DATA_PIN);
    pdm_config.mode = NRF_PDM_MODE_MONO;
    pdm_config.edge = NRF_PDM_EDGE_LEFTFALLING;
    // pdm_config.clock_freq = NRF_PDM_FREQ_1000K * 0.768;
    pdm_config.clock_freq = 260300800;
    pdm_config.ratio = NRF_PDM_RATIO_64X;
    pdm_config.mclksrc = NRF_PDM_MCLKSRC_ACLK;

    /*! Initialize the PDM peripheral */
    status = nrfx_pdm_init(&pdm_config, pdm_handler);

    IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PDM0), IRQ_PRIO_LOWEST, nrfx_pdm_irq_handler, 0)

    return (status == NRFX_SUCCESS)? 0 : -1;
}

/****************************************************************************/
/****************************************************************************/

void t5838_start_saving(void)
{
    T5838_save_stop_thread = false;
    k_thread_create(&T5838_save_thread, T5838_save_thread_stack, K_THREAD_STACK_SIZEOF(T5838_save_thread_stack), 
                    t5838_save_thread_func, NULL, NULL, NULL, T5838_PRIO, 0, K_NO_WAIT);
}

/****************************************************************************/
/****************************************************************************/

void t5838_stop_saving(void)
{
    T5838_save_stop_thread = true;
    // k_thread_abort(&T5838_save_thread);
}

/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

void t5838_save_thread_func(void *arg1, void *arg2, void *arg3)
{
    int error;
    OpusEncoder *enc;
    enc = opus_encoder_create(12000, 1, OPUS_APPLICATION_AUDIO, &error);
    if (error != OPUS_OK)
    {
        LOG_ERR("Error: %s\n", opus_strerror(error));
    }
    else
    {
        LOG_INF("Opus encoder initialized\n");
    }
    error = opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY(1), OPUS_SET_VBR(1));
    if (error != OPUS_OK)
    {
        LOG_ERR("Error: %s\n", opus_strerror(error));
    }
    else
    {
        LOG_INF("Opus encoder configured\n");
    }
    int16_t CompressedFrame[PDM_BUFFER_SIZE] = {0};
    uint8_t len = 0;
    uint8_t index = 0;
    T5838_storage.header = 0xAAAA;

    while (!T5838_save_stop_thread)
    {
        struct time_values current_time = get_time_values();
        uint32_t rawtime_bin = current_time.rawtime_s_bin;
        uint16_t time_ms_bin = current_time.time_ms_bin;

        if (t5838_frame1_new)
        {
            t5838_frame1_new = false;
            //start timmer
            int start = k_uptime_get();

            len = (uint8_t)opus_encode(enc, t5838_frames.t5838_frame1, PDM_BUFFER_SIZE, CompressedFrame, PDM_BUFFER_SIZE) * sizeof(int16_t);
            int end = k_uptime_get();
            printk("Time to encode: %d\n", end - start);
            T5838_storage.rawtime_bin = rawtime_bin;
            T5838_storage.time_ms_bin = time_ms_bin;
            T5838_storage.len = len+1;
            T5838_storage.index = index;
            index++;
            memcpy(T5838_storage.data, CompressedFrame, len);

            int ret = storage_write_to_fifo((uint8_t *)&T5838_storage, len + 10);
            if (ret != 0)
            {
                LOG_ERR("Error writing to flash\n");
            }
            receive_sensor_data((uint8_t *)&T5838_storage, len + 10);

            printk("Frame 1, len : %d\n", len);
        }
        else if (t5838_frame2_new)
        {
            t5838_frame2_new = false;

            len = (uint8_t)opus_encode(enc, t5838_frames.t5838_frame2, PDM_BUFFER_SIZE, CompressedFrame, PDM_BUFFER_SIZE) * sizeof(int16_t);
            T5838_storage.rawtime_bin = rawtime_bin;
            T5838_storage.time_ms_bin = time_ms_bin;
            T5838_storage.len = len+1;
            T5838_storage.index = index;
            index++;
            memcpy(T5838_storage.data, CompressedFrame, len);

            int ret = storage_write_to_fifo((uint8_t *)&T5838_storage, len + 10);
            if (ret != 0)
            {
                LOG_ERR("Error writing to flash\n");
            }
            receive_sensor_data((uint8_t *)&T5838_storage, len + 10);

            printk("Frame 2, len : %d\n", len);
        }
        else if (t5838_frame3_new)
        {
            t5838_frame3_new = false;

            len = (uint8_t)opus_encode(enc, t5838_frames.t5838_frame3, PDM_BUFFER_SIZE, CompressedFrame, PDM_BUFFER_SIZE) * sizeof(int16_t);
            T5838_storage.rawtime_bin = rawtime_bin;
            T5838_storage.time_ms_bin = time_ms_bin;
            T5838_storage.len = len+1;
            T5838_storage.index = index;
            index++;
            memcpy(T5838_storage.data, CompressedFrame, len);

            int ret = storage_write_to_fifo((uint8_t *)&T5838_storage, len + 10);
            if (ret != 0)
            {
                LOG_ERR("Error writing to flash\n");
            }
            receive_sensor_data((uint8_t *)&T5838_storage, len + 10);

            printk("Frame 3, len : %d\n", len);
            
        }
        k_sleep(K_MSEC(10));
    }
    k_thread_abort(k_current_get());
}

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/