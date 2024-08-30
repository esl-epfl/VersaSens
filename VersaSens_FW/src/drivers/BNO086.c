/*
                              *******************
******************************* C SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                    **
** filename : BNO086.c                                                     **
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
* @file   BNO086.c
* @date   DD/MM/YY
* @brief  This is the main header of BNO086.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#define _BNO086_C_SRC

/****************************************************************************/
/**                                                                        **/
/*                             MODULES USED                                 */
/**                                                                        **/
/****************************************************************************/

#include <stdlib.h>
#include "BNO086.h"
#include <nrfx_uarte.h>
#include <nrfx_gpiote.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "storage.h"
#include "versa_time.h"
#include "versa_ble.h"

LOG_MODULE_REGISTER(bno086, LOG_LEVEL_INF);

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

/**
 * @brief Handles the delayed start of BNO086 sensor.
 * 
 * @details This function is a work handler for the BNO086 sensor. It's called 
 * when a delayed start work item is processed by the system workqueue. The work 
 * item could be a request to start the sensor after a delay.
 *
 * @param work A pointer to the work item structure.
 */
void work_handler_bno086_delayed_start(struct k_work *work);

/**
 * @brief Handles UART events.
 * 
 * @details This function is a handler for UART events. It's called when a UART 
 * event occurs, such as data received or transmission completed. The specific 
 * event can be determined by inspecting the p_event parameter.
 *
 * @param p_event A pointer to the UART event structure.
 * @param p_context A pointer to a context structure that's passed to the 
 *                  handler along with the event. The context can contain 
 *                  additional data or state related to the event.
 */
static void uarte_handler(nrfx_uarte_event_t const * p_event, void * p_context);

/**
 * @brief Saves the BNO086 sensor frame to flash.
 * 
 * @details This function is a thread function that saves the BNO086 sensor frame
 * to flash. It's called when the BNO086 sensor frame is received and ready to be saved.
 * The function writes the frame to flash and then starts a new reception.
 * 
 * @param arg1 A pointer to the first argument passed to the thread.
 * @param arg2 A pointer to the second argument passed to the thread.
 * @param arg3 A pointer to the third argument passed to the thread.
 */
void bno086_save_thread_func(void *arg1, void *arg2, void *arg3);

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

/*! Work item to start the BNO086 sensor after a delay */
K_WORK_DELAYABLE_DEFINE(start_delayed, work_handler_bno086_delayed_start);

/*! BNO086 save thread */
K_THREAD_STACK_DEFINE(BNO086_save_thread_stack, 1024);
struct k_thread BNO086_save_thread;

/*! BNO086 save thread semaphore */
K_SEM_DEFINE(BNO086_save_thread_sem, 0, 1);

/*! UARTE Instance*/
nrfx_uarte_t uarte_inst = NRFX_UARTE_INSTANCE(UARTE_INST_IDX);

/*! BNO086 Frames */
struct {
    uint8_t bno_frame1[BNO086_RVC_MSG_LENGTH];
    uint8_t bno_frame2[BNO086_RVC_MSG_LENGTH];
}__attribute__((packed)) bno_frames;

/*! Active BNO086 Frame */
uint8_t *bno_frame = bno_frames.bno_frame1;

/*! BNO086 Frame Flags */
bool bno086_frame1_new = false;
bool bno086_frame2_new = false;

bool bno086_stop_save = false;

BNO086_StorageFormat bno_storage;

/****************************************************************************/
/**                                                                        **/
/*                           EXPORTED FUNCTIONS                             */
/**                                                                        **/
/****************************************************************************/

int bno086_start_stream(void)
{
    nrfx_err_t status;
    (void)status;

    /* Reset the frame flags and set the active frame */
    bno086_frame1_new = false;
    bno086_frame2_new = false;
    bno_frame = bno_frames.bno_frame1;
    bno086_stop_save = false;

    /*! Start the BNO086 save thread */
    k_thread_create(&BNO086_save_thread, BNO086_save_thread_stack, K_THREAD_STACK_SIZEOF(BNO086_save_thread_stack),
                    bno086_save_thread_func, NULL, NULL, NULL, BNO086_PRIO, 0, K_NO_WAIT);

    /*! Start a UART reception */
    status = nrfx_uarte_rx(&uarte_inst, bno_frame, BNO086_RVC_MSG_LENGTH);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    bno_frame = bno_frames.bno_frame2;

    status = nrfx_uarte_rx(&uarte_inst, bno_frame, BNO086_RVC_MSG_LENGTH);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    LOG_INF("UARTE instance started\n");

    return (status == NRFX_SUCCESS)? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

int bno086_stop_stream(void)
{
    nrfx_err_t status;
    (void)status;

    bno086_stop_save = true;

    /*! Cancel the delayed work item */
    k_work_cancel_delayable(&start_delayed);

    /*! Stop any ongoing reception */
    status = nrfx_uarte_rx_abort(&uarte_inst, 0, 0);
    NRFX_ASSERT(status == NRFX_SUCCESS);
    status = nrfx_uarte_rx_abort(&uarte_inst, 0, 0);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    LOG_INF("UARTE instance stopped\n");

    return (status == NRFX_SUCCESS)? 0 : -1;
}

/*****************************************************************************
*****************************************************************************/

static void uarte_handler(nrfx_uarte_event_t const * p_event, void * p_context)
{

    nrfx_err_t status;
    (void)status;

    nrfx_uarte_t * p_inst = p_context;
    /*! Handle the UARTE transmission event done */
    if (p_event->type == NRFX_UARTE_EVT_TX_DONE)
    {}

    /*! Handle the UARTE reception event done */
    else if (p_event->type == NRFX_UARTE_EVT_RX_DONE)
    {
        /*! Cancel the delayed work item */
        k_work_cancel_delayable(&start_delayed);
        /*! Check if the frame is aligned */
        if (bno086_find_frame_start() != 0)
        {
            if(bno086_stop_save)
            {
                return;
            }

            LOG_INF("Frame not aligned %d", bno086_find_frame_start());
            nrfx_uarte_rx_abort(p_inst, 0, 0);
            /*! Schedule the work to start the UARTE with delay to get the frame start */
            if (bno086_find_frame_start() == 1){
                k_work_schedule(&start_delayed, K_MSEC(9));
            }
            else{
                k_work_schedule(&start_delayed, K_MSEC(10));
            }
            return;
        }

        /*! Update  the active frame and the frame flags */
        if(bno_frame == bno_frames.bno_frame1)
        {
            bno086_frame1_new = true;
            bno_frame = bno_frames.bno_frame2;
        }
        else
        {
            bno086_frame2_new = true;
            bno_frame = bno_frames.bno_frame1;
        }

        if(bno086_stop_save)
        {
            return;
        }
        
        /*! Start a new reception */
        status = nrfx_uarte_rx(p_inst, bno_frame, BNO086_RVC_MSG_LENGTH);
        NRFX_ASSERT(status == NRFX_SUCCESS);

        k_sem_give(&BNO086_save_thread_sem);
        return;
    }
    
}

/*****************************************************************************
*****************************************************************************/

int bno086_find_frame_start(void)
{
    /*! Search for the frame start */
    for (size_t i = 1; i < BNO086_RVC_MSG_LENGTH; i++)
    {
        if (*(bno_frame + i) == IMU_HEADER_BYTE_0 && *(bno_frame + i - 1) == IMU_HEADER_BYTE_0)
        {
            /*! Return the index of the frame start */
            return i-1;
        }
    }

    return -1;
}

/****************************************************************************
 ****************************************************************************/

int bno086_init(void)
{
    nrfx_err_t status;
    (void)status; 

    /*! Get the default UARTE configuration */
    nrfx_uarte_config_t uarte_config = NRFX_UARTE_DEFAULT_CONFIG(UARTE_TX_PIN, UARTE_RX_PIN);
    uarte_config.p_context = &uarte_inst;

    /*! Initialize the UARTE instance with the given configuration */
    status = nrfx_uarte_init(&uarte_inst, &uarte_config, uarte_handler);

    /*! Connect the UARTE interrupt to the UARTE instance handler */
    IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_UARTE_INST_GET(UARTE_INST_IDX)), IRQ_PRIO_LOWEST,
                       NRFX_UARTE_INST_HANDLER_GET(UARTE_INST_IDX), 0);

    LOG_INF("UARTE instance initialized\n");

    return (status == NRFX_SUCCESS)? 0 : -1;
}

/****************************************************************************
 ****************************************************************************/

void bno086_reset(void){
    //set reset pin to output
    nrf_gpio_cfg_output(BNO_RST_N_Pin);
    //set reset pin to low
    nrf_gpio_pin_clear(BNO_RST_N_Pin);
    k_sleep(K_MSEC(100));
    //set reset pin to high
    nrf_gpio_pin_set(BNO_RST_N_Pin);
}

/****************************************************************************/
/**                                                                        **/
/*                            LOCAL FUNCTIONS                               */
/**                                                                        **/
/****************************************************************************/

void work_handler_bno086_delayed_start(struct k_work *work)
{
    nrfx_err_t status;
    (void)status;
    /*! Cancel the delayed work item */
    nrfx_uarte_rx_abort(&uarte_inst, 0, 0);
    /*! Start a UART reception */
    status = nrfx_uarte_rx(&uarte_inst, bno_frame, BNO086_RVC_MSG_LENGTH);
    NRFX_ASSERT(status == NRFX_SUCCESS);
}

void bno086_save_thread_func(void *arg1, void *arg2, void *arg3)
 {
    nrfx_err_t status;
    nrfx_uarte_t * p_inst = &uarte_inst;

    bno_storage.header = 0xCCCC;
    bno_storage.len = 130;

    uint8_t write_packet[13*10];

    while(!bno086_stop_save)
    {
        k_sem_take(&BNO086_save_thread_sem, K_FOREVER);

        uint8_t frame_write[BNO086_RVC_MSG_LENGTH];

        //write the non active frame to flash
        if(bno_frame == bno_frames.bno_frame1)
        {
            memcpy(frame_write, bno_frames.bno_frame2, BNO086_RVC_MSG_LENGTH);
        }
        else
        {
            memcpy(frame_write, bno_frames.bno_frame1, BNO086_RVC_MSG_LENGTH);
        }

        struct time_values current_time = get_time_values();
        uint32_t rawtime_bin = current_time.rawtime_s_bin;
        uint16_t time_ms_bin = current_time.time_ms_bin;

        bno_storage.rawtime_bin = rawtime_bin;
        bno_storage.time_ms_bin = time_ms_bin;

        for (size_t i = 0; i < 10; i++)
        {
            memcpy(write_packet + i*13, frame_write + 2 + i*19, 13);
        }

        memcpy(bno_storage.data, write_packet, 13*10);

        int ret = storage_write_to_fifo((uint8_t *)&bno_storage, sizeof(BNO086_StorageFormat));
        if(ret != 0)
        {
            LOG_INF("Flash write failed");
        }

        receive_sensor_data((uint8_t *)&bno_storage, sizeof(BNO086_StorageFormat));

        bno086_frame_t *frame = (bno086_frame_t *)frame_write;
        LOG_INF("Index: %02x", frame->B.index);
        k_sleep(K_MSEC(1));
    }
    k_thread_abort(k_current_get());
 }

/****************************************************************************/
/**                                                                        **/
/*                                 EOF                                      */
/**                                                                        **/
/****************************************************************************/