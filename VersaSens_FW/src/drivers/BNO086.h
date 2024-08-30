/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : BNO086.h                                                   **
** version  : 1                                                            **
** date     : DD/MM/21                                                     **
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
* @file   BNO086.h
* @date   DD/MM/YY
* @brief  This is the main header of BNO086.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _BNO086_H
#define _BNO086_H

/****************************************************************************/
/**                                                                        **/
/**                            MODULES USED                                **/
/**                                                                        **/
/****************************************************************************/

// #include "sdk_common.h"
#include <zephyr/types.h>

/****************************************************************************/
/**                                                                        **/
/**                       DEFINITIONS AND MACROS                           **/
/**                                                                        **/
/****************************************************************************/

#define BNO086_RVC_MSG_LENGTH 19*10
#define BNO086_RVC_RX_BUF_LENGTH BNO086_RVC_MSG_LENGTH

#define BNO086_RVC_FRAME_PERIOD_MS	10

#define IMU_HEADER_BYTE_0	0xaa
#define IMU_PREFIX_BYTE		0xaa
#define IMU_PREFIX_LENGTH	1
#define IMU_FRAME_LENGTH	BNO086_RVC_MSG_LENGTH

#define BNO_RST_N_Pin       23

#define UARTE_INST_IDX      1
#define UARTE_RX_PIN        21
#define UARTE_TX_PIN        38

/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

typedef union {
    struct {
        uint8_t header_lsb;
        uint8_t header_msb;
        uint8_t index;
        uint8_t yaw_lsb;
        uint8_t yaw_msb;
        uint8_t pitch_lsb;
        uint8_t pitch_msb;
        uint8_t roll_lsb;
        uint8_t roll_msb;
        uint8_t acc_x_lsb;
        uint8_t acc_x_msb;
        uint8_t acc_y_lsb;
        uint8_t acc_y_msb;
        uint8_t acc_z_lsb;
        uint8_t acc_z_msb;
        uint8_t motion_intent;
        uint8_t motion_request;
        uint8_t rsvd;
        uint8_t checksum;
    } B;
    uint8_t frame[19];
}__attribute__((packed)) bno086_frame_t;

typedef struct {
    int16_t header;
    int32_t rawtime_bin;
    int16_t time_ms_bin;
    uint8_t len;
    uint8_t data[13*10];
} __attribute__((packed)) BNO086_StorageFormat;


/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/

#ifndef _BNO086_C_SRC



#endif  /* _BNO086_C_SRC */

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

/**
 * @brief Initializes the BNO086 sensor.
 * 
 * @details This function initializes the UARTE peripheral for communication with the BNO086 sensor.
 *          It configures the UARTE instance, sets up the UARTE handler, and connects the UARTE interrupt.
 * 
 * @return 0 if initialization was successful, -1 otherwise.
 */
int bno086_init(void);

/**
 * @brief Resets the BNO086 sensor.
 * 
 * @details This function resets the BNO086 sensor by manipulating the reset pin.
 *          It first sets the reset pin to low, waits for 100 milliseconds, and then 
 *          sets the reset pin to high.
 */
void bno086_reset(void);

/**
 * @brief Starts the data stream from the BNO086 sensor.
 * 
 * @details This function starts the UARTE instance to receive data from the BNO086 sensor.
 *           The received data is stored in the `bno_frame` array. 
 *          
 * @return 0 if the UARTE instance is started successfully, -1 otherwise.
 */
int bno086_start_stream(void);

/**
 * @brief Stops the data stream from the BNO086 sensor.
 * 
 * @details Cancels any pending start operations and aborts the UARTE instance 
 *          receiving data from the BNO086 sensor. Logs the stop event.
 * 
 * @return 0 if the UARTE instance is stopped successfully, -1 otherwise.
 */
int bno086_stop_stream(void);

/**
 * @brief Finds the start of the frame in the BNO086 sensor data.
 * 
 * @details This function iterates over the `bno_frame` array and looks for two consecutive 
 *          bytes that match the `IMU_HEADER_BYTE_0`. This byte sequence indicates the start 
 *          of a frame in the BNO086 sensor data.
 * 
 * @return The index of the first byte of the frame start sequence if found, -1 otherwise.
 */
int bno086_find_frame_start(void);


/****************************************************************************/
/**                                                                        **/
/**                          INLINE FUNCTIONS                              **/
/**                                                                        **/
/****************************************************************************/



#endif /* _BNO086_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/