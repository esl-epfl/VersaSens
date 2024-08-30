/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : T5838.h                                                   **
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
* @file   T5838.h
* @date   DD/MM/YY
* @brief  This is the main header of T5838.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _T5838_H
#define _T5838_H

/****************************************************************************/
/**                                                                        **/
/**                            MODULES USED                                **/
/**                                                                        **/
/****************************************************************************/

#include <zephyr/types.h>
#include "thread_config.h"

/****************************************************************************/
/**                                                                        **/
/**                       DEFINITIONS AND MACROS                           **/
/**                                                                        **/
/****************************************************************************/

/*! PDM pin configuration */
#define PDM_CLK_PIN         43
#define PDM_DATA_PIN        35

/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

/*! Foramt of the data to be stored in the flash */
typedef struct {
    int16_t header;
    int32_t rawtime_bin;
    int16_t time_ms_bin;
    uint8_t len;
    uint8_t index;
    uint8_t data[240];
} __attribute__((packed)) T5838_StorageFormat;

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/

#ifndef _T5838_C_SRC



#endif  /* _T5838_C_SRC */

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

/**
 * @brief  This function starts the T5838 driver
 * 
 * @param  None
 * 
 * @retval 0 if successful, -1 otherwise
 */
int t5838_start(void);

/**
 * @brief  This function stops the T5838 driver
 * 
 * @param  None
 * 
 * @retval 0 if successful, -1 otherwise
 */
int t5838_stop(void);

/**
 * @brief  This function initializes the T5838 driver
 * 
 * @param  None
 * 
 * @retval 0 if successful, -1 otherwise
 */
int t5838_init(void);

/**
 * @brief  This function set up the frequency for the T5838
 * 
 * @param  freq: the frequency to set [kHz]
 * 
 * @retval 0 if successful, -1 otherwise
 */
int t5838_set_freq(uint32_t freq);

/**
 * @brief  This function starts the continuous saving of the data
 * 
 * @param  None
 * 
 * @retval None
 */
void t5838_start_saving(void);

/**
 * @brief  This function stops the continuous saving of the data
 * 
 * @param  None
 * 
 * @retval None
 */
void t5838_stop_saving(void);

/****************************************************************************/
/**                                                                        **/
/**                          INLINE FUNCTIONS                              **/
/**                                                                        **/
/****************************************************************************/



#endif /* _T5838_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/