/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : thread_config.h                                                   **
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
* @file   thread_config.h
* @date   DD/MM/YY
* @brief  
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _THREAD_CONGIG_H
#define _THREAD_CONGIG_H

/****************************************************************************/
/**                                                                        **/
/**                            MODULES USED                                **/
/**                                                                        **/
/****************************************************************************/

#include <zephyr/types.h>
#include <zephyr/kernel.h>

/****************************************************************************/
/**                                                                        **/
/**                       DEFINITIONS AND MACROS                           **/
/**                                                                        **/
/****************************************************************************/

#define FIFO_BUFFER_SIZE       4096

/*Priority of the ADS1298 thread */
#define ADS1298_PRIO        K_PRIO_PREEMPT(4)

/*Priority of the BNO086 thread */
#define BNO086_PRIO         K_PRIO_PREEMPT(3)

/*Priority of the MAX30001 thread */
#define MAX30001_PRIO       K_PRIO_PREEMPT(4)

/*Priority of the MAX77658 thread */
#define MAX77658_PRIO       K_PRIO_PREEMPT(6)

/*Priority of the MAX86178 thread */
#define MAX86178_PRIO       K_PRIO_PREEMPT(5)

/*Priority of the MLX90632 thread */
#define MLX90632_PRIO       K_PRIO_PREEMPT(7)

/*Priority of the T5838 thread */
#define T5838_PRIO          K_PRIO_PREEMPT(8)

/*Priority of the Bluetooth thread */
#define BLE_PRIO            K_PRIO_PREEMPT(11)

/*Priority of the storage thread */
#define STORAGE_PRIO        K_PRIO_PREEMPT(10)



/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/**                          INLINE FUNCTIONS                              **/
/**                                                                        **/
/****************************************************************************/



#endif /* _THREAD_CONGIG_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/