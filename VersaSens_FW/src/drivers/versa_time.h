/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : versa_time.h                                                   **
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
* @file   versa_time.h
* @date   DD/MM/YY
* @brief  This is the main header of versa_time.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _VERSA_TIME_H
#define _VERSA_TIME_H

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

/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

struct time_values
{
    uint32_t rawtime_s_bin;
    uint16_t time_ms_bin;
};

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/

#ifndef _VERSA_TIME_C_SRC



#endif  /* _VERSA_TIME_C_SRC */

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

/**
 * @brief Retrieve the current time and system uptime.
 *
 * @details function retrieves the current time in seconds since the Unix epoch (1970-01-01 00:00:00 UTC)
 *          and the system uptime in milliseconds. It then calculates the current time in milliseconds by
 *          adding the uptime to the current time in seconds.
 *
 *
 * @return struct time_values A struct containing the raw time in seconds (`rawtime_s_bin`) and
 *         the current time in milliseconds (`time_ms_bin`).
 */
struct time_values get_time_values(void);

/**
 * @brief Synchronize the time with the current time.
 *
 * @details This function synchronizes the internal time with the current time
 */
void sync_time(void);

/****************************************************************************/
/**                                                                        **/
/**                          INLINE FUNCTIONS                              **/
/**                                                                        **/
/****************************************************************************/



#endif /* _VERSA_TIME_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/