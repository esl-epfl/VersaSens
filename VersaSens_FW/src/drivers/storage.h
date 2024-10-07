/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : storage.h                                                   **
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
* @file   storage.h
* @date   DD/MM/YY
* @brief  This is the main header of storage.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _STORAGE_H
#define _STORAGE_H

/****************************************************************************/
/**                                                                        **/
/**                            MODULES USED                                **/
/**                                                                        **/
/****************************************************************************/

#include <zephyr/types.h>
#include <zephyr/fs/fs.h>
#include "thread_config.h"

/****************************************************************************/
/**                                                                        **/
/**                       DEFINITIONS AND MACROS                           **/
/**                                                                        **/
/****************************************************************************/    

/*! Flash memory write configuration */

/* Create a new file */
#define STORAGE_CREATE        FS_O_CREATE | FS_O_WRITE
/* Overwrite an existing file */
#define STORAGE_OVERWRITE     FS_O_WRITE
/* Append to an existing file */
#define STORAGE_APPEND        FS_O_CREATE | FS_O_APPEND | FS_O_WRITE

/* sync period in seconds */
#define PERIOD_SYNC 5

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

#ifndef _STORAGE_C_SRC



#endif  /* _STORAGE_C_SRC */

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

/**
 * @brief Initialize the storage memory
 *
 * @return int - status of the operation (0 for success, non-zero for failure)
 */
int storage_init(void);

/**
 * @brief Read data from a file in a specific directory in the storage memory
 *
 * @param file_name - name of the file to read from
 * @param path - directory where the file is located, empty string for root directory
 * @param data - pointer to buffer where read data will be stored
 * @param size - number of bytes to read
 * 
 * @return int - status of the operation (0 for success, non-zero for failure)
 */
int storage_read(const char *file_name, const char *path, uint8_t *data, size_t size);

/**
 * @brief Write data to a file in a specific directory in the storage memory
 *
 * @param file - file structure
 * @param data - pointer to buffer containing data to write
 * @param size - number of bytes to write
 * 
 * @return int - status of the operation (0 for success, non-zero for failure)
 */
int storage_write_to_fifo(uint8_t *data, size_t size);

/**
 * @brief List the contents of a directory in the storage memory
 *
 * @param path - directory to list, empty string for root directory
 * 
 * @return int - status of the operation (0 for success, non-zero for failure)
 */
int lsdir(const char *path);

/**
 * @brief Erase all data in the flash memory
 *
 * @return int - status of the operation (0 for success, non-zero for failure)
 */
int flash_erase_all(void);

/**
 * @brief Create a new directory in the storage memory
 *
 * @param path - name of the directory to create
 * 
 * @return int - status of the operation (0 for success, non-zero for failure)
 */
int storage_create_dir(const char *dir_name);


/**
 * @brief Open a file in the storage memory for measurement storage
 * 
 * @return void
 */
void storage_open_file(int conf);

/**
 * @brief Close the measurement file in the storage memory
 *
 * @return void
 */
void storage_close_file(void);

/**
 * @brief Erase the file used for the measurements
 * 
 * @return void
 */
void erase_file(void);

/**
 * @brief Get the write failed flag
 * 
 * @return bool - true if the write operation failed, false otherwise
 */
bool get_write_failed(void);


/****************************************************************************/
/**                                                                        **/
/**                          INLINE FUNCTIONS                              **/
/**                                                                        **/
/****************************************************************************/


#endif /* _STORAGE_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/