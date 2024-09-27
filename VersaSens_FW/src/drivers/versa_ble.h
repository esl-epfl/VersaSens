/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : versa_ble.h                                                   **
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
* @file   versa_ble.h
* @date   DD/MM/YY
* @brief  
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _VERSA_BLE_H
#define _VERSA_BLE_H

/****************************************************************************/
/**                                                                        **/
/**                            MODULES USED                                **/
/**                                                                        **/
/****************************************************************************/

/****************************************************************************/
/**                                                                        **/
/**                       DEFINITIONS AND MACROS                           **/
/**                                                                        **/
/****************************************************************************/

/* Custom service UUID */
#define BT_UUID_CUSTOM_SERVICE \		
	BT_UUID_128_ENCODE(0xE11D2E00, 0x04AB, 0x4DA5, 0xB66A, 0xEECB738F90F3)

/* Custom characteristic UUID */
#define BT_UUID_CUSTOM_CHARA \ 	
	BT_UUID_128_ENCODE(0xE11D2E01, 0x04AB, 0x4DA5, 0xB66A, 0xEECB738F90F3)

/* Custom characteristic UUID */
#define BT_UUID_CUSTOM_CHARA_WRITE \	
    BT_UUID_128_ENCODE(0xE11D2E02, 0x04AB, 0x4DA5, 0xB66A, 0xEECB738F90F3)

/* Custom characteristic UUID */
#define BT_UUID_CUSTOM_CHARA_STATUS \	
    BT_UUID_128_ENCODE(0xE11D2E03, 0x04AB, 0x4DA5, 0xB66A, 0xEECB738F90F3)

#define BT_UUID_CUSTOM_CHARA_CMD \
    BT_UUID_128_ENCODE(0xE11D2E04, 0x04AB, 0x4DA5, 0xB66A, 0xEECB738F90F3)	

/* Maximum size of the data from the sensor */
#define MAX_DATA_SIZE 244

/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

struct sensor_data {
	void *fifo_reserved;  // reserved for use by k_fifo
	uint8_t data[MAX_DATA_SIZE];  // sensor data
	size_t size;  // size of the data
};

struct battery_data {
    void *fifo_reserved;  // reserved for use by k_fifo
    uint16_t data;  // battery data
};


/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/

#ifndef _VERSA_BLE_C_SRC



#endif  /* _VERSA_BLE_C_SRC */

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

/*
    * @brief Start the BLE service and advertise the custom service
    * 
    * @return int 0 if the BLE service started correctly, -1 otherwise
*/
int start_ble(void);

/*
    * @brief Send the sensor data to the BLE service
    * 
    * @param data the sensor data
    * @param size the size of the data
*/
void send_notification_to_custom_chara(void);

/*
    * @brief Receive the sensor data for the BLE service
    * 
    * @param data the sensor data
    * @param size the size of the data
*/
void receive_sensor_data(uint8_t *data, size_t size);

/*
    * @brief Receive the battery data for the BLE service
    * 
    * @param data the battery data
*/
void set_battery_data(uint16_t *data);

/*
    * @brief Enable the stream data from the sensors
*/
void enable_stream_data(void);

/*
    * @brief Disable the stream data from the sensors
*/
void disable_stream_data(void);

/*
    * @brief Set the status characteristic
    * 
    * @param new_status the new status to be set
*/
void set_status(uint8_t new_status);

/****************************************************************************/
/**                                                                        **/
/**                          INLINE FUNCTIONS                              **/
/**                                                                        **/
/****************************************************************************/



#endif /* _VERSA_BLE_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/
