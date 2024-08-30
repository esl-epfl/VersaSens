/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : MLX90632.h                                                   **
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
* @file   MLX90632.h
* @date   DD/MM/YY
* @brief  This is the main header of MLX90632.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _MLX90632_H
#define _MLX90632_H

/****************************************************************************/
/**                                                                        **/
/**                            MODULES USED                                **/
/**                                                                        **/
/****************************************************************************/

// #include "sdk_common.h"
#include <zephyr/types.h>
#include "twim_inst.h"
#include "thread_config.h"

/****************************************************************************/
/**                                                                        **/
/**                       DEFINITIONS AND MACROS                           **/
/**                                                                        **/
/****************************************************************************/

/*!Device Address*/
#define TEMP_SENSOR_I2C_ADDR                0x3A   

/*!I2C Configuration*/
#define MAX_SIZE_TRANSFER            20  

#define RX_BUF_SIZE                  30

/*!Registers addresses*/
#define REG_I2C_ADDRESS_Addr          0x3000    //I2C Slave Address  >> 1
#define REG_CONTROL_Addr              0x3001    //Control Register
#define REG_STATUS_Addr               0x3FFF    //Status Register

/*!RAM addresses*/
#define RAM_1_Addr                    0x4000    //RAM 1 address
#define RAM_2_Addr                    0x4001    //RAM 2 address
#define RAM_3_Addr                    0x4002    //RAM 3 address
#define RAM_4_Addr                    0x4003    //RAM 4 address
#define RAM_5_Addr                    0x4004    //RAM 5 address
#define RAM_6_Addr                    0x4005    //RAM 6 address
#define RAM_7_Addr                    0x4006    //RAM 7 address
#define RAM_8_Addr                    0x4007    //RAM 8 address
#define RAM_9_Addr                    0x4008    //RAM 9 address

/*!EEPROM addresses*/
#define EEPROM_ID0_Addr               0x2405    //Chip ID
#define EEPROM_ID1_Addr               0x2406    //Chip ID
#define EEPROM_ID2_Addr               0x2407    //Chip ID
#define EEPROM_ID_CRC16_Addr          0x2408    //Chip ID CRC
#define EEPROM_EE_PRODUCT_CODE_Addr   0x2409    //Sensor information

#define EEPROM_EE_Version_Addr        0x240B    //EEPROM version
#define EEPROM_EE_P_R_LSW_Addr        0x240C    //P_R calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_P_R_MSW_Addr        0x240D    //P_R calibration constant (16-bits, Most Significant Word)
#define EEPROM_EE_P_G_LSW_Addr        0x240E    //P_G calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_P_G_MSW_Addr        0x240F    //P_G calibration constant (16-bits, Most Significant Word)
#define EEPROM_EE_P_T_LSW_Addr        0x2410    //P_T calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_P_T_MSW_Addr        0x2411    //P_T calibration constant (16-bits, Most Significant Word)
#define EEPROM_EE_P_O_LSW_Addr        0x2412    //P_O calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_P_O_MSW_Addr        0x2413    //P_O calibration constant (16-bits, Most Significant Word)
#define EEPROM_EE_Aa_LSW_Addr         0x2414    //Aa calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_Aa_MSW_Addr         0x2415    //Aa calibration constant (16-bits, Most Significant Word)
#define EEPROM_EE_Ab_LSW_Addr         0x2416    //Ab calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_Ab_MSW_Addr         0x2417    //Ab calibration constant (16-bits, Most Significant Word)
#define EEPROM_EE_Ba_LSW_Addr         0x2418    //Ba calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_Ba_MSW_Addr         0x2419    //Ba calibration constant (16-bits, Most Significant Word)
#define EEPROM_EE_Bb_LSW_Addr         0x241A    //Bb calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_Bb_MSW_Addr         0x241B    //Bb calibration constant (16-bits, Most Significant Word)
#define EEPROM_EE_Ca_LSW_Addr         0x241C    //Ca calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_Ca_MSW_Addr         0x241D    //Ca calibration constant (16-bits, Most Significant Word)
#define EEPROM_EE_Cb_LSW_Addr         0x241E    //Cb calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_Cb_MSW_Addr         0x241F    //Cb calibration constant (16-bits, Most Significant Word)
#define EEPROM_EE_Da_LSW_Addr         0x2420    //Da calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_Da_MSW_Addr         0x2421    //Da calibration constant (16-bits, Most Significant Word)
#define EEPROM_EE_Db_LSW_Addr         0x2422    //Db calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_Db_MSW_Addr         0x2423    //Db calibration constant (16-bits, Most Significant Word) 
#define EEPROM_EE_Ea_LSW_Addr         0x2424    //Ea calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_Ea_MSW_Addr         0x2425    //Ea calibration constant (16-bits, Most Significant Word)
#define EEPROM_EE_Eb_LSW_Addr         0x2426    //Eb calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_Eb_MSW_Addr         0x2427    //Eb calibration constant (16-bits, Most Significant Word)
#define EEPROM_EE_Fa_LSW_Addr         0x2428    //Fa calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_Fa_MSW_Addr         0x2429    //Fa calibration constant (16-bits, Most Significant Word)
#define EEPROM_EE_Fb_LSW_Addr         0x242A    //Fb calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_Fb_MSW_Addr         0x242B    //Fb calibration constant (16-bits, Most Significant Word)
#define EEPROM_EE_Ga_LSW_Addr         0x242C    //Ga calibration constant (16-bits, Least Significant Word)
#define EEPROM_EE_Ga_MSW_Addr         0x242D    //Ga calibration constant (16-bits, Most Significant Word) 
#define EEPROM_EE_Gb_Addr             0x242E    //Gb calibration constant
#define EEPROM_EE_Ka_Addr             0x242F    //Ka calibration constant
#define EEPROM_EE_Kb_Addr             0x2430    //Kb calibration constant

#define EEPROM_EE_Ha_Addr             0x2481    //Ha calibration constant
#define EEPROM_EE_Hb_Addr             0x2482    //Hb calibration constant

#define EEPROM_EE_CONTROL_Addr        0x24D4    //EEPROM Control register, measurement control
#define EEPROM_EE_I2C_ADDRESS_Addr    0x24D5    //I2C address >> 1

#define EEPROM_EE_MEAS_1_Addr         0x24E1    //Measurement settings 1
#define EEPROM_EE_MEAS_2_Addr         0x24E2    //Measurement settings 2




/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

/*!MLX90632 Registers Stucture*/
typedef struct {  

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {                                                                                      
        uint8_t ConfAddr            :6;      /*!< bit: 0-5   Configurable 6-bit slave address (0x1D)            */
        uint16_t _reserved_6        :10;     /*!< bit: 6-15  reserved bit                                       */
        } b;                                   /*!< Structure used for bit access                                */
        uint16_t w;                            /*!< Type used for word access                                    */
    } REG_I2C_ADDRESS; 

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {                                                                                                  
        uint8_t _reserved_0         :1;      /*!< bit: 0     reserved bit                                       */
        uint8_t Mode                :2;      /*!< bit: 1-2   0perating mode                                     */
        uint8_t Soc                 :1;      /*!< bit: 3     Start single measurement                           */
        uint8_t MeasType            :5;      /*!< bit: 4-8   Type of measurement                                */
        uint8_t _reserved_9         :2;      /*!< bit: 9-10  reserved bit                                       */
        uint8_t Sob                 :1;      /*!< bit: 11    Starts a full measurement table                    */
        uint8_t _reserved_12        :4;      /*!< bit: 12-15 reserved bit                                       */
        } b;                                   /*!< Structure used for bit access                                */
        uint16_t w;                            /*!< Type used for word access                                    */
    } REG_CONTROL; 

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {                                                                                                  
        uint8_t NewD                :1;      /*!< bit: 0     New date avaible (should be set to 0 by user)      */
        uint8_t _reserved_1         :1;      /*!< bit: 1     reserved bit                                       */
        uint8_t CycPos              :5;      /*!< bit: 2-6   Index of the last written measurement              */
        uint8_t _reserved_7         :1;      /*!< bit: 7     reserved bit                                       */
        uint8_t BOut                :1;      /*!< bit: 8     Bit is set to 0 on reset                           */
        uint8_t EepBsy              :1;      /*!< bit: 9     EEPROM is busy flag                                */
        uint8_t DevBsy              :1;      /*!< bit: 10    Measurement being executed flag                    */
        uint8_t _reserved_11        :5;      /*!< bit: 11-15 reserved bit                                       */
        } b;                                   /*!< Structure used for bit access                                */
        uint16_t w;                            /*!< Type used for word access                                    */
    } REG_STATUS; 

}__attribute__((packed)) MLX90632_REG;

/*!Temperature Compuation Parameters*/
typedef struct { 
    float Fa;
    float Fb;
    float Ga;
    float Gb;
    float Ha; 
    float Hb;
    float TO0;
    float TA0;
    float TAdut;
    float Ea;
    float Eb;
    float Ka;
    float P_O;
    float P_T;
    float P_G;
    float P_R;
    float TOdut;
    float e;
} MLX90632_MATH_PARAM;

/*!RAM Data*/
typedef struct { 
    int16_t RAM_1;
    int16_t RAM_2;
    int16_t RAM_3;
    int16_t RAM_4;
    int16_t RAM_5;
    int16_t RAM_6;
    int16_t RAM_7;
    int16_t RAM_8;
    int16_t RAM_9;
} MLX90632_RAM_DATA;

/*! Foramt of the data to be stored in the flash */
typedef struct {
    int16_t header;
    int32_t rawtime_bin;
    int16_t time_ms_bin;
    int8_t len;
    uint8_t index;
    float AmbientTemperature;
    float ObjectTemperature;
} __attribute__((packed)) MLX90632_StorageFormat;

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/

#ifndef _MLX90632_C_SRC



#endif  /* _MLX90632_C_SRC */

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

/**
 * @brief This function reads a register from the MLX90632 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, reading from a register
 *          on the MLX90632 device. The address of the register to read from is provided as a parameter,
 *          and the data read from the register is stored in a buffer also provided as a parameter.
 * 
 * @param[in]   addr      The address of the register to read from.
 * @param[out]  data      A pointer to the buffer where the read data should be stored.
 * @param[in]   reg_size  The size of the register to read from.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int mlx90632_read_reg(uint16_t addr, uint8_t *data, uint8_t reg_size);

/**
 * @brief This function writes a byte of data to a register on the MLX90632 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, writing a byte of data to a register
 *          on the MLX90632 device. The address of the register and the data to be written are provided as parameters.
 * 
 * @param[in]   addr  The address of the register to write to.
 * @param[in]   data  The data to write to the register.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int mlx90632_write_reg(uint16_t addr, uint16_t data);

/**
 * @brief This function reads a sequence of bytes from sequential registers on the MLX90632 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, reading a sequence of bytes from 
 *          sequential registers on the MLX90632 device. The start address of the registers and the number of bytes 
 *          to read are provided as parameters. The read data is stored in the buffer pointed to by the data parameter.
 * 
 * @param[in]   start_addr  The start address of the registers to read from.
 * @param[out]  data        A pointer to a buffer where the read data will be stored.
 * @param[in]   num_bytes   The number of bytes to read.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int mlx90632_read_seq_reg(uint16_t start_addr, uint8_t *data, uint8_t num_bytes);

/**
 * @brief This function writes a sequence of bytes to sequential registers on the MLX90632 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, writing a sequence of bytes to 
 *          sequential registers on the MLX90632 device. The start address of the registers and the data to be written 
 *          are provided as parameters.
 * 
 * @param[in]   start_addr  The start address of the registers to write to.
 * @param[in]   data        A pointer to the data to be written.
 * @param[in]   num_bytes   The number of bytes to write.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int mlx90632_write_seq_reg(uint16_t start_addr, const uint8_t *data, uint8_t num_bytes);

/**
 * @brief This function unlocks the EEPROM of the MLX90632 device for one-time write access.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, writing a sequence of bytes to
 *         the EEPROM of the MLX90632 device in order to unlock it for one-time write access.   
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int mlx90632_eeprom_unlock(void);

/**
 * @brief This function resets the MLX90632 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, writing a sequence of bytes to
 *         the MLX90632 device in order to reset it.   
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int mlx90632_reset(void);

/**
 * @brief This function starts continuous read mode on the MLX90632 device.
 * 
 * @details The function starts a thread that continuously reads the MLX90632 device and stores the data in a buffer.   
 * 
 * @return 0 if successful, -1 otherwise.
 */
int mlx90632_start_continuous_read(void);

/**
 * @brief This function stops continuous read mode on the MLX90632 device.
 * 
 * @details The function stops the thread that continuously reads the MLX90632 device.   
 * 
 * @return 0 if successful, -1 otherwise. 
 */
int mlx90632_stop_continuous_read(void);

/**
 * @brief This function configures the MLX90632 device.
 * 
 * @details The function performs I2C transfer, to configure the MLX90632 related structures.   
 * 
 * @return 0 if successful, -1 otherwise.
 */
int mlx90632_config(void);

/**
 * @brief This function configures the MLX90632 device.
 * 
 * @details The function initializes the MLX90632 related structures and set the I2C instance to be used.   
 * 
 * @return 0 if successful, -1 otherwise.
 */
int mlx90632_init(void);

/**
 * @brief This function computes the temperature from the raw data.
 * 
 * @details The function computes the temperature from the raw data using the MLX90632_MATH_PARAM structure.   
 * 
 * @return void
 */
void mlx90632_temp_comp(void);

/**
 * @brief This function logs a float value.   
 * 
 * @param[in]  f  The float value to log.
 * 
 * @return void
 */
void log_float(float f);

/****************************************************************************/
/**                                                                        **/
/**                          INLINE FUNCTIONS                              **/
/**                                                                        **/
/****************************************************************************/



#endif /* _MLX90632_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/
