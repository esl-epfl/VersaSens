/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : MAX86178.h                                                   **
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
* @file   MAX86178.h
* @date   DD/MM/YY
* @brief  This is the main header of MAX86178.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _MAX86178_H
#define _MAX86178_H

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

/*!Device Addresses*/
#define MAX86178_I2C_ADDR            0x6C  

/*!I2C Configuration*/
#define MAX_SIZE_TRANSFER            20  

/*maximum I2C read register size*/
#define RX_BUF_SIZE                  200

/*MAX86178 interrupt pin*/
#define MAX86178_INT_PIN             42

/*Number of measurements read from the MAX86178 fifo*/
#define MAX86178_FIFO_READ_SIZE      150

/*!Registers addresses*/

/*! Status registers*/
#define REG_STATUS_1_Addr                 0x00 // First status register
#define REG_STATUS_2_Addr                 0x01 // Second status register
#define REG_STATUS_3_Addr                 0x02 // Third status register
#define REG_STATUS_4_Addr                 0x03 // Fourth status register
#define REG_STATUS_5_Addr                 0x04 // Fifth status register

/*! FIFO registers*/
#define REG_FIFO_WRITE_PTR_Addr           0x08 // FIFO write pointer
#define REG_FIFO_READ_PTR_Addr            0x09 // FIFO read pointer
#define REG_FIFO_COUNTER_1_Addr           0x0A // FIFO counter register 1
#define REG_FIFO_COUNTER_2_Addr           0x0B // FIFO counter register 2
#define REG_FIFO_DATA_REGISTER_Addr       0x0C // FIFO data register
#define REG_FIFO_CONFIG_1_Addr            0x0D // FIFO configuration register 1
#define REG_FIFO_CONFIG_2_Addr            0x0E // FIFO configuration register 2

/*! System control registers*/
#define REG_SYSTEM_SYNC_Addr              0x10 // System synchronization register
#define REG_SYSTEM_CONFIG_1_Addr          0x11 // System configuration register 1
#define REG_SYSTEM_CONFIG_2_Addr          0x12 // System configuration register 2
#define REG_PIN_FUNCTIONAL_CONFIG_Addr    0x13 // Pin functional configuration register
#define REG_OUTPUT_PIN_CONFIG_Addr        0x14 // Output pin configuration register
#define REG_I2C_BROADCAST_ADDR_Addr       0x15 // I2C broadcast address register

/*! PLL registers*/
#define REG_PLL_CONFIG_1_Addr             0x18 // PLL configuration register 1
#define REG_PLL_CONFIG_2_Addr             0x19 // PLL configuration register 2
#define REG_PLL_CONFIG_3_Addr             0x1A // PLL configuration register 3
#define REG_PLL_CONFIG_4_Addr             0x1B // PLL configuration register 4
#define REG_PLL_CONFIG_5_Addr             0x1C // PLL configuration register 5
#define REG_PLL_CONFIG_6_Addr             0x1D // PLL configuration register 6

/*! PPG setup registers*/
#define REG_PPG_CONFIG_1_Addr            0x20 // PPG configuration register 1
#define REG_PPG_CONFIG_2_Addr            0x21 // PPG configuration register 2
#define REG_PPG_CONFIG_3_Addr            0x22 // PPG configuration register 3
#define REG_PPG_CONFIG_4_Addr            0x23 // PPG configuration register 4
#define REG_PHOTODIODE_BIAS_Addr         0x24 // Photodiodes bias register

/*! PPG frame rate clock registers*/
#define REG_FR_CLOCK_DIVIDER_MSB_Addr    0x28 // Frame rate clock divider MSB
#define REG_FR_CLOCK_DIVIDER_LSB_Addr    0x29 // Frame rate clock divider LSB

/*! PPG MEAS1 setup registers*/
#define REG_MEAS1_SELECTS_Addr           0x30 // MEAS1 selects register
#define REG_MEAS1_CONFIG_1_Addr          0x31 // MEAS1 configuration register 1
#define REG_MEAS1_CONFIG_2_Addr          0x32 // MEAS1 configuration register 2
#define REG_MEAS1_CONFIG_3_Addr          0x33 // MEAS1 configuration register 3
#define REG_MEAS1_CONFIG_4_Addr          0x34 // MEAS1 configuration register 4
#define REG_MEAS1_CONFIG_5_Addr          0x35 // MEAS1 configuration register 5
#define REG_MEAS1_LEDA_CURRENT_Addr      0x36 // MEAS1 LEDA current register
#define REG_MEAS1_LEDB_CURRENT_Addr      0x37 // MEAS1 LEDB current register

/*! PPG MEAS2 setup registers*/
#define REG_MEAS2_SELECTS_Addr           0x38 // MEAS2 selects register
#define REG_MEAS2_CONFIG_1_Addr          0x39 // MEAS2 configuration register 1
#define REG_MEAS2_CONFIG_2_Addr          0x3A // MEAS2 configuration register 2
#define REG_MEAS2_CONFIG_3_Addr          0x3B // MEAS2 configuration register 3
#define REG_MEAS2_CONFIG_4_Addr          0x3C // MEAS2 configuration register 4
#define REG_MEAS2_CONFIG_5_Addr          0x3D // MEAS2 configuration register 5
#define REG_MEAS2_LEDA_CURRENT_Addr      0x3E // MEAS2 LEDA current register
#define REG_MEAS2_LEDB_CURRENT_Addr      0x3F // MEAS2 LEDB current register

/*! PPG MEAS3 setup registers*/
#define REG_MEAS3_SELECTS_Addr           0x40 // MEAS3 selects register
#define REG_MEAS3_CONFIG_1_Addr          0x41 // MEAS3 configuration register 1
#define REG_MEAS3_CONFIG_2_Addr          0x42 // MEAS3 configuration register 2
#define REG_MEAS3_CONFIG_3_Addr          0x43 // MEAS3 configuration register 3
#define REG_MEAS3_CONFIG_4_Addr          0x44 // MEAS3 configuration register 4
#define REG_MEAS3_CONFIG_5_Addr          0x45 // MEAS3 configuration register 5
#define REG_MEAS3_LEDA_CURRENT_Addr      0x46 // MEAS3 LEDA current register
#define REG_MEAS3_LEDB_CURRENT_Addr      0x47 // MEAS3 LEDB current register

/*! PPG MEAS4 setup registers*/
#define REG_MEAS4_SELECTS_Addr           0x48 // MEAS4 selects register
#define REG_MEAS4_CONFIG_1_Addr          0x49 // MEAS4 configuration register 1
#define REG_MEAS4_CONFIG_2_Addr          0x4A // MEAS4 configuration register 2
#define REG_MEAS4_CONFIG_3_Addr          0x4B // MEAS4 configuration register 3
#define REG_MEAS4_CONFIG_4_Addr          0x4C // MEAS4 configuration register 4
#define REG_MEAS4_CONFIG_5_Addr          0x4D // MEAS4 configuration register 5
#define REG_MEAS4_LEDA_CURRENT_Addr      0x4E // MEAS4 LEDA current register
#define REG_MEAS4_LEDB_CURRENT_Addr      0x4F // MEAS4 LEDB current register

/*! PPG MEAS5 setup registers*/
#define REG_MEAS5_SELECTS_Addr           0x50 // MEAS5 selects register
#define REG_MEAS5_CONFIG_1_Addr          0x51 // MEAS5 configuration register 1
#define REG_MEAS5_CONFIG_2_Addr          0x52 // MEAS5 configuration register 2
#define REG_MEAS5_CONFIG_3_Addr          0x53 // MEAS5 configuration register 3
#define REG_MEAS5_CONFIG_4_Addr          0x54 // MEAS5 configuration register 4
#define REG_MEAS5_CONFIG_5_Addr          0x55 // MEAS5 configuration register 5
#define REG_MEAS5_LEDA_CURRENT_Addr      0x56 // MEAS5 LEDA current register
#define REG_MEAS5_LEDB_CURRENT_Addr      0x57 // MEAS5 LEDB current register

/*! PPG MEAS6 setup registers*/
#define REG_MEAS6_SELECTS_Addr           0x58 // MEAS6 selects register
#define REG_MEAS6_CONFIG_1_Addr          0x59 // MEAS6 configuration register 1
#define REG_MEAS6_CONFIG_2_Addr          0x5A // MEAS6 configuration register 2
#define REG_MEAS6_CONFIG_3_Addr          0x5B // MEAS6 configuration register 3
#define REG_MEAS6_CONFIG_4_Addr          0x5C // MEAS6 configuration register 4
#define REG_MEAS6_CONFIG_5_Addr          0x5D // MEAS6 configuration register 5
#define REG_MEAS6_LEDA_CURRENT_Addr      0x5E // MEAS6 LEDA current register
#define REG_MEAS6_LEDB_CURRENT_Addr      0x5F // MEAS6 LEDB current register

/*! PPG threshold interrupt registers*/
#define REG_THRESHOLD_MEAS_SEL_Addr     0x70 // Thresholds measurement select register
#define REG_THRESHOLD_HYST_Addr         0x71 // Thresholds hysteresis register
#define REG_PPG_HI_THRESHOLD1_Addr      0x72 // High thresholds register 1
#define REG_PPG_LO_THRESHOLD1_Addr      0x73 // Low thresholds register 1
#define REG_PPG_HI_THRESHOLD2_Addr      0x74 // High thresholds register 2
#define REG_PPG_LO_THRESHOLD2_Addr      0x75 // Low thresholds register 2

/*! ECG setup registers*/
#define REG_ECG_CONFIG_1_Addr            0x80 // ECG configuration register 1
#define REG_ECG_CONFIG_2_Addr            0x81 // ECG configuration register 2
#define REG_ECG_CONFIG_3_Addr            0x82 // ECG configuration register 3
#define REG_ECG_CONFIG_4_Addr            0x83 // ECG configuration register 4

/*! ECG calibration registers*/
#define REG_ECG_CAL_CONFIG_1_Addr        0x84 // ECG calibration configuration register 1
#define REG_ECG_CAL_CONFIG_2_Addr        0x85 // ECG calibration configuration register 2
#define REG_ECG_CAL_CONFIG_3_Addr        0x86 // ECG calibration configuration register 3

/*! ECG lead detect registers*/
#define REG_ECG_LEAD_DETECT_CONFIG_1_Addr       0x88 // ECG lead configuration register 1
#define REG_ECG_LEAD_DETECT_CONFIG_2_Addr       0x89 // ECG lead configuration register 2

/*! ECG lead bias registers*/
#define REG_ECG_LEAD_BIAS_CONFIG_1_Addr         0x90 // ECG lead bias configuration register 1

/*! ECG RLD and CM Amps registers*/
#define REG_RLD_CONFIG_1_Addr            0x92 // RLD configuration register 1
#define REG_RLD_CONFIG_2_Addr            0x93 // RLD configuration register 2

/*! BIOZ setup registers*/
#define REG_BIOZ_CONFIG_1_Addr           0xA0 // BIOZ configuration register 1
#define REG_BIOZ_CONFIG_2_Addr           0xA1 // BIOZ configuration register 2
#define REG_BIOZ_CONFIG_3_Addr           0xA2 // BIOZ configuration register 3
#define REG_BIOZ_CONFIG_4_Addr           0xA3 // BIOZ configuration register 4
#define REG_BIOZ_CONFIG_5_Addr           0xA4 // BIOZ configuration register 5
#define REG_BIOZ_CONFIG_6_Addr           0xA5 // BIOZ configuration register 6
#define REG_BIOZ_CONFIG_7_Addr           0xA6 // BIOZ configuration register 7
#define REG_BIOZ_CONFIG_8_Addr           0xA7 // BIOZ configuration register 8
#define REG_BIOZ_LOW_TRESHOLD_Addr       0xA8 // BIOZ low threshold register
#define REG_BIOZ_HIGH_TRESHOLD_Addr      0xA9 // BIOZ high threshold register

/*! BIOZ calibration registers*/
#define REG_BIOZ_MUX_CONFIG_1_Addr       0xAA // BIOZ calibration configuration register 1
#define REG_BIOZ_MUX_CONFIG_2_Addr       0xAB // BIOZ calibration configuration register 2
#define REG_BIOZ_MUX_CONFIG_3_Addr       0xAC // BIOZ calibration configuration register 3
#define REG_BIOZ_MUX_CONFIG_4_Addr       0xAD // BIOZ calibration configuration register 4

/*! BIOZ lead detect registers*/
#define REG_BIOZ_LEAD_DETECT_CONFIG_1_Addr      0xB0 // BIOZ lead configuration register 1
#define REG_BIOZ_LEAD_OFF_THRESHOLD_Addr        0xB1 // BIOZ lead off threshold register

/*! BIOZ lead bias registers*/
#define REG_BIOZ_LEAD_BIAS_CONFIG_1_Addr        0xB4 // BIOZ lead bias configuration register 1

/*! Respiration setup registers*/
#define REG_RESPIRATION_CONFIG_1_Addr    0xB6 // Respiration configuration register 1

/*! Interrupt enable registers*/
#define REG_INTERRUPT1_ENABLE_1_Addr          0xC0 // Interrupt 1 enable register 1
#define REG_INTERRUPT1_ENABLE_2_Addr          0xC1 // Interrupt 1 enable register 2
#define REG_INTERRUPT1_ENABLE_3_Addr          0xC2 // Interrupt 1 enable register 3
#define REG_INTERRUPT1_ENABLE_4_Addr          0xC3 // Interrupt 1 enable register 4
#define REG_INTERRUPT1_ENABLE_5_Addr          0xC4 // Interrupt 1 enable register 5
#define REG_INTERRUPT2_ENABLE_1_Addr          0xC5 // Interrupt 2 enable register 1
#define REG_INTERRUPT2_ENABLE_2_Addr          0xC6 // Interrupt 2 enable register 2
#define REG_INTERRUPT2_ENABLE_3_Addr          0xC7 // Interrupt 2 enable register 3
#define REG_INTERRUPT2_ENABLE_4_Addr          0xC8 // Interrupt 2 enable register 4
#define REG_INTERRUPT2_ENABLE_5_Addr          0xC9 // Interrupt 2 enable register 5

/*! Part ID registers*/
#define REG_PART_ID_Addr                 0xFF // Part ID register





/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

/*!MAX86178 Registers Stucture*/
typedef struct {  

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t PWR_RDY             :1;      /*!< bit: 0  Power ready flag                                  */
        uint8_t PPG_THRESH1_HILOW   :1;      /*!< bit: 1  PPG threshold 1 high/low                          */
        uint8_t PPG_THRESH2_HILOW   :1;      /*!< bit: 2  PPG threshold 2 high/low                          */
        uint8_t EXP_OVF             :1;      /*!< bit: 3  Exposure measurement overflow flag                */
        uint8_t ALC_OVF             :1;      /*!< bit: 4  ALC overflow flag                                 */    
        uint8_t FIFO_DATA_RDY       :1;      /*!< bit: 5  Set to 1 when FIFO data is ready                  */                                                                                   
        uint8_t PPG_FRAME_RDY       :1;      /*!< bit: 6  Set to 1 when a PPG frame is ready                */
        uint8_t A_FULL              :1;      /*!< bit: 7  Set to 1 when FIFO is full                        */
        } b;                                 /*!< Structure used for bit access                             */
        uint8_t w;                           /*!< Type used for word access                                 */
    } REG_STATUS_1; 

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t LED1_COMPB         :1;      /*!< bit: 0  Set to 1 when LED1 voltage is below compliance     */
        uint8_t LED2_COMPB         :1;      /*!< bit: 1  Set to 1 when LED2 voltage is below compliance     */
        uint8_t LED3_COMPB         :1;      /*!< bit: 2  Set to 1 when LED3 voltage is below compliance     */
        uint8_t LED4_COMPB         :1;      /*!< bit: 3  Set to 1 when LED4 voltage is below compliance     */
        uint8_t LED5_COMP          :1;      /*!< bit: 4  Set to 1 when LED5 voltage is below compliance     */
        uint8_t LED6_COMP          :1;      /*!< bit: 5  Set to 1 when LED6 voltage is below compliance     */
        uint8_t LED7_COMP          :1;      /*!< bit: 6  Set to 1 when LED7 voltage is below compliance     */
        uint8_t INVALID_CFG_PPG    :1;      /*!< bit: 7  Set to 1 when an invalid PPG frame rate            */
        } b;                                /*!< Structure used for bit access                              */
        uint8_t w;                          /*!< Type used for word access                                  */
    } REG_STATUS_2; 

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t _reserved_0        :1;      /*!< bit: 0  Reserved                                           */
        uint8_t PHASE_LOCK         :1;      /*!< bit: 1  Set to 1 when the PLL achieves phase lock          */
        uint8_t PHASE_UNLOCK       :1;      /*!< bit: 2  Set to 1 when the PLL loses phase lock             */
        uint8_t FREQ_LOCK          :1;      /*!< bit: 3  Set to 1 when the PLL achieves frequency lock      */
        uint8_t FREQ_UNLOCK        :1;      /*!< bit: 4  Set to 1 when the PLL loses frequency lock         */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                           */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                           */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                           */
        } b;                                /*!< Structure used for bit access                              */
        uint8_t w;                          /*!< Type used for word access                                  */
    } REG_STATUS_3;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t ECG_LOFF_NL        :1;      /*!< bit: 0  Set to 1 when ECGN voltage is below lead-off threshold */
        uint8_t ECG_LOFF_Nh        :1;      /*!< bit: 1  Set to 1 when ECGN voltage is above lead-off threshold */        
        uint8_t ECG_LOFF_PL        :1;      /*!< bit: 2  Set to 1 when ECGP voltage is below lead-off threshold */
        uint8_t ECG_LOFF_PH        :1;      /*!< bit: 3  Set to 1 when ECGP voltage is above lead-off threshold */
        uint8_t RLD_OOR            :1;      /*!< bit: 4  Set to 1 when RLD voltage is out of range              */
        uint8_t ECG_FAST_REC       :1;      /*!< bit: 5  Set to 1 when ECG fast recovery is enabled             */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                               */
        uint8_t ECG_LON            :1;      /*!< bit: 7  Set to 1 when ECG lead-on is detected                  */
        } b;                                /*!< Structure used for bit access                                  */
        uint8_t w;                          /*!< Type used for word access                                      */
    } REG_STATUS_4;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t BIOZ_LOFF_NL       :1;      /*!< bit: 0  Set to 1 when BIN voltage is below lead-off threshold */
        uint8_t BIOZ_LOFF_NH       :1;      /*!< bit: 1  Set to 1 when BIN voltage is above lead-off threshold */
        uint8_t BIOZ_LOFF_PL       :1;      /*!< bit: 2  Set to 1 when BIP voltage is below lead-off threshold */
        uint8_t BIOZ_LOFF_PH       :1;      /*!< bit: 3  Set to 1 when BIP voltage is above lead-off threshold */
        uint8_t BIOZ_DRV_OOR       :1;      /*!< bit: 4  Set to 1 when BIOZ driver voltage is out of range     */
        uint8_t BIOZ_UNDR          :1;      /*!< bit: 5  Set to 1 when BIOZ ADC value is under the threshold   */
        uint8_t BIOZ_OVER          :1;      /*!< bit: 6  Set to 1 when BIOZ ADC value is over the threshold    */ 
        uint8_t BIOZ_LON           :1;      /*!< bit: 7  Set to 1 when BIOZ lead-on is detected                */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_STATUS_5;

    uint8_t _unused_05;                    /*!< Unused register 0x05                                           */
    uint8_t _unused_06;                    /*!< Unused register 0x06                                           */
    uint8_t _unused_07;                    /*!< Unused register 0x07                                           */

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t FIFO_WR_PTR        :8;      /*!< bit: 0-7  FIFO write pointer                                  */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_FIFO_WRITE_PTR;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t FIFO_RD_PTR        :8;      /*!< bit: 0-7  FIFO read pointer                                */
        } b;                                /*!< Structure used for bit access                              */
        uint8_t w;                          /*!< Type used for word access                                  */
    } REG_FIFO_READ_PTR;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t OVF_COUNTER        :7;      /*!< bit: 0-6  FIFO overflow counter                            */
        uint8_t FIFO_DATA_COUNT    :1;      /*!< bit: 7  FIFO data count MSB                              */
        } b;                                /*!< Structure used for bit access                              */
        uint8_t w;                          /*!< Type used for word access                                  */
    } REG_FIFO_COUNTER_1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t FIFO_DATA_COUNT    :8;      /*!< bit: 0-7  FIFO data count LSB                              */
        } b;                                /*!< Structure used for bit access                              */
        uint8_t w;                          /*!< Type used for word access                                  */
    } REG_FIFO_COUNTER_2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t FIFO_DATA          :8;      /*!< bit: 0-7  FIFO data register                              */
        } b;                                /*!< Structure used for bit access                              */
        uint8_t w;                          /*!< Type used for word access                                  */
    } REG_FIFO_DATA_REGISTER;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t FIFO_A_FULL        :8;      /*!< bit: 0-7  Set the high water mark for the FIFO             */
        } b;                                /*!< Structure used for bit access                              */
        uint8_t w;                          /*!< Type used for word access                                  */
    } REG_FIFO_CONFIG_1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t _reserved_0        :1;      /*!< bit: 0  Reserved                                               */
        uint8_t FIFO_RO            :1;      /*!< bit: 1  FIFO roll-over mode                                    */
        uint8_t A_FULL_TYPE        :1;      /*!< bit: 2  Define behavior of A_FULL flag and Interrupt           */
        uint8_t FIFO_STAT_CLR      :1;      /*!< bit: 3  Determine if A_FULL is cleared by reading FIFO_DATA    */
        uint8_t FLUSH_FIFO         :1;      /*!< bit: 4  Flush the FIFO                                         */
        uint8_t FIFO_MARK          :1;      /*!< bit: 5  when set to 1, a marker tag is pushed into the FIFO    */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                               */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                               */
        } b;                                /*!< Structure used for bit access                                  */
        uint8_t w;                          /*!< Type used for word access                                      */
    } REG_FIFO_CONFIG_2;

    uint8_t _unused_0F;                    /*!< Unused register 0x0F                                           */

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t _reserved_0        :1;      /*!< bit: 0  Reserved                                               */
        uint8_t _reserved_1        :1;      /*!< bit: 1  Reserved                                               */
        uint8_t _reserved_2        :1;      /*!< bit: 2  Reserved                                               */
        uint8_t _reserved_3        :1;      /*!< bit: 3  Reserved                                               */
        uint8_t _reserved_4        :1;      /*!< bit: 4  Reserved                                               */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                               */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                               */
        uint8_t TIMING_SYS_RESET   :1;      /*!< bit: 7  Timing system reset                                    */
        } b;                                /*!< Structure used for bit access                                  */
        uint8_t w;                          /*!< Type used for word access                                      */
    } REG_SYSTEM_SYNC;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t RESET                   :1;      /*!< bit: 0  Reset the device                                       */
        uint8_t SHDN                    :1;      /*!< bit: 1  Shutdown the device                                    */
        uint8_t _reserved_2             :1;      /*!< bit: 2  Reserved                                               */
        uint8_t ECG_BIOZ_TIMING_DATA    :1;      /*!< bit: 3  Enable saving ECG to BIOZ timing data in FIFO          */
        uint8_t BIOZ_PPG_TIMING_DATA    :1;      /*!< bit: 4  Enable saving BIOZ to PPG timing data in FIFO          */
        uint8_t ECG_PPG_TIMING_DATA     :1;      /*!< bit: 5  Enable saving ECG to PPG timing data in FIFO           */
        uint8_t DISABLE_I2C             :1;      /*!< bit: 6  When set to 1, Disable I2C and use SPI only            */
        uint8_t _reserved_7             :1;      /*!< bit: 7  Reserved                                               */
        } b;                                /*!< Structure used for bit access                                       */
        uint8_t w;                          /*!< Type used for word access                                           */
    } REG_SYSTEM_CONFIG_1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        { 
        uint8_t ECG_SAMP_SYNC_FREQ      :4;      /*!< bit: 0-3  ECG sample synchronization frequency                 */
        uint8_t _reserved_4             :1;      /*!< bit: 4  Reserved                                               */
        uint8_t _reserved_5             :1;      /*!< bit: 5  Reserved                                               */
        uint8_t _reserved_6             :1;      /*!< bit: 6  Reserved                                               */
        uint8_t BYP_DLY                 :1;      /*!< bit: 7  Bypass delay                                           */
        } b;                                /*!< Structure used for bit access                                       */
        uint8_t w;                          /*!< Type used for word access                                           */
    } REG_SYSTEM_CONFIG_2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t INT1_FCFG          :2;      /*!< bit: 0-1  INT1 functional configuration                        */
        uint8_t INT2_FCFG          :2;      /*!< bit: 2-3  INT2 functional configuration                        */
        uint8_t TRI_ICFG           :1;      /*!< bit: 4  Set the input active edge of the TRIG pin              */
        uint8_t TRIG_FCFG          :3;      /*!< bit: 5-7  TRIG functional configuration                        */
        } b;                                /*!< Structure used for bit access                                  */
        uint8_t w;                          /*!< Type used for word access                                      */
    } REG_PIN_FUNCTIONAL_CONFIG;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t INT1_OCFG          :2;      /*!< bit: 0-1  INT1 output drive configuration                        */
        uint8_t INT2_OCFG          :2;      /*!< bit: 2-3  INT2 output drive configuration                        */
        uint8_t _reserved_4        :1;      /*!< bit: 4  Reserved                                                 */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                                 */
        uint8_t TRIG_OCFG          :2;      /*!< bit: 6-7  TRIG output drive configuration                        */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_OUTPUT_PIN_CONFIG;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t I2C_BCAST_EN        :1;      /*!< bit: 0  Enable I2C broadcast mode                               */
        uint8_t I2C_BCAST_ADDR      :7;      /*!< bit: 1-7  I2C broadcast address                                 */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_I2C_BROADCAST_ADDR;

    uint8_t _unused_16;                    /*!< Unused register 0x16                                           */
    uint8_t _unused_17;                    /*!< Unused register 0x17                                           */

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t PLL_EN             :1;      /*!< bit: 0  Enable the PLL                                          */
        uint8_t PLL_LOCK_WNDW      :1;      /*!< bit: 1  Select time window for PLL lock detection               */
        uint8_t _reserved_2        :1;      /*!< bit: 2  Reserved                                                */
        uint8_t _reserved_3        :1;      /*!< bit: 3  Reserved                                                */
        uint8_t _reserved_4        :1;      /*!< bit: 4  Reserved                                                */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                                */
        uint8_t MDIV               :2;      /*!< bit: 6-7  PLL M-divider MSB                                     */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_PLL_CONFIG_1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MDIV               :8;      /*!< bit: 0-7  PLL M-divider LSB                                     */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_PLL_CONFIG_2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t BIOZ_KDIV          :4;      /*!< bit: 0-3  BIOZ K-divider                                        */
        uint8_t _reserved_4        :1;      /*!< bit: 4  Reserved                                                */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                                */
        uint8_t BIOZ_NDIV          :2;      /*!< bit: 6-7  BIOZ N-divider                                        */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_PLL_CONFIG_3;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t ECG_FDIV           :3;      /*!< bit: 0-2  ECG F-divider                                         */
        uint8_t _reserved_3        :1;      /*!< bit: 3  Reserved                                                */
        uint8_t _reserved_4        :1;      /*!< bit: 4  Reserved                                                */
        uint8_t ECG_NDIV           :3;      /*!< bit: 5-7  ECG N-divider MSB                                     */ 
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_PLL_CONFIG_4;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t ECG_NDIV           :8;      /*!< bit: 0-7  ECG N-divider LSB                                     */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_PLL_CONFIG_5;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t CLK_FINE_TUNE      :5;      /*!< bit: 0-4  Clock fine tune                                       */
        uint8_t CLK_FREQ_SEL       :1;      /*!< bit: 5  Clock frequency select                                  */
        uint8_t REF_CLK_SEL        :1;      /*!< bit: 6  Reference clock select (1 for external oscillator)      */
        uint8_t _reserved_7        :1;      /*!< bit: 7  ECG M-divider                                           */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_PLL_CONFIG_6;

    uint8_t _unused_1E;                    /*!< Unused register 0x1E                                           */
    uint8_t _unused_1F;                    /*!< Unused register 0x1F                                           */

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS1_EN           :1;      /*!< bit: 0  Enable MEAS1                                            */
        uint8_t MEAS2_EN           :1;      /*!< bit: 1  Enable MEAS2                                            */
        uint8_t MEAS3_EN           :1;      /*!< bit: 2  Enable MEAS3                                            */
        uint8_t MEAS4_EN           :1;      /*!< bit: 3  Enable MEAS4                                            */
        uint8_t MEAS5_EN           :1;      /*!< bit: 4  Enable MEAS5                                            */
        uint8_t MEAS6_EN           :1;      /*!< bit: 5  Enable MEAS6                                            */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                                */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_PPG_CONFIG_1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t _reserved_0        :1;      /*!< bit: 0  Reserved                                                */
        uint8_t _reserved_1        :1;      /*!< bit: 1  Reserved                                                */
        uint8_t PPG1_PWRDN        :1;      /*!< bit: 2  PPG1 power down                                         */
        uint8_t PPG2_PWRDN        :1;      /*!< bit: 3  PPG2 power down                                         */
        uint8_t _reserved_4        :1;      /*!< bit: 4  Reserved                                                */
        uint8_t PPG_SYNC_MODE      :1;      /*!< bit: 5  PPG synchronization mode (0:inernal, 1:external)        */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                                */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_PPG_CONFIG_2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS1_CONFIG_SEL   :1;      /*!< bit: 0  MEAS1 configuration select                              */
        uint8_t COLLECT_RAW_DATA   :1;      /*!< bit: 1  Collect raw data (0:computed data, 1:raw data)          */
        uint8_t _reserved_2        :1;      /*!< bit: 2  Reserved                                                */
        uint8_t ALC_DISABLE        :1;      /*!< bit: 3  Disable ambient light cancelation                       */
        uint8_t SMP_AVE            :3;      /*!< bit: 4-6  Sample averaging configuration                        */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_PPG_CONFIG_3;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t _reserved_0        :1;      /*!< bit: 0  Reserved                                                */
        uint8_t _reserved_1        :1;      /*!< bit: 1  Reserved                                                */
        uint8_t _reserved_2        :1;      /*!< bit: 2  Reserved                                                */
        uint8_t PROX_AUTO          :1;      /*!< bit: 3  Enable Proximity auto detect mode                       */
        uint8_t PROX_DATA_EN       :1;      /*!< bit: 4  Enable MEAS6 data saving in FIFO                        */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                                */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                                */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_PPG_CONFIG_4;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t PD1_BIAS           :2;      /*!< bit: 0-1  Photodiode 1 bias current                             */
        uint8_t PD2_BIAS           :2;      /*!< bit: 2-3  Photodiode 2 bias current                             */
        uint8_t PD3_BIAS           :2;      /*!< bit: 4-5  Photodiode 3 bias current                             */
        uint8_t PD4_BIAS           :2;      /*!< bit: 6-7  Photodiode 4 bias current                             */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_PHOTODIODE_BIAS;

    uint8_t _unused_25;                    /*!< Unused register 0x25                                           */
    uint8_t _unused_26;                    /*!< Unused register 0x26                                           */
    uint8_t _unused_27;                    /*!< Unused register 0x27                                           */

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t FR_CLK_DIV          :8;     /*!< bit: 0-7  Frame rate clock divider                              */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_FR_CLOCK_DIVIDER_MSB;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t FR_CLK_DIV          :8;     /*!< bit: 0-7  Frame rate clock divider                              */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_FR_CLOCK_DIVIDER_LSB;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS1_DRVA         :3;      /*!< bit: 0-2  select the LEDn_DRV pin driven by LED driver A        */
        uint8_t MEAS1_DRVB         :3;      /*!< bit: 3-5  select the LEDn_DRV pin driven by LED driver B        */
        uint8_t MEAS1_AMB          :1;      /*!< bit: 6  enable ambient light measurement                        */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS1_SELECTS;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS1_AVER         :3;      /*!< bit: 0-2  select the number of exposition to be average for a sample   */
        uint8_t MEAS1_TINT         :2;      /*!< bit: 3-4  select the integration time of PPG ADCs                      */
        uint8_t MEAS1_FILT_SEL     :1;      /*!< bit: 5  select the ambient light rejection method                      */
        uint8_t MEAS1_FILT2_SEL    :1;      /*!< bit: 6  select the ambient light rejection method (higher order)       */
        uint8_t MEAS1_SINC3_SEL    :1;      /*!< bit: 7  select the SINC3 filter for PPG ADCs                           */
        } b;                                /*!< Structure used for bit access                                          */
        uint8_t w;                          /*!< Type used for word access                                              */
    } REG_MEAS1_CONFIG_1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS1_PPG1_ADC_RGE :2;      /*!< bit: 0-1  select the ADC range for PPG1                         */
        uint8_t _reserved_2        :1;      /*!< bit: 2  Reserved                                                */
        uint8_t _reserved_3        :1;      /*!< bit: 3  Reserved                                                */
        uint8_t MEAS1_PPG2_ADC_RGE :2;      /*!< bit: 4-5  select the ADC range for PPG2                         */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                                */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS1_CONFIG_2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS1_PPG1_DACOFF  :4;      /*!< bit: 0-3  PPG1 DAC offset                                      */
        uint8_t MEAS1_PPG2_DACOFF  :4;      /*!< bit: 4-7  PPG2 DAC offset                                      */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS1_CONFIG_3;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS1_LED_RGE      :2;      /*!< bit: 0-1  select the LED drive currentrange                     */
        uint8_t MEAS1_LED_SETLNG   :2;      /*!< bit: 2-3  select the LED settling time                          */
        uint8_t _reserved_4        :1;      /*!< bit: 4  Reserved                                                */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                                */
        uint8_t MEAS1_PD_SETLNG    :2;      /*!< bit: 6-7  select the photodiode settling time                   */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS1_CONFIG_4;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS1_PD1_SEL     :2;       /*!< bit: 0-1  select the photodiode 1 input                          */
        uint8_t MEAS1_PD2_SEL     :2;       /*!< bit: 2-3  select the photodiode 2 input                          */
        uint8_t MEAS1_PD3_SEL     :2;       /*!< bit: 4-5  select the photodiode 3 input                          */
        uint8_t MEAS1_PD4_SEL     :2;       /*!< bit: 6-7  select the photodiode 4 input                          */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS1_CONFIG_5;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS1_DRVA_PA     :8;      /*!< bit: 0-7  select the LED drive current on LED driver A            */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS1_LEDA_CURRENT;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS1_DRVB_PA     :8;      /*!< bit: 0-7  select the LED drive current on LED driver B            */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS1_LEDB_CURRENT;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS2_DRVA         :3;      /*!< bit: 0-2  select the LEDn_DRV pin driven by LED driver A        */
        uint8_t MEAS2_DRVB         :3;      /*!< bit: 3-5  select the LEDn_DRV pin driven by LED driver B        */
        uint8_t MEAS2_AMB          :1;      /*!< bit: 6  enable ambient light measurement                        */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS2_SELECTS;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS2_AVER         :3;      /*!< bit: 0-2  select the number of exposition to be average for a sample   */
        uint8_t MEAS2_TINT         :2;      /*!< bit: 3-4  select the integration time of PPG ADCs                      */
        uint8_t MEAS2_FILT_SEL     :1;      /*!< bit: 5  select the ambient light rejection method                      */
        uint8_t MEAS2_FILT2_SEL    :1;      /*!< bit: 6  select the ambient light rejection method (higher order)       */
        uint8_t MEAS2_SINC3_SEL    :1;      /*!< bit: 7  select the SINC3 filter for PPG ADCs                           */
        } b;                                /*!< Structure used for bit access                                          */
        uint8_t w;                          /*!< Type used for word access                                              */
    } REG_MEAS2_CONFIG_1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS2_PPG1_ADC_RGE :2;      /*!< bit: 0-1  select the ADC range for PPG1                         */
        uint8_t _reserved_2        :1;      /*!< bit: 2  Reserved                                                */
        uint8_t _reserved_3        :1;      /*!< bit: 3  Reserved                                                */
        uint8_t MEAS2_PPG2_ADC_RGE :2;      /*!< bit: 4-5  select the ADC range for PPG2                         */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                                */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS2_CONFIG_2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS2_PPG1_DACOFF  :4;      /*!< bit: 0-3  PPG1 DAC offset                                      */
        uint8_t MEAS2_PPG2_DACOFF  :4;      /*!< bit: 4-7  PPG2 DAC offset                                      */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS2_CONFIG_3;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS2_LED_RGE      :2;      /*!< bit: 0-1  select the LED drive currentrange                     */
        uint8_t MEAS2_LED_SETLNG   :2;      /*!< bit: 2-3  select the LED settling time                          */
        uint8_t _reserved_4        :1;      /*!< bit: 4  Reserved                                                */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                                */
        uint8_t MEAS2_PD_SETLNG    :2;      /*!< bit: 6-7  select the photodiode settling time                   */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS2_CONFIG_4;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS2_PD1_SEL     :2;       /*!< bit: 0-1  select the photodiode 1 input                          */
        uint8_t MEAS2_PD2_SEL     :2;       /*!< bit: 2-3  select the photodiode 2 input                          */
        uint8_t MEAS2_PD3_SEL     :2;       /*!< bit: 4-5  select the photodiode 3 input                          */
        uint8_t MEAS2_PD4_SEL     :2;       /*!< bit: 6-7  select the photodiode 4 input                          */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS2_CONFIG_5;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS2_DRVA_PA     :8;      /*!< bit: 0-7  select the LED drive current on LED driver A            */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS2_LEDA_CURRENT;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS2_DRVB_PA     :8;      /*!< bit: 0-7  select the LED drive current on LED driver B            */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS2_LEDB_CURRENT;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS3_DRVA         :3;      /*!< bit: 0-2  select the LEDn_DRV pin driven by LED driver A        */
        uint8_t MEAS3_DRVB         :3;      /*!< bit: 3-5  select the LEDn_DRV pin driven by LED driver B        */
        uint8_t MEAS3_AMB          :1;      /*!< bit: 6  enable ambient light measurement                        */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS3_SELECTS;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS3_AVER         :3;      /*!< bit: 0-2  select the number of exposition to be average for a sample   */
        uint8_t MEAS3_TINT         :2;      /*!< bit: 3-4  select the integration time of PPG ADCs                      */
        uint8_t MEAS3_FILT_SEL     :1;      /*!< bit: 5  select the ambient light rejection method                      */
        uint8_t MEAS3_FILT2_SEL    :1;      /*!< bit: 6  select the ambient light rejection method (higher order)       */
        uint8_t MEAS3_SINC3_SEL    :1;      /*!< bit: 7  select the SINC3 filter for PPG ADCs                           */
        } b;                                /*!< Structure used for bit access                                          */
        uint8_t w;                          /*!< Type used for word access                                              */
    } REG_MEAS3_CONFIG_1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS3_PPG1_ADC_RGE :2;      /*!< bit: 0-1  select the ADC range for PPG1                         */
        uint8_t _reserved_2        :1;      /*!< bit: 2  Reserved                                                */
        uint8_t _reserved_3        :1;      /*!< bit: 3  Reserved                                                */
        uint8_t MEAS3_PPG2_ADC_RGE :2;      /*!< bit: 4-5  select the ADC range for PPG2                         */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                                */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS3_CONFIG_2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS3_PPG1_DACOFF  :4;      /*!< bit: 0-3  PPG1 DAC offset                                      */
        uint8_t MEAS3_PPG2_DACOFF  :4;      /*!< bit: 4-7  PPG2 DAC offset                                      */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS3_CONFIG_3;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS3_LED_RGE      :2;      /*!< bit: 0-1  select the LED drive currentrange                     */
        uint8_t MEAS3_LED_SETLNG   :2;      /*!< bit: 2-3  select the LED settling time                          */
        uint8_t _reserved_4        :1;      /*!< bit: 4  Reserved                                                */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                                */
        uint8_t MEAS3_PD_SETLNG    :2;      /*!< bit: 6-7  select the photodiode settling time                   */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS3_CONFIG_4;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS3_PD1_SEL     :2;       /*!< bit: 0-1  select the photodiode 1 input                          */
        uint8_t MEAS3_PD2_SEL     :2;       /*!< bit: 2-3  select the photodiode 2 input                          */
        uint8_t MEAS3_PD3_SEL     :2;       /*!< bit: 4-5  select the photodiode 3 input                          */
        uint8_t MEAS3_PD4_SEL     :2;       /*!< bit: 6-7  select the photodiode 4 input                          */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS3_CONFIG_5;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS3_DRVA_PA     :8;      /*!< bit: 0-7  select the LED drive current on LED driver A            */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS3_LEDA_CURRENT;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS3_DRVB_PA     :8;      /*!< bit: 0-7  select the LED drive current on LED driver B            */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS3_LEDB_CURRENT;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS4_DRVA         :3;      /*!< bit: 0-2  select the LEDn_DRV pin driven by LED driver A        */
        uint8_t MEAS4_DRVB         :3;      /*!< bit: 3-5  select the LEDn_DRV pin driven by LED driver B        */
        uint8_t MEAS4_AMB          :1;      /*!< bit: 6  enable ambient light measurement                        */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS4_SELECTS;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS4_AVER         :3;      /*!< bit: 0-2  select the number of exposition to be average for a sample   */
        uint8_t MEAS4_TINT         :2;      /*!< bit: 3-4  select the integration time of PPG ADCs                      */
        uint8_t MEAS4_FILT_SEL     :1;      /*!< bit: 5  select the ambient light rejection method                      */
        uint8_t MEAS4_FILT2_SEL    :1;      /*!< bit: 6  select the ambient light rejection method (higher order)       */
        uint8_t MEAS4_SINC3_SEL    :1;      /*!< bit: 7  select the SINC3 filter for PPG ADCs                           */
        } b;                                /*!< Structure used for bit access                                          */
        uint8_t w;                          /*!< Type used for word access                                              */
    } REG_MEAS4_CONFIG_1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS4_PPG1_ADC_RGE :2;      /*!< bit: 0-1  select the ADC range for PPG1                         */
        uint8_t _reserved_2        :1;      /*!< bit: 2  Reserved                                                */
        uint8_t _reserved_3        :1;      /*!< bit: 3  Reserved                                                */
        uint8_t MEAS4_PPG2_ADC_RGE :2;      /*!< bit: 4-5  select the ADC range for PPG2                         */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                                */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS4_CONFIG_2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS4_PPG1_DACOFF  :4;      /*!< bit: 0-3  PPG1 DAC offset                                      */
        uint8_t MEAS4_PPG2_DACOFF  :4;      /*!< bit: 4-7  PPG2 DAC offset                                      */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS4_CONFIG_3;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS4_LED_RGE      :2;      /*!< bit: 0-1  select the LED drive currentrange                     */
        uint8_t MEAS4_LED_SETLNG   :2;      /*!< bit: 2-3  select the LED settling time                          */
        uint8_t _reserved_4        :1;      /*!< bit: 4  Reserved                                                */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                                */
        uint8_t MEAS4_PD_SETLNG    :2;      /*!< bit: 6-7  select the photodiode settling time                   */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS4_CONFIG_4;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS4_PD1_SEL     :2;       /*!< bit: 0-1  select the photodiode 1 input                          */
        uint8_t MEAS4_PD2_SEL     :2;       /*!< bit: 2-3  select the photodiode 2 input                          */
        uint8_t MEAS4_PD3_SEL     :2;       /*!< bit: 4-5  select the photodiode 3 input                          */
        uint8_t MEAS4_PD4_SEL     :2;       /*!< bit: 6-7  select the photodiode 4 input                          */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS4_CONFIG_5;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS4_DRVA_PA     :8;      /*!< bit: 0-7  select the LED drive current on LED driver A            */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS4_LEDA_CURRENT;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS4_DRVB_PA     :8;      /*!< bit: 0-7  select the LED drive current on LED driver B            */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS4_LEDB_CURRENT;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS5_DRVA         :3;      /*!< bit: 0-2  select the LEDn_DRV pin driven by LED driver A        */
        uint8_t MEAS5_DRVB         :3;      /*!< bit: 3-5  select the LEDn_DRV pin driven by LED driver B        */
        uint8_t MEAS5_AMB          :1;      /*!< bit: 6  enable ambient light measurement                        */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS5_SELECTS;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS5_AVER         :3;      /*!< bit: 0-2  select the number of exposition to be average for a sample   */
        uint8_t MEAS5_TINT         :2;      /*!< bit: 3-4  select the integration time of PPG ADCs                      */
        uint8_t MEAS5_FILT_SEL     :1;      /*!< bit: 5  select the ambient light rejection method                      */
        uint8_t MEAS5_FILT2_SEL    :1;      /*!< bit: 6  select the ambient light rejection method (higher order)       */
        uint8_t MEAS5_SINC3_SEL    :1;      /*!< bit: 7  select the SINC3 filter for PPG ADCs                           */
        } b;                                /*!< Structure used for bit access                                          */
        uint8_t w;                          /*!< Type used for word access                                              */
    } REG_MEAS5_CONFIG_1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS5_PPG1_ADC_RGE :2;      /*!< bit: 0-1  select the ADC range for PPG1                         */
        uint8_t _reserved_2        :1;      /*!< bit: 2  Reserved                                                */
        uint8_t _reserved_3        :1;      /*!< bit: 3  Reserved                                                */
        uint8_t MEAS5_PPG2_ADC_RGE :2;      /*!< bit: 4-5  select the ADC range for PPG2                         */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                                */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS5_CONFIG_2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS5_PPG1_DACOFF  :4;      /*!< bit: 0-3  PPG1 DAC offset                                      */
        uint8_t MEAS5_PPG2_DACOFF  :4;      /*!< bit: 4-7  PPG2 DAC offset                                      */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS5_CONFIG_3;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS5_LED_RGE      :2;      /*!< bit: 0-1  select the LED drive currentrange                     */
        uint8_t MEAS5_LED_SETLNG   :2;      /*!< bit: 2-3  select the LED settling time                          */
        uint8_t _reserved_4        :1;      /*!< bit: 4  Reserved                                                */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                                */
        uint8_t MEAS5_PD_SETLNG    :2;      /*!< bit: 6-7  select the photodiode settling time                   */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS5_CONFIG_4;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS5_PD1_SEL     :2;       /*!< bit: 0-1  select the photodiode 1 input                          */
        uint8_t MEAS5_PD2_SEL     :2;       /*!< bit: 2-3  select the photodiode 2 input                          */
        uint8_t MEAS5_PD3_SEL     :2;       /*!< bit: 4-5  select the photodiode 3 input                          */
        uint8_t MEAS5_PD4_SEL     :2;       /*!< bit: 6-7  select the photodiode 4 input                          */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS5_CONFIG_5;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS5_DRVA_PA     :8;      /*!< bit: 0-7  select the LED drive current on LED driver A            */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS5_LEDA_CURRENT;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS5_DRVB_PA     :8;      /*!< bit: 0-7  select the LED drive current on LED driver B            */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS5_LEDB_CURRENT;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS6_DRVA         :3;      /*!< bit: 0-2  select the LEDn_DRV pin driven by LED driver A        */
        uint8_t MEAS6_DRVB         :3;      /*!< bit: 3-5  select the LEDn_DRV pin driven by LED driver B        */
        uint8_t MEAS6_AMB          :1;      /*!< bit: 6  enable ambient light measurement                        */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS6_SELECTS;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS6_AVER         :3;      /*!< bit: 0-2  select the number of exposition to be average for a sample   */
        uint8_t MEAS6_TINT         :2;      /*!< bit: 3-4  select the integration time of PPG ADCs                      */
        uint8_t MEAS6_FILT_SEL     :1;      /*!< bit: 5  select the ambient light rejection method                      */
        uint8_t MEAS6_FILT2_SEL    :1;      /*!< bit: 6  select the ambient light rejection method (higher order)       */
        uint8_t MEAS6_SINC3_SEL    :1;      /*!< bit: 7  select the SINC3 filter for PPG ADCs                           */
        } b;                                /*!< Structure used for bit access                                          */
        uint8_t w;                          /*!< Type used for word access                                              */
    } REG_MEAS6_CONFIG_1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS6_PPG1_ADC_RGE :2;      /*!< bit: 0-1  select the ADC range for PPG1                         */
        uint8_t _reserved_2        :1;      /*!< bit: 2  Reserved                                                */
        uint8_t _reserved_3        :1;      /*!< bit: 3  Reserved                                                */
        uint8_t MEAS6_PPG2_ADC_RGE :2;      /*!< bit: 4-5  select the ADC range for PPG2                         */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                                */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS6_CONFIG_2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS6_PPG1_DACOFF  :4;      /*!< bit: 0-3  PPG1 DAC offset                                      */
        uint8_t MEAS6_PPG2_DACOFF  :4;      /*!< bit: 4-7  PPG2 DAC offset                                      */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS6_CONFIG_3;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS6_LED_RGE      :2;      /*!< bit: 0-1  select the LED drive currentrange                     */
        uint8_t MEAS6_LED_SETLNG   :2;      /*!< bit: 2-3  select the LED settling time                          */
        uint8_t _reserved_4        :1;      /*!< bit: 4  Reserved                                                */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                                */
        uint8_t MEAS6_PD_SETLNG    :2;      /*!< bit: 6-7  select the photodiode settling time                   */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_MEAS6_CONFIG_4;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS6_PD1_SEL     :2;       /*!< bit: 0-1  select the photodiode 1 input                          */
        uint8_t MEAS6_PD2_SEL     :2;       /*!< bit: 2-3  select the photodiode 2 input                          */
        uint8_t MEAS6_PD3_SEL     :2;       /*!< bit: 4-5  select the photodiode 3 input                          */
        uint8_t MEAS6_PD4_SEL     :2;       /*!< bit: 6-7  select the photodiode 4 input                          */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS6_CONFIG_5;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS6_DRVA_PA     :8;      /*!< bit: 0-7  select the LED drive current on LED driver A            */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS6_LEDA_CURRENT;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MEAS6_DRVB_PA     :8;      /*!< bit: 0-7  select the LED drive current on LED driver B            */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_MEAS6_LEDB_CURRENT;

    uint8_t _unused_60;                   /*!< Unused register 0x60                                           */
    uint8_t _unused_61;                   /*!< Unused register 0x61                                           */
    uint8_t _unused_62;                   /*!< Unused register 0x62                                           */
    uint8_t _unused_63;                   /*!< Unused register 0x63                                           */
    uint8_t _unused_64;                   /*!< Unused register 0x64                                           */
    uint8_t _unused_65;                   /*!< Unused register 0x65                                           */
    uint8_t _unused_66;                   /*!< Unused register 0x66                                           */
    uint8_t _unused_67;                   /*!< Unused register 0x67                                           */
    uint8_t _unused_68;                   /*!< Unused register 0x68                                           */
    uint8_t _unused_69;                   /*!< Unused register 0x69                                           */
    uint8_t _unused_6A;                   /*!< Unused register 0x6A                                           */
    uint8_t _unused_6B;                   /*!< Unused register 0x6B                                           */
    uint8_t _unused_6C;                   /*!< Unused register 0x6C                                           */
    uint8_t _unused_6D;                   /*!< Unused register 0x6D                                           */
    uint8_t _unused_6E;                   /*!< Unused register 0x6E                                           */
    uint8_t _unused_6F;                   /*!< Unused register 0x6F                                           */

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t THRESH1_MEAS_SEL   :3;      /*!< bit: 0-2  select the measurement source for threshold 1         */
        uint8_t _reserved_3        :1;      /*!< bit: 3  Reserved                                                */
        uint8_t THRESH2_MEAS_SEL   :3;      /*!< bit: 4-6  select the measurement source for threshold 2         */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_THRESHOLD_MEAS_SEL;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t LEVEL_HYST         :3;      /*!< bit: 0-2  select the hysteresis magnitude for the threshold     */
        uint8_t TIME_HYST          :2;      /*!< bit: 3-4  select the number of sample outside limit before int  */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                                */
        uint8_t THRESH1_PPG_SEL    :1;      /*!< bit: 6  select the optical channel for thresh 1                 */
        uint8_t THRESH2_PPG_SEL    :1;      /*!< bit: 7  select the optical channel for thresh 2                 */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_THERSHOLD_HYST;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t THRESHOLD1_UPPER   :8;      /*!< bit: 0-7  set the upper threshold 1                             */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_PPG_HI_THRESHOLD1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t THRESHOLD1_LOWER   :8;      /*!< bit: 0-7  set the lower threshold 1                              */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_PPG_LO_THRESHOLD1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t THRESHOLD2_UPPER   :8;      /*!< bit: 0-7  set the upper threshold 2                             */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_PPG_HI_THRESHOLD2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t THRESHOLD2_LOWER   :8;      /*!< bit: 0-7  set the lower threshold 2                             */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_PPG_LO_THRESHOLD2;

    uint8_t _unused_76;                   /*!< Unused register 0x76                                           */
    uint8_t _unused_77;                   /*!< Unused register 0x77                                           */
    uint8_t _unused_78;                   /*!< Unused register 0x78                                           */
    uint8_t _unused_79;                   /*!< Unused register 0x79                                           */
    uint8_t _unused_7A;                   /*!< Unused register 0x7A                                           */
    uint8_t _unused_7B;                   /*!< Unused register 0x7B                                           */
    uint8_t _unused_7C;                   /*!< Unused register 0x7C                                           */
    uint8_t _unused_7D;                   /*!< Unused register 0x7D                                           */
    uint8_t _unused_7E;                   /*!< Unused register 0x7E                                           */
    uint8_t _unused_7F;                   /*!< Unused register 0x7F                                           */

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t ECG_EN             :1;      /*!< bit: 0  enable ECG channel and data conversion                  */
        uint8_t ECG_DEC_RATE       :3;      /*!< bit: 1-3  select the decimation ratio for ECG_ADS               */
        uint8_t _reserved_4        :1;      /*!< bit: 4  Reserved                                                */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                                */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                                */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_ECG_CONFIG_1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t ECG_INA_GAIN       :2;      /*!< bit: 0-1  select the gain for the ECG input amplifier           */
        uint8_t ECG_INA_RGE        :2;      /*!< bit: 2-3  select the gain range for the input amplifier         */
        uint8_t ECG_PGA_GAIN       :3;      /*!< bit: 4-6  select the gain for the PGA in ECG channel            */
        uint8_t ECG_IPOL           :1;      /*!< bit: 7  select the polarity of the ECG signal                   */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_ECG_CONFIG_2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t ECG_MUX_SEL        :2;      /*!< bit: 0-1  select the ECG input multiplexer                      */
        uint8_t ECG_AUTO_REC       :1;      /*!< bit: 2  enable the analog automatic fast recovery mode          */
        uint8_t ECH_IMP_HIGH       :1;      /*!< bit: 3  select the combined output impedance of CAPP and CAPN   */
        uint8_t _reserved_4        :1;      /*!< bit: 4  Reserved                                                */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                                */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                                */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_ECG_CONFIG_3;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t ECG_FAST_REC_THRESHOLD :6;  /*!< bit: 0-5  set the threshold for the fast recovery algorithm     */
        uint8_t EN_ECG_FAST_REC    :2;      /*!< bit: 6-7  enable the fast recovery mode or manual               */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_ECG_CONFIG_4;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t ECG_CAL_EN         :1;      /*!< bit: 0  enable the ECG calibration source VCALP and VCALN       */
        uint8_t ECG_CAL_DUTY       :1;      /*!< bit: 1 select between time high and 50% duty mode               */
        uint8_t ECG_CAL_FREQ       :3;      /*!< bit: 2-4  select the frequency of the calibration source        */
        uint8_t ECG_CAL_HIGH       :3;      /*!< bit: 5-7  MSB of the time high for the calibration source       */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_ECG_CAL_CONFIG_1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t ECG_CAL_HIGH        :8;     /*!< bit: 0-7  LSB of the time high for the calibration source       */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_ECG_CAL_CONFIG_2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t ECG_CAL_N_SEL      :2;      /*!< bit: 0-1  select the calibration source for the ECGN input      */
        uint8_t ECG_CAL_P_SEL      :2;      /*!< bit: 2-3  select the calibration source for the ECGP input      */
        uint8_t ECG_CAL_MAG        :1;      /*!< bit: 4  select the magnitude of the calibration source          */
        uint8_t ECG_CAL_MODE       :1;      /*!< bit: 5  select the calibration mode                             */
        uint8_t ECG_OPEN_N         :1;      /*!< bit: 6  open the ECGN input for calibration                     */
        uint8_t ECG_OPEN_P         :1;      /*!< bit: 7  open the ECGP input for calibration                     */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_ECG_CAL_CONFIG_3;

    uint8_t _unused_87;                   /*!< Unused register 0x86                                           */

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t ECG_LOFF_FREQ      :3;      /*!< bit: 0-2  select the frequency of the lead-off detection        */
        uint8_t ECG_LOFF_MODE      :1;      /*!< bit: 3  select the lead-off detection mode                      */
        uint8_t _reserved_4        :1;      /*!< bit: 4  Reserved                                                */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                                */
        uint8_t EN_ECG_LOFF        :1;      /*!< bit: 6  enable the lead-off detection                           */
        uint8_t EN_ECG_LON         :1;      /*!< bit: 7  enable the lead-on detection                            */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_ECG_LEAD_DETECT_CONFIG_1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t ECG_LOFF_THRESH    :4;      /*!< bit: 0-3  set the threshold for the lead-off detection          */
        uint8_t ECG_LOFF_IMAG      :3;      /*!< bit: 4-6  select the DC/AC lead-off current amplitude           */
        uint8_t ECG_LOFF_IPOL      :1;      /*!< bit: 7  select the polarity of the lead-off current             */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_ECG_LEAD_DETECT_CONFIG_2;

    uint8_t _unused_8a;                   /*!< Unused register 0x8a                                           */
    uint8_t _unused_8b;                   /*!< Unused register 0x8b                                           */
    uint8_t _unused_8c;                   /*!< Unused register 0x8c                                           */
    uint8_t _unused_8d;                   /*!< Unused register 0x8d                                           */
    uint8_t _unused_8e;                   /*!< Unused register 0x8e                                           */
    uint8_t _unused_8f;                   /*!< Unused register 0x8f                                           */

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t EN_ECG_RBIASN       :1;      /*!< bit: 0  enable the ECG lead bias for the ECGN input            */
        uint8_t EN_ECG_RBIASP       :1;      /*!< bit: 1  enable the ECG lead bias for the ECGP input            */
        uint8_t ECG_RBIAS_VALUE     :2;      /*!< bit: 2-3  select the value of the bias resistance              */
        uint8_t _reserved_4         :1;      /*!< bit: 4  Reserved                                               */
        uint8_t _reserved_5         :1;      /*!< bit: 5  Reserved                                               */
        uint8_t _reserved_6         :1;      /*!< bit: 6  Reserved                                               */
        uint8_t _reserved_7         :1;      /*!< bit: 7  Reserved                                               */
        } b;                                 /*!< Structure used for bit access                                  */
        uint8_t w;                           /*!< Type used for word access                                      */
    } REG_ECG_LEAD_BIAS_CONFIG_1;

    uint8_t _unused_91;                   /*!< Unused register 0x91                                           */

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t RLD_GAIN            :2;      /*!< bit: 0-1  select the internal RLD gain                         */
        uint8_t ACTV_CM_N           :1;      /*!< bit: 2  Enable the negative inpute to the common mode averager */
        uint8_t ACTV_CM_P           :1;      /*!< bit: 3  Enable the positive inpute to the common mode averager */
        uint8_t EN_RLD_OOR          :1;      /*!< bit: 4  Enable the OOR comparator                              */
        uint8_t RLD_RBIAS           :1;      /*!< bit: 5  select the lead bias voltage                           */
        uint8_t RLD_MODE            :1;      /*!< bit: 6  select the RLD mode (0:open-loop, 1:close-loop)        */
        uint8_t RLD_EN              :1;      /*!< bit: 7  Enable the right leg drive circuit                     */
        } b;                                 /*!< Structure used for bit access                                  */
        uint8_t w;                           /*!< Type used for word access                                      */
    } REG_RLD_CONFIG_1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t BODY_BIAS_DAC       :4;      /*!< bit: 0-3  set voltage at noninvert terminal of the RLD amp     */
        uint8_t RLD_BW              :2;      /*!< bit: 4-5  select the bandwidth of the RLD circuit              */
        uint8_t RLD_SEL_ECG         :1;      /*!< bit: 6  select the input for the RLD circuit                   */
        uint8_t RLD_EXT_RES         :1;      /*!< bit: 7  connect/disconnect the internal resistor               */
        } b;                                 /*!< Structure used for bit access                                  */
        uint8_t w;                           /*!< Type used for word access                                      */
    } REG_RLD_CONFIG_2;

    uint8_t _unused_94;                   /*!< Unused register 0x94                                           */
    uint8_t _unused_95;                   /*!< Unused register 0x95                                           */
    uint8_t _unused_96;                   /*!< Unused register 0x96                                           */
    uint8_t _unused_97;                   /*!< Unused register 0x97                                           */
    uint8_t _unused_98;                   /*!< Unused register 0x98                                           */
    uint8_t _unused_99;                   /*!< Unused register 0x99                                           */
    uint8_t _unused_9A;                   /*!< Unused register 0x9A                                           */
    uint8_t _unused_9B;                   /*!< Unused register 0x9B                                           */
    uint8_t _unused_9C;                   /*!< Unused register 0x9C                                           */
    uint8_t _unused_9D;                   /*!< Unused register 0x9D                                           */
    uint8_t _unused_9E;                   /*!< Unused register 0x9E                                           */
    uint8_t _unused_9F;                   /*!< Unused register 0x9F                                           */

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t BIOZ_EN             :2;      /*!< bit: 0-1  enable BIOZ channels                                */
        uint8_t ECG_BIOZ_BG_EN      :1;      /*!< bit: 2  enable the ECG_BIOZ bandgap                           */
        uint8_t BIOZ_ADC_OSR        :3;      /*!< bit: 3-5  select the oversampling ratio for the BIOZ ADCs     */
        uint8_t BIOZ_DAC_OSR        :2;      /*!< bit: 6-7  select the oversampling ratio for the BIOZ DACs     */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_BIOZ_CONFIG_1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t EN_BIOZ_THRESH      :1;      /*!< bit: 0  enable the BIOZ threshold detection                    */
        uint8_t _reserved_1         :1;      /*!< bit: 1  Reserved                                               */
        uint8_t _reserved_2         :1;      /*!< bit: 2  Reserved                                               */
        uint8_t BIOZ_DLPF           :2;      /*!< bit: 3-4  select the digital low-pass filter for the BIOZ      */
        uint8_t BIOZ_DHPF           :2;      /*!< bit: 5-6  select the digital high-pass filter for the BIOZ     */
        } b;                                /*!< Structure used for bit access                                   */
        uint8_t w;                          /*!< Type used for word access                                       */
    } REG_BIOZ_CONFIG_2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t BIOZ_DRV_MODE       :2;      /*!< bit: 0-1  select the stimulus type of the BIOZ transmit channel */
        uint8_t BIOZ_IDRV_RGE       :2;      /*!< bit: 2-3  select the internal current range resistor            */
        uint8_t BIOZ_VDRV_MAG       :2;      /*!< bit: 4-5  select the magnitude of the voltage driver            */
        uint8_t _reserved_6         :1;      /*!< bit: 6  Reserved                                                */
        uint8_t BIOZ_EXT_RES        :1;      /*!< bit: 7  specify if the internal or external resistor is used    */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_BIOZ_CONFIG_3;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t _reserved_0         :1;      /*!< bit: 0  Reserved                                                */
        uint8_t _reserved_1         :1;      /*!< bit: 1  Reserved                                                */
        uint8_t _reserved_2         :1;      /*!< bit: 2  Reserved                                                */
        uint8_t _reserved_3         :1;      /*!< bit: 3  Reserved                                                */
        uint8_t _reserved_4         :1;      /*!< bit: 4  Reserved                                                */
        uint8_t _reserved_5         :1;      /*!< bit: 5  Reserved                                                */
        uint8_t _reserved_6         :1;      /*!< bit: 6  Reserved                                                */
        uint8_t EN_UTIL_MODE        :1;      /*!< bit: 7  enable the utility mode                                */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_BIOZ_CONFIG_4;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t BIOZ_DC_DAC_CODE    :7;      /*!< bit: 0-6  upper 7 bits of the DAC code for the BIOZ DAC         */
        uint8_t BIOZ_DC_CODE_SEL    :1;      /*!< bit: 7  select the DDS DAC code                                 */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_BIOZ_CONFIG_5;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t BIOZ_GAIN          :2;      /*!< bit: 0-1  select the gain for the BIOZ receive channel           */
        uint8_t BIOZ_DM_DIS        :1;      /*!< bit: 2  disable the demodulator for the BIOZ receive channel     */
        uint8_t BIOZ_INA_MODE      :1;      /*!< bit: 3  select the mode of the instrumentation amplifier (INA)   */
        uint8_t BIOZ_AHPF          :4;      /*!< bit: 4-7  select the analog high-pass filter for the BIOZ        */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_BIOZ_CONFIG_6;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t BIOZ_AMP_BW        :2;      /*!< bit: 0-1  select the bandwidth of the BIOZ amplifier             */
        uint8_t BIOZ_AMP_RGE       :2;      /*!< bit: 2-3  select the gain range of the BIOZ amplifier            */
        uint8_t BIOZ_DAC_RESET     :1;      /*!< bit: 4  reset the BIOZ DAC                                       */
        uint8_t BIOZ_DRV_RESET     :1;      /*!< bit: 5  reset the BIOZ transmit channel                          */
        uint8_t BIOZ_DC_RESTORE    :1;      /*!< bit: 6  10MOhm applied to the current drive amplifier            */
        uint8_t BIOZ_EXT_CAP       :1;      /*!< bit: 7  enable the use of external capacitor                     */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_BIOZ_CONFIG_7;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t BIOZ_CH_FSEL       :1;      /*!< bit: 0  select the chopping frequency of the BIOZ channel        */
        uint8_t BIOZ_INA_CHOP_EN   :1;      /*!< bit: 1  enable the chopping of the INA                           */
        uint8_t BIOZ_FAST_CHOP_EN  :1;      /*!< bit: 2  enable the FAST_START function                           */
        uint8_t BIOZ_IPOL          :1;      /*!< bit: 3  select the polarity of the BIOZ signal                   */
        uint8_t BIOZ_STBYON        :1;      /*!< bit: 4  select BioZ receive channel behavior in standby mode     */
        uint8_t BIOZ_CMRES_DIS     :1;      /*!< bit: 5  set the common mode impedance of BioZ receive channel    */
        uint8_t RLD_DRV            :1;      /*!< bit: 6  select the source of BioZ common-mode voltage            */
        uint8_t RLD_SEL_BIOZ       :1;      /*!< bit: 7  select BIP and BIN voltage as input for the RLD          */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_BIOZ_CONFIG_8;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t BIOZ_LO_THRESH    :8;      /*!< bit: 0-7  set the BioZ underrange threshold                       */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_BIOZ_LOW_TRESHOLD;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t BIOZ_HI_THRESH    :8;      /*!< bit: 0-7  set the BioZ overrange threshold                        */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_BIOZ_HI_TRESHOLD;

    union
    {
        struct
        {
        uint8_t BIOZ_CAL_EN     :1;      /*!< bit: 0  connect the calibration pinsto measure ext calibr res  */
        uint8_t BIOZ_MUX_EN     :1;      /*!< bit: 1  enable the BioZ MUX connection                         */
        uint8_t CONNECT_CAL_ONLY    :1;  /*!< bit: 2  connect only the calibration pins to the BioZ MUX      */
        uint8_t _reserved_3     :1;      /*!< bit: 3  Reserved                                               */
        uint8_t _reserved_4     :1;      /*!< bit: 4  Reserved                                               */
        uint8_t BMUX_BIST_EN    :1;      /*!< bit: 5  enable the built in self test resistor                 */
        uint8_t BMUX_RSEL       :2;      /*!< bit: 6-7  select the calibration resistance of the BioZ MUX    */
        } b;                                /*!< Structure used for bit access                               */
        uint8_t w;                          /*!< Type used for word access                                   */
    } REG_BIOZ_MUX_CONFIG_1;

    union
    {
        struct
        {
        uint8_t EN_INTERRUPTINLOAD  :1;      /*!< bit: 0  enable the input capacitive loading compensation         */
        uint8_t EN_EXT_INLOAD  :1;      /*!< bit: 1  enable the external guard-drive circuit                  */
        uint8_t _reserved_2    :1;      /*!< bit: 2  Reserved                                                 */
        uint8_t _reserved_3    :1;      /*!< bit: 3  Reserved                                                 */
        uint8_t _reserved_4    :1;      /*!< bit: 4  Reserved                                                 */
        uint8_t GSR_LOAD_EN    :1;      /*!< bit: 5  enable the built-in GSR resistor                         */
        uint8_t BMUX_GSR_RSEL  :2;      /*!< bit: 6-7  select the resistance of the GSR resistor              */
        } b;                                /*!< Structure used for bit access                                */
        uint8_t w;                          /*!< Type used for word access                                    */
    } REG_BIOZ_MUX_CONFIG_2;

    union
    {
        struct
        {
        uint8_t DRVN_ASSIGN    :2;      /*!< bit: 0-1  select the electrode pin used for the BioZ neg drive   */
        uint8_t DRVP_ASSIGN    :2;      /*!< bit: 2-3  select the electrode pin used for the BioZ pos drive   */
        uint8_t BIN_ASSIGN     :2;      /*!< bit: 4-5  select the electrode pin used for the BioZ neg input   */
        uint8_t BIP_ASSIGN     :2;      /*!< bit: 6-7  select the electrode pin used for the BioZ pos input   */
        } b;                                /*!< Structure used for bit access                                */
        uint8_t w;                          /*!< Type used for word access                                    */
    } REG_BIOZ_MUX_CONFIG_3;

    union
    {
        struct
        {
        uint8_t BIST_R_ERR    :8;       /*!< bit: 0-7  error of the value of the built-in self test resistor      */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_BIOZ_MUX_CONFIG_4;

    uint8_t _unused_AE;                   /*!< Unused register 0xAE                                           */
    uint8_t _unused_AF;                   /*!< Unused register 0xAF                                           */

    union
    {
        struct
        {
        uint8_t BIOZ_LOFF_IMAG  :3;     /*!< bit: 0-2  select the DC/AC lead-off current amplitude           */
        uint8_t BIOZ_LOFF_IPOL  :1;     /*!< bit: 3  select the polarity of the lead-off current             */
        uint8_t EN_BIOZ_DRV_OOR :1;     /*!< bit: 4  enable the OOR detection for the drive voltage          */
        uint8_t EN_EXT_BIOZ_LOFF :1;    /*!< bit: 5  enable the external lead-off detection                  */
        uint8_t EN_BIOZ_LOFF   :1;     /*!< bit: 6  enable the lead-off detection                            */
        uint8_t EN_BIOZ_LON    :1;     /*!< bit: 7  enable the lead-on detection                             */
        } b;                                /*!< Structure used for bit access                               */
        uint8_t w;                          /*!< Type used for word access                                   */
    } REG_BIOZ_LEAD_DETECT_CONFIG_1;

    union
    {
        struct
        {
        uint8_t BIOZ_LOFF_THRESH  :4;     /*!< bit: 0-3  set the threshold for the lead-off detection           */
        uint8_t RESP_CG_MAG       :4;     /*!< bit: 4-7  select the magnitude of the respiratory drive current  */
        } b;                                /*!< Structure used for bit access                                  */
        uint8_t w;                          /*!< Type used for word access                                      */
    } REG_BIOZ_LEAD_OFF_THRESHOLD;

    uint8_t _unused_B2;                   /*!< Unused register 0xB2                                           */
    uint8_t _unused_B3;                   /*!< Unused register 0xB3                                           */

    union
    {
        struct
        {
        uint8_t EN_BIOZ_RBIASN   :1;      /*!< bit: 0  enable the BioZ lead bias for the BIN input             */
        uint8_t EN_BIOZ_RBIASP   :1;      /*!< bit: 1  enable the BioZ lead bias for the BIP input             */
        uint8_t BIOZ_RBIAS_VALUE :2;      /*!< bit: 2-3  select the value of the bias resistance               */
        uint8_t _reserved_4      :1;      /*!< bit: 4  Reserved                                                */
        uint8_t _reserved_5      :1;      /*!< bit: 5  Reserved                                                */
        uint8_t _reserved_6      :1;      /*!< bit: 6  Reserved                                                */
        uint8_t _reserved_7      :1;      /*!< bit: 7  Reserved                                                */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_BIOZ_LEAD_BIAS_CONFIG_1;

    uint8_t _unused_B5;                   /*!< Unused register 0xB5                                           */

    union
    {
        struct
        {
        uint8_t RESP_EN          :1;      /*!< bit: 0  enable the respiration mode                                */
        uint8_t CG_MODE          :2;      /*!< bit: 1-2  select a current source dynamic matching an CM feedback  */
        uint8_t CG_CHOP_CLK      :2;      /*!< bit: 3-4  select the respiration current source chopping clk       */
        uint8_t CG_LPF_DUTY      :3;      /*!< bit: 5-7  select the duty cycle of the respiration current generator common mode feedback low-pass filter */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_RESPIRATION_CONFIG_1;

    uint8_t _unused_B7;                   /*!< Unused register 0xB7                                           */
    uint8_t _unused_B8;                   /*!< Unused register 0xB8                                           */
    uint8_t _unused_B9;                   /*!< Unused register 0xB9                                           */
    uint8_t _unused_BA;                   /*!< Unused register 0xBA                                           */
    uint8_t _unused_BB;                   /*!< Unused register 0xBB                                           */
    uint8_t _unused_BC;                   /*!< Unused register 0xBC                                           */
    uint8_t _unused_BD;                   /*!< Unused register 0xBD                                           */
    uint8_t _unused_BE;                   /*!< Unused register 0xBE                                           */
    uint8_t _unused_BF;                   /*!< Unused register 0xBF                                           */

    union
    {
        struct
        {
        uint8_t _reserved_0        :1;      /*!< bit: 0  Reserved                                                 */
        uint8_t PPG_THRESH1_HILO_EN1 :1;    /*!< bit: 1  enable the PPG_THRESH1_HILO status to trigger INT1 pin   */
        uint8_t PPG_THRESH2_HILO_EN1 :1;    /*!< bit: 1  enable the PPG_THRESH2_HILO status to trigger INT1 pin   */
        uint8_t EXP_OVF_EN1        :1;      /*!< bit: 1  enable the EXP_OVF status to trigger INT1 pin            */
        uint8_t ACL_OVF_EN1        :1;      /*!< bit: 1  enable the ACL_OVF status to trigger INT1 pin            */
        uint8_t FIFO_DATA_RDY_EN1  :1;      /*!< bit: 1  enable the FIFO_DATA_RDY status to trigger INT1 pin      */
        uint8_t PPG_FRAME_RDY_EN1  :1;      /*!< bit: 1  enable the PPG_FRAME_RDY status to trigger INT1 pin      */
        uint8_t A_FULL_EN1         :1;      /*!< bit: 1  enable the A_FULL status to trigger INT1 pin             */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_INTERRUPT1_ENABLE_1;

    union
    {
        struct
        {
        uint8_t LED1_COMPB_EN1    :1;      /*!< bit: 0  enable the LED1_COMPB status to trigger INT1 pin         */
        uint8_t LED2_COMPB_EN1    :1;      /*!< bit: 1  enable the LED2_COMPB status to trigger INT1 pin         */
        uint8_t LED3_COMPB_EN1    :1;      /*!< bit: 2  enable the LED3_COMPB status to trigger INT1 pin         */
        uint8_t LED4_COMPB_EN1    :1;      /*!< bit: 3  enable the LED4_COMPB status to trigger INT1 pin         */
        uint8_t LED5_COMPB_EN1    :1;      /*!< bit: 4  enable the LED5_COMPB status to trigger INT1 pin         */
        uint8_t LED6_COMPB_EN1    :1;      /*!< bit: 5  enable the LED6_COMPB status to trigger INT1 pin         */
        uint8_t _reserved_6       :1;      /*!< bit: 6  Reserved                                                 */
        uint8_t INVALID_PPG_CFG_EN1 :1;    /*!< bit: 7  enable the INVALID_PPG_CFG status to trigger INT1 pin    */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_INTERRUPT1_ENABLE_2;

    union
    {
        struct
        {
        uint8_t _reserved_0        :1;      /*!< bit: 0  Reserved                                                 */
        uint8_t PHASE_LOCK_EN1     :1;      /*!< bit: 1  enable the PHASE_LOCK status to trigger INT1 pin         */
        uint8_t PHASE_UNLOCK_EN1   :1;      /*!< bit: 2  enable the PHASE_UNLOCK status to trigger INT1 pin       */
        uint8_t FREQ_LOCK_EN1      :1;      /*!< bit: 3  enable the FREQ_LOCK status to trigger INT1 pin          */
        uint8_t FREQ_UNLOCK_EN1    :1;      /*!< bit: 4  enable the FREQ_UNLOCK status to trigger INT1 pin        */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                                 */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                                 */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                 */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_INTERRUPT1_ENABLE_3;

    union
    {
        struct
        {
        uint8_t ECG_LOFF_NL_EN1    :1;      /*!< bit: 0  enable the ECG_LOFF_NL status to trigger INT1 pin        */
        uint8_t ECG_LOFF_NH_EN1    :1;      /*!< bit: 1  enable the ECG_LOFF_NH status to trigger INT1 pin        */
        uint8_t ECG_LOFF_PL_EN1    :1;      /*!< bit: 2  enable the ECG_LOFF_PL status to trigger INT1 pin        */
        uint8_t ECG_LOFF_PH_EN1    :1;      /*!< bit: 3  enable the ECG_LOFF_PH status to trigger INT1 pin        */
        uint8_t RLD_OOR_EN1        :1;      /*!< bit: 4  enable the RLD_OOR status to trigger INT1 pin            */
        uint8_t ECG_FAST_REC_EN1   :1;      /*!< bit: 5  enable the ECG_FAST_REC status to trigger INT1 pin       */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                                 */
        uint8_t ECG_LON_EN1        :1;      /*!< bit: 7  enable the ECG_LON status to trigger INT1 pin            */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_INTERRUPT1_ENABLE_4;

    union
    {
        struct
        {
        uint8_t BIOZ_LOFF_NL_EN1   :1;      /*!< bit: 0  enable the BIOZ_LOFF_NL status to trigger INT1 pin       */
        uint8_t BIOZ_LOFF_NH_EN1   :1;      /*!< bit: 1  enable the BIOZ_LOFF_NH status to trigger INT1 pin       */
        uint8_t BIOZ_LOFF_PL_EN1   :1;      /*!< bit: 2  enable the BIOZ_LOFF_PL status to trigger INT1 pin       */
        uint8_t BIOZ_LOFF_PH_EN1   :1;      /*!< bit: 3  enable the BIOZ_LOFF_PH status to trigger INT1 pin       */
        uint8_t BIOZ_DRVP_OFF_EN1  :1;      /*!< bit: 4  enable the BIOZ_DRVP_OFF status to trigger INT1 pin      */
        uint8_t BIOZ_UNDR_EN1      :1;      /*!< bit: 5  enable the BIOZ_UNDR status to trigger INT1 pin          */
        uint8_t BIOZ_OVER_EN1      :1;      /*!< bit: 6  enable the BIOZ_OVER status to trigger INT1 pin          */
        uint8_t BIOZ_LON_EN1       :1;      /*!< bit: 7  enable the BIOZ_LON status to trigger INT1 pin           */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_INTERRUPT1_ENABLE_5;

    union
    {
        struct
        {
        uint8_t _reserved_0        :1;      /*!< bit: 0  Reserved                                                 */
        uint8_t PPG_THRESH1_HILO_EN2 :1;    /*!< bit: 1  enable the PPG_THRESH1_HILO status to trigger INT2 pin   */
        uint8_t PPG_THRESH2_HILO_EN2 :1;    /*!< bit: 1  enable the PPG_THRESH2_HILO status to trigger INT2 pin   */
        uint8_t EXP_OVF_EN2        :1;      /*!< bit: 1  enable the EXP_OVF status to trigger INT2 pin            */
        uint8_t ACL_OVF_EN2        :1;      /*!< bit: 1  enable the ACL_OVF status to trigger INT2 pin            */
        uint8_t FIFO_DATA_RDY_EN2  :1;      /*!< bit: 1  enable the FIFO_DATA_RDY status to trigger INT2 pin      */
        uint8_t PPG_FRAME_RDY_EN2  :1;      /*!< bit: 1  enable the PPG_FRAME_RDY status to trigger INT2 pin      */
        uint8_t A_FULL_EN2         :1;      /*!< bit: 1  enable the A_FULL status to trigger INT2 pin             */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_INTERRUPT2_ENABLE_1;

    union
    {
        struct
        {
        uint8_t LED1_COMPB_EN2    :1;      /*!< bit: 0  enable the LED1_COMPB status to trigger INT2 pin         */
        uint8_t LED2_COMPB_EN2    :1;      /*!< bit: 1  enable the LED2_COMPB status to trigger INT2 pin         */
        uint8_t LED3_COMPB_EN2    :1;      /*!< bit: 2  enable the LED3_COMPB status to trigger INT2 pin         */
        uint8_t LED4_COMPB_EN2    :1;      /*!< bit: 3  enable the LED4_COMPB status to trigger INT2 pin         */
        uint8_t LED5_COMPB_EN2    :1;      /*!< bit: 4  enable the LED5_COMPB status to trigger INT2 pin         */
        uint8_t LED6_COMPB_EN2    :1;      /*!< bit: 5  enable the LED6_COMPB status to trigger INT2 pin         */
        uint8_t _reserved_6       :1;      /*!< bit: 6  Reserved                                                 */
        uint8_t INVALID_PPG_CFG_EN2 :1;    /*!< bit: 7  enable the INVALID_PPG_CFG status to trigger INT2 pin    */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_INTERRUPT2_ENABLE_2;

    union
    {
        struct
        {
        uint8_t _reserved_0        :1;      /*!< bit: 0  Reserved                                                 */
        uint8_t PHASE_LOCK_EN2     :1;      /*!< bit: 1  enable the PHASE_LOCK status to trigger INT2 pin         */
        uint8_t PHASE_UNLOCK_EN2   :1;      /*!< bit: 2  enable the PHASE_UNLOCK status to trigger INT2 pin       */
        uint8_t FREQ_LOCK_EN2      :1;      /*!< bit: 3  enable the FREQ_LOCK status to trigger INT2 pin          */
        uint8_t FREQ_UNLOCK_EN2    :1;      /*!< bit: 4  enable the FREQ_UNLOCK status to trigger INT2 pin        */
        uint8_t _reserved_5        :1;      /*!< bit: 5  Reserved                                                 */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                                 */
        uint8_t _reserved_7        :1;      /*!< bit: 7  Reserved                                                 */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_INTERRUPT2_ENABLE_3;

    union
    {
        struct
        {
        uint8_t ECG_LOFF_NL_EN2    :1;      /*!< bit: 0  enable the ECG_LOFF_NL status to trigger INT2 pin        */
        uint8_t ECG_LOFF_NH_EN2    :1;      /*!< bit: 1  enable the ECG_LOFF_NH status to trigger INT2 pin        */
        uint8_t ECG_LOFF_PL_EN2    :1;      /*!< bit: 2  enable the ECG_LOFF_PL status to trigger INT2 pin        */
        uint8_t ECG_LOFF_PH_EN2    :1;      /*!< bit: 3  enable the ECG_LOFF_PH status to trigger INT2 pin        */
        uint8_t RLD_OOR_EN2        :1;      /*!< bit: 4  enable the RLD_OOR status to trigger INT2 pin            */
        uint8_t ECG_FAST_REC_EN2   :1;      /*!< bit: 5  enable the ECG_FAST_REC status to trigger INT2 pin       */
        uint8_t _reserved_6        :1;      /*!< bit: 6  Reserved                                                 */
        uint8_t ECG_LON_EN2        :1;      /*!< bit: 7  enable the ECG_LON status to trigger INT2 pin            */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_INTERRUPT2_ENABLE_4;

    union
    {
        struct
        {
        uint8_t BIOZ_LOFF_NL_EN2   :1;      /*!< bit: 0  enable the BIOZ_LOFF_NL status to trigger INT2 pin       */
        uint8_t BIOZ_LOFF_NH_EN2   :1;      /*!< bit: 1  enable the BIOZ_LOFF_NH status to trigger INT2 pin       */
        uint8_t BIOZ_LOFF_PL_EN2   :1;      /*!< bit: 2  enable the BIOZ_LOFF_PL status to trigger INT2 pin       */
        uint8_t BIOZ_LOFF_PH_EN2   :1;      /*!< bit: 3  enable the BIOZ_LOFF_PH status to trigger INT2 pin       */
        uint8_t BIOZ_DRVP_OFF_EN2  :1;      /*!< bit: 4  enable the BIOZ_DRVP_OFF status to trigger INT2 pin      */
        uint8_t BIOZ_UNDR_EN2      :1;      /*!< bit: 5  enable the BIOZ_UNDR status to trigger INT2 pin          */
        uint8_t BIOZ_OVER_EN2      :1;      /*!< bit: 6  enable the BIOZ_OVER status to trigger INT2 pin          */
        uint8_t BIOZ_LON_EN2       :1;      /*!< bit: 7  enable the BIOZ_LON status to trigger INT2 pin           */
        } b;                                /*!< Structure used for bit access                                    */
        uint8_t w;                          /*!< Type used for word access                                        */
    } REG_INTERRUPT2_ENABLE_5;   

}__attribute__((packed)) MAX86178_REG;

/*! Foramt of the data to be stored in the flash */
typedef struct {
    int16_t header;
    int32_t time_s_bin;
    int16_t time_ms_bin;
    int8_t len;
    uint8_t index;
    uint8_t data[MAX86178_FIFO_READ_SIZE];
} __attribute__((packed)) MAX86178_StorageFormat;


/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/

#ifndef _MAX86178_C_SRC



#endif  /* _MAX86178_C_SRC */

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

/**
 * @brief This function configures the MAX86178 device.
 * 
 * @details The function initializes the MAX86178 related structures and set the I2C instance to be used.   
 * 
 * @return 0 if successful, -1 otherwise.
 */
int max86178_init(void);

/**
 * @brief This function writes a byte of data to a register on the MAX86178 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, writing a byte of data to a register
 *          on the MAX86178 device. The address of the register and the data to be written are provided as parameters.
 * 
 * @param[in]   addr  The address of the register to write to.
 * @param[in]   data  The data to write to the register.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int max86178_write_reg(uint8_t addr, uint8_t data);

/**
 * @brief This function reads a byte of data from a register on the MAX86178 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, reading a byte of data from a register
 *          on the MAX86178 device. The address of the register to read from is provided as a parameter.
 * 
 * @param[in]   addr  The address of the register to read from.
 * @param[out]  data  Pointer to the location where the read data will be stored.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int max86178_read_reg(uint8_t addr, uint8_t *data);

/**
 * @brief This function writes a sequence of registers on the MAX86178 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, writing a sequence of registers on the MAX86178 device.
 *         The starting address of the sequence of registers and the data to be written are provided as parameters.
 * 
 * @param[in]   start_addr  The starting address of the sequence of registers to write to.
 * @param[in]   data        Pointer to the data to write to the registers.
 * @param[in]   num_bytes   The number of bytes to write.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int max86178_write_seq_reg(uint8_t start_addr, const uint8_t *data, uint8_t num_bytes);

/**
 * @brief This function reads a sequence of registers on the MAX86178 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, reading a sequence of registers on the MAX86178 device.
 *         The starting address of the sequence of registers and the number of bytes to read are provided as parameters.
 * 
 * @param[in]   start_addr  The starting address of the sequence of registers to read from.
 * @param[out]  data        Pointer to the location where the read data will be stored.
 * @param[in]   num_bytes   The number of bytes to read.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int max86178_read_seq_reg(uint8_t start_addr, uint8_t *data, uint8_t num_bytes);

/**
 * @brief This function start the thread that manage the continuous measurement form the MAX86178 device.
 * 
 * @details The function creates a thread that will manage the continuous measurement from the MAX86178 device.
 * 
 * @return 0 if the thread was created successfully, -1 otherwise.
 */
int max86178_start_thread(void);

/**
 * @brief This function stop the thread that manage the continuous measurement form the MAX86178 device.
 * 
 * @details The function stops the thread that manages the continuous measurement from the MAX86178 device.
 * 
 * @return 0 if the thread was stopped successfully, -1 otherwise.
 */
int max86178_stop_thread(void);

/**
 * @brief This function configures the MAX86178 device.
 * 
 * @details The function configures the MAX86178 device by writing the necessary configuration registers.
 * 
 * @return 0 if the configuration was successful, -1 otherwise.
 */
int max86178_config(void);

/**
 * @brief This function check if the MAX86178 device is present.
 * 
 * @details The function checks if the MAX86178 device is present by reading the part ID register.
 * 
 * @return 0 if the device is present, -1 otherwise.
 */
int max86178_check_present(void);

/****************************************************************************/
/**                                                                        **/
/**                          INLINE FUNCTIONS                              **/
/**                                                                        **/
/****************************************************************************/



#endif /* _MAX86178_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/
