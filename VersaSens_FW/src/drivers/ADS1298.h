/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : ADS1298.h                                                   **
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
* @file   ADS1298.h
* @date   DD/MM/YY
* @brief  This is the main header of ADS1298.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _ADS1298_H
#define _ADS1298_H

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

/*! ADS1298 SPI settings */
#define ADS_SCK_PIN 29
#define ADS_MOSI_PIN 30
#define ADS_MISO_PIN 32
#define ADS_SS_PIN 7
#define ADS_INT_PIN 28
#define ADS_INST_IDX 3

/*! DEVICE SETTINGS registers*/
#define REG_ID_Addr             0x00 // read-olny ID control register

/*! GLOBAL SETTINGS ACROSS CHANNELS registers*/
#define REG_CONFIG1_Addr        0x01 // Configuration register 1
#define REG_CONFIG2_Addr        0x02 // Configuration register 2
#define REG_CONFIG3_Addr        0x03 // Configuration register 3
#define REG_LOFF_Addr           0x04 // Lead-off control register

/*! CHANNEL-SPECIFIC SETTINGS registers*/
#define REG_CH1SET_Addr         0x05 // Channel 1 settings register
#define REG_CH2SET_Addr         0x06 // Channel 2 settings register
#define REG_CH3SET_Addr         0x07 // Channel 3 settings register
#define REG_CH4SET_Addr         0x08 // Channel 4 settings register
#define REG_CH5SET_Addr         0x09 // Channel 5 settings register
#define REG_CH6SET_Addr         0x0A // Channel 6 settings register
#define REG_CH7SET_Addr         0x0B // Channel 7 settings register
#define REG_CH8SET_Addr         0x0C // Channel 8 settings register
#define REG_RLD_SENSP_Addr      0x0D // RLD Positive Signal Derivation Register
#define REG_RLD_SENSN_Addr      0x0E // RLD Negative Signal Derivation Register
#define REG_LOFF_SENSP_Addr     0x0F // Positive Signal Lead-Off Detection Register
#define REG_LOFF_SENSN_Addr     0x10 // Negative Signal Lead-Off Detection Register
#define REG_LOFF_FLIP_Addr      0x11 // Lead-Off Flip Register

/*! LEAD-OFF STATUS registers*/
#define REG_LOFF_STATP_Addr     0x12 // Lead-Off Positive Signal Status Register
#define REG_LOFF_STATN_Addr     0x13 // Lead-Off Negative Signal Status Register

/*! GPIO AND OTHER registers*/
#define REG_GPIO_Addr           0x14 // General-Purpose I/O Register
#define REG_PACE_Addr           0x15 // PACE Detect Register
#define REG_RESP_Addr           0x16 // Respiration Control Register
#define REG_CONFIG4_Addr        0x17 // Configuration Register 4
#define REG_WCT1_Addr           0x18 // Wilson Central Terminal 1 Register
#define REG_WCT2_Addr           0x19 // Wilson Central Terminal 2 Register

/*! ADS1298 Commands*/
#define WAKEUP_CMD              0x02
#define STANDBY_CMD             0x04
#define RESET_CMD               0x06
#define START_CMD               0x08
#define STOP_CMD                0x0A
#define RDATAC_CMD              0x10
#define SDATAC_CMD              0x11
#define RDATA_CMD               0x12

/*! ADS1298 SPI settings */
#define ADS1298_CONSEC_MEAS     7

/*! ADS1298 Registers Default Values*/
#define ADS1298_CH_SET_EEG      0x60
#define ADS1298_CH_SET_ECG      0x00
#define ADS1298_CH_SET_TEST     0x05

/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

/*!ADC1298 Registers Stucture*/
typedef struct {  

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t DEV_ID              :3;      /*!< bit: 0-2  Device ID                                  */
        uint8_t _reserved_3         :1;      /*!< bit: 3  Reserved                                     */
        uint8_t _reserved_4         :1;      /*!< bit: 4  Reserved                                     */
        uint8_t CH_ID               :3;      /*!< bit: 5-7  Channel ID                                 */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_ID; 

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t DR                  :3;      /*!< bit: 0-2  Output data rate                           */
        uint8_t _reserved_3         :1;      /*!< bit: 3  Reserved                                     */
        uint8_t _reserved_4         :1;      /*!< bit: 4  Reserved                                     */
        uint8_t CLK_EN              :1;      /*!< bit: 5  CLK connection                               */
        uint8_t DAISY_EN            :1;      /*!< bit: 6  Daisy-chain or multiple readback mode        */
        uint8_t HR                  :1;      /*!< bit: 7  High-resolution or low-power mode            */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_CONFIG1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t TEST_FREQ           :2;      /*!< bit: 0-1  Test signal frequency                      */
        uint8_t TEST_AMP            :1;      /*!< bit: 2  Test signal amplitude                        */
        uint8_t _reserved_3         :1;      /*!< bit: 3  Reserved                                     */
        uint8_t INT_TEST            :1;      /*!< bit: 4  TEST source                                  */
        uint8_t WCT_CHOP            :1;      /*!< bit: 5  WCT chopping scheme                          */
        uint8_t _reserved_6         :1;      /*!< bit: 6  Reserved                                     */
        uint8_t _reserved_7         :1;      /*!< bit: 7  Reserved                                     */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_CONFIG2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t RLD_STAT            :1;      /*!< bit: 0  RLD lead-off status                          */
        uint8_t RLD_LOFF_SENS       :1;      /*!< bit: 1  RLD sense function                           */
        uint8_t PD_RLD              :1;      /*!< bit: 2  RLD buffer power                             */
        uint8_t RLDREF_INT          :1;      /*!< bit: 3  RLDREF signal                                */
        uint8_t RLD_MEAS            :1;      /*!< bit: 4  RLD measurement                              */
        uint8_t VREF_4V             :1;      /*!< bit: 5  Reference voltage                            */
        uint8_t _reserved_6         :1;      /*!< bit: 6  Reserved                                     */
        uint8_t PD_REFBUF           :1;      /*!< bit: 7  Power-down reference buffer                  */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_CONFIG3;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t FLEAD_OFF           :2;      /*!< bit: 0-1  Lead-off frequency                         */
        uint8_t ILEAD_OFF           :2;      /*!< bit: 2-3  Lead-off current magnitude                 */
        uint8_t VLEAD_OFF_EN        :1;      /*!< bit: 4  Lead-off detection mode                      */
        uint8_t COMP_TH             :3;      /*!< bit: 5-7  Lead-off comparator threshold              */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_LOFF;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MUX1                :3;      /*!< bit: 0-2  Channel input                              */
        uint8_t _reserved_3         :1;      /*!< bit: 3  Reserved                                     */
        uint8_t GAIN1               :3;      /*!< bit: 4-6  PGA gain                                   */
        uint8_t PD1                 :1;      /*!< bit: 7  Power-down                                   */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_CH1SET;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MUX2                :3;      /*!< bit: 0-2  Channel input                              */
        uint8_t _reserved_3         :1;      /*!< bit: 3  Reserved                                     */
        uint8_t GAIN2               :3;      /*!< bit: 4-6  PGA gain                                   */
        uint8_t PD2                 :1;      /*!< bit: 7  Power-down                                   */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_CH2SET;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MUX3                :3;      /*!< bit: 0-2  Channel input                              */
        uint8_t _reserved_3         :1;      /*!< bit: 3  Reserved                                     */
        uint8_t GAIN3               :3;      /*!< bit: 4-6  PGA gain                                   */
        uint8_t PD3                 :1;      /*!< bit: 7  Power-down                                   */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_CH3SET;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MUX4                :3;      /*!< bit: 0-2  Channel input                              */
        uint8_t _reserved_3         :1;      /*!< bit: 3  Reserved                                     */
        uint8_t GAIN4               :3;      /*!< bit: 4-6  PGA gain                                   */
        uint8_t PD4                 :1;      /*!< bit: 7  Power-down                                   */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_CH4SET;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MUX5                :3;      /*!< bit: 0-2  Channel input                              */
        uint8_t _reserved_3         :1;      /*!< bit: 3  Reserved                                     */
        uint8_t GAIN5               :3;      /*!< bit: 4-6  PGA gain                                   */
        uint8_t PD5                 :1;      /*!< bit: 7  Power-down                                   */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_CH5SET;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MUX6                :3;      /*!< bit: 0-2  Channel input                              */
        uint8_t _reserved_3         :1;      /*!< bit: 3  Reserved                                     */
        uint8_t GAIN6               :3;      /*!< bit: 4-6  PGA gain                                   */
        uint8_t PD6                 :1;      /*!< bit: 7  Power-down                                   */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_CH6SET;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MUX7                :3;      /*!< bit: 0-2  Channel input                              */
        uint8_t _reserved_3         :1;      /*!< bit: 3  Reserved                                     */
        uint8_t GAIN7               :3;      /*!< bit: 4-6  PGA gain                                   */
        uint8_t PD7                 :1;      /*!< bit: 7  Power-down                                   */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_CH7SET;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MUX8                :3;      /*!< bit: 0-2  Channel input                              */
        uint8_t _reserved_3         :1;      /*!< bit: 3  Reserved                                     */
        uint8_t GAIN8               :3;      /*!< bit: 4-6  PGA gain                                   */
        uint8_t PD8                 :1;      /*!< bit: 7  Power-down                                   */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_CH8SET;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t RLD1P               :1;      /*!< bit: 0  IN1P to RLD                                  */
        uint8_t RLD2P               :1;      /*!< bit: 1  IN2P to RLD                                  */
        uint8_t RLD3P               :1;      /*!< bit: 2  IN3P to RLD                                  */
        uint8_t RLD4P               :1;      /*!< bit: 3  IN4P to RLD                                  */
        uint8_t RLD5P               :1;      /*!< bit: 4  IN5P to RLD                                  */
        uint8_t RLD6P               :1;      /*!< bit: 5  IN6P to RLD                                  */
        uint8_t RLD7P               :1;      /*!< bit: 6  IN7P to RLD                                  */
        uint8_t RLD8P               :1;      /*!< bit: 7  IN8P to RLD                                  */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_RLD_SENSP;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t RLD1N               :1;      /*!< bit: 0  IN1N to RLD                                  */
        uint8_t RLD2N               :1;      /*!< bit: 1  IN2N to RLD                                  */
        uint8_t RLD3N               :1;      /*!< bit: 2  IN3N to RLD                                  */
        uint8_t RLD4N               :1;      /*!< bit: 3  IN4N to RLD                                  */
        uint8_t RLD5N               :1;      /*!< bit: 4  IN5N to RLD                                  */
        uint8_t RLD6N               :1;      /*!< bit: 5  IN6N to RLD                                  */
        uint8_t RLD7N               :1;      /*!< bit: 6  IN7N to RLD                                  */
        uint8_t RLD8N               :1;      /*!< bit: 7  IN8N to RLD                                  */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_RLD_SENSN;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t LOFF1P              :1;      /*!< bit: 0  IN1P lead off                                */
        uint8_t LOFF2P              :1;      /*!< bit: 1  IN2P lead off                                */
        uint8_t LOFF3P              :1;      /*!< bit: 2  IN3P lead off                                */
        uint8_t LOFF4P              :1;      /*!< bit: 3  IN4P lead off                                */
        uint8_t LOFF5P              :1;      /*!< bit: 4  IN5P lead off                                */
        uint8_t LOFF6P              :1;      /*!< bit: 5  IN6P lead off                                */
        uint8_t LOFF7P              :1;      /*!< bit: 6  IN7P lead off                                */
        uint8_t LOFF8P              :1;      /*!< bit: 7  IN8P lead off                                */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_LOFF_SENSP;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t LOFF1N              :1;      /*!< bit: 0  IN1N lead off                                */
        uint8_t LOFF2N              :1;      /*!< bit: 1  IN2N lead off                                */
        uint8_t LOFF3N              :1;      /*!< bit: 2  IN3N lead off                                */
        uint8_t LOFF4N              :1;      /*!< bit: 3  IN4N lead off                                */
        uint8_t LOFF5N              :1;      /*!< bit: 4  IN5N lead off                                */
        uint8_t LOFF6N              :1;      /*!< bit: 5  IN6N lead off                                */
        uint8_t LOFF7N              :1;      /*!< bit: 6  IN7N lead off                                */
        uint8_t LOFF8N              :1;      /*!< bit: 7  IN8N lead off                                */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_LOFF_SENSN;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t LOFF_FLIP1          :1;      /*!< bit: 0  Channel 1 LOFF Polarity Flip                 */
        uint8_t LOFF_FLIP2          :1;      /*!< bit: 1  Channel 2 LOFF Polarity Flip                 */
        uint8_t LOFF_FLIP3          :1;      /*!< bit: 2  Channel 3 LOFF Polarity Flip                 */
        uint8_t LOFF_FLIP4          :1;      /*!< bit: 3  Channel 4 LOFF Polarity Flip                 */
        uint8_t LOFF_FLIP5          :1;      /*!< bit: 4  Channel 5 LOFF Polarity Flip                 */
        uint8_t LOFF_FLIP6          :1;      /*!< bit: 5  Channel 6 LOFF Polarity Flip                 */
        uint8_t LOFF_FLIP7          :1;      /*!< bit: 6  Channel 7 LOFF Polarity Flip                 */
        uint8_t LOFF_FLIP8          :1;      /*!< bit: 7  Channel 8 LOFF Polarity Flip                 */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_LOFF_FLIP;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t IN1P_OFF            :1;      /*!< bit: 0  Channel 1 positive channel lead-off status   */
        uint8_t IN2P_OFF            :1;      /*!< bit: 1  Channel 2 positive channel lead-off status   */
        uint8_t IN3P_OFF            :1;      /*!< bit: 2  Channel 3 positive channel lead-off status   */
        uint8_t IN4P_OFF            :1;      /*!< bit: 3  Channel 4 positive channel lead-off status   */
        uint8_t IN5P_OFF            :1;      /*!< bit: 4  Channel 5 positive channel lead-off status   */
        uint8_t IN6P_OFF            :1;      /*!< bit: 5  Channel 6 positive channel lead-off status   */
        uint8_t IN7P_OFF            :1;      /*!< bit: 6  Channel 7 positive channel lead-off status   */
        uint8_t IN8P_OFF            :1;      /*!< bit: 7  Channel 8 positive channel lead-off status   */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_LOFF_STATP;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t IN1N_OFF            :1;      /*!< bit: 0  Channel 1 negative channel lead-off status   */
        uint8_t IN2N_OFF            :1;      /*!< bit: 1  Channel 2 negative channel lead-off status   */
        uint8_t IN3N_OFF            :1;      /*!< bit: 2  Channel 3 negative channel lead-off status   */
        uint8_t IN4N_OFF            :1;      /*!< bit: 3  Channel 4 negative channel lead-off status   */
        uint8_t IN5N_OFF            :1;      /*!< bit: 4  Channel 5 negative channel lead-off status   */
        uint8_t IN6N_OFF            :1;      /*!< bit: 5  Channel 6 negative channel lead-off status   */
        uint8_t IN7N_OFF            :1;      /*!< bit: 6  Channel 7 negative channel lead-off status   */
        uint8_t IN8N_OFF            :1;      /*!< bit: 7  Channel 8 negative channel lead-off status   */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_LOFF_STATN;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t GPIOC               :4;      /*!< bit: 0-3  GPIO control (corresponding GPIOD)         */
        uint8_t GPIOD               :4;      /*!< bit: 4-7  GPIO data                                  */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_GPIO;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t PD_PACE             :1;      /*!< bit: 0  Pace detect buffer                           */
        uint8_t PACEO               :2;      /*!< bit: 1-2  Pace odd channels                          */
        uint8_t PACEE               :2;      /*!< bit: 3-4  Pace even channels                         */
        uint8_t _reserved_5         :1;      /*!< bit: 5  Reserved                                     */
        uint8_t _reserved_6         :1;      /*!< bit: 6  Reserved                                     */
        uint8_t _reserved_7         :1;      /*!< bit: 7  Reserved                                     */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_PACE;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t RESP_CTRL           :2;      /*!< bit: 0-1  Respiration control                        */
        uint8_t RESP_PH             :3;      /*!< bit: 2-4  Respiration phase                          */
        uint8_t _reserved_5         :1;      /*!< bit: 5  Reserved                                     */
        uint8_t RESP_MOD_EN1        :1;      /*!< bit: 6  Enables respiration modulation circuitry     */
        uint8_t RESP_DEMOD_EN1      :1;      /*!< bit: 7  Enables respiration demodulation circuitry   */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_RESP;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t _reserved_0         :1;      /*!< bit: 0  Reserved                                     */
        uint8_t PD_LOFF_COMP        :1;      /*!< bit: 1  Lead-off comparator power-down               */
        uint8_t WCT_TO_RLD          :1;      /*!< bit: 2  Connects the WCT to the RLD                  */
        uint8_t SINGLE_SHOT         :1;      /*!< bit: 3  Single-shot conversion                       */
        uint8_t _reserved_4         :1;      /*!< bit: 4  Reserved                                     */
        uint8_t RESP_FREQ           :3;      /*!< bit: 5-7  Respiration modulation frequency           */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_CONFIG4;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t WCTA                :3;      /*!< bit: 0-2  WCT Amplifier A channel selection          */
        uint8_t PD_WCTA             :1;      /*!< bit: 3  Power-down WCTA                              */
        uint8_t aVR_CH4             :1;      /*!< bit: 4  Enable (WCTB + WCTC)/2 to the negative input of channel 4     */
        uint8_t aVR_CH7             :1;      /*!< bit: 5  Enable (WCTB + WCTC)/2 to the negative input of channel 7     */
        uint8_t aVL_CH5             :1;      /*!< bit: 6  Enable (WCTA + WCTC)/2 to the negative input of channel 5     */
        uint8_t aVF_CH6             :1;      /*!< bit: 7  Enable (WCTA + WCTB)/2 to the negative input of channel 6     */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_WCT1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t WCTC                :3;      /*!< bit: 0-2  WCT Amplifier C channel selection          */
        uint8_t WCTB                :3;      /*!< bit: 3-5  WCT Amplifier B channel selection          */
        uint8_t PD_WCTB             :1;      /*!< bit: 6  Power-down WCTB                              */
        uint8_t PD_WCTC             :1;      /*!< bit: 7  Power-down WCTC                              */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w;                           /*!< Type used for word access                            */
    } REG_WCT2;

}__attribute__((packed)) ADS1298_REG;

/*! Foramt of the data to be stored in the flash */
typedef struct {
    int16_t header;
    int32_t rawtime_bin;
    int16_t time_ms_bin;
    uint8_t len;
    uint8_t index;
    uint8_t measurements[24];
} __attribute__((packed)) ADS1298_StorageFormat;

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/

#ifndef _ADS1298_C_SRC



#endif  /* _ADS1298_C_SRC */

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

/**
 * @brief  This function initializes the ADS1298 spi instance
 * 
 * @param  void
 * 
 * @return 0 if successful, -1 if failed, -2 if not present 
 */
int ADS1298_init(void);

/**
 * @brief  This function sends a command to the ADS1298
 * 
 * @param  cmd : command to send
 * 
 * @return 0 if successful, -1 otherwise 
 */
int ADS1298_send_cmd(uint8_t cmd);

/**
 * @brief  This function reads data from the ADS1298
 * 
 * @param  data : pointer to the data buffer
 * 
 * @return 0 if successful, -1 otherwise 
 */
int ADS1298_read_data(uint8_t *data);

/**
 * @brief  This function initializes the ADS1298 interrupt
 * 
 * @param  void
 * 
 * @return 0 if successful, -1 otherwise 
 */
int ADS1298_init_interrupt(void);

/**
 * @brief  This function starts the ADS1298 thread
 * 
 * @param  void
 * 
 * @return 0 if successful, -1 otherwise
 */
int ADS1298_start_thread(void);

/**
 * @brief  This function stops the ADS1298 thread
 * 
 * @param  void
 * 
 * @return 0 if successful, -1 otherwise 
 */
int ADS1298_stop_thread(void);

/**
 * @brief  This function configures the ADS1298 registers
 * 
 * @param  void
 * 
 * @return 0 if successful, -1 otherwise
 */
int ADS1298_config(void);

/**
 * @brief  This function checks if the ADS1298 is present
 * 
 * @param  void
 * 
 * @return 0 if successful, -1 otherwise 
 */
int ADS1298_check_present(void);

/**
 * @brief  This function reads a register from the ADS1298
 * 
 * @param  addr : address of the register
 * @param  data : pointer to the data buffer
 * 
 * @return 0 if successful, -1 otherwise 
 */
int ADS1298_read_reg(uint8_t addr, uint8_t *data);

/**
 * @brief  This function writes a register to the ADS1298
 * 
 * @param  addr : address of the register
 * @param  data : data to write
 * 
 * @return 0 if successful, -1 otherwise 
 */
int ADS1298_write_reg(uint8_t addr, uint8_t data);

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
