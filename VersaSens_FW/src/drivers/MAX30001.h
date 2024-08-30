/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : MAX30001.h                                                   **
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
* @file   MAX30001.h
* @date   DD/MM/YY
* @brief  This is the main header of MAX30001.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _MAX30001_H
#define _MAX30001_H

/****************************************************************************/
/**                                                                        **/
/**                            MODULES USED                                **/
/**                                                                        **/
/****************************************************************************/

// #include "sdk_common.h"
#include <zephyr/types.h>
#include "twim_inst.h"
#include "thread_config.h"
#include "versa_config.h"

/****************************************************************************/
/**                                                                        **/
/**                       DEFINITIONS AND MACROS                           **/
/**                                                                        **/
/****************************************************************************/

#define MAX_SCK_PIN 29
#define MAX_MOSI_PIN 30
#define MAX_MISO_PIN 32
#define MAX_SS_PIN 46
#define MAX_INT_PIN 44

/*!MAX30001 SPIM Instance*/
#define MAX_INST_IDX 3

/*Format sizes*/
#define FORMAT_SIZE_EEG         13
#define FORMAT_SIZE_EEG_BIOZ    16

/*Consecutive samples to be stored at a time*/
#define CONSEC_SAMPLES          12 

/*!MAX30001 Registers Addresses*/
#define REG_NO_OP_00_Addr           0x00 // No Operation registers
#define REG_STATUS_Addr             0x01 // Status register
#define REG_EN_INT_Addr             0x02 // Interrupt 1 configuration register
#define REG_EN_INT2_Addr            0x03 // Interrupt 2 configuration register
#define REG_MNGR_INT_Addr           0x04 // Interrupt manager configuration register
#define REG_MNGR_DYN_Addr           0x05 // Interrupt manager dynamic register

#define REG_SW_RST_Addr             0x08 // Software reset register
#define REG_SYNCH_Addr              0x09 // Synchronization register
#define REG_FIFO_RST_Addr           0x0A // FIFO reset register

#define REG_INFO_Addr               0x0F // Information register
#define REG_CNFG_GEN_Addr           0x10 // General configuration register
#define REG_CNFG_CAL_Addr           0x12 // Calibration configuration register
#define REG_CNFG_EMUX_Addr          0x14 // External multiplexer configuration register
#define REG_CNFG_ECG_Addr           0x15 // ECG configuration register

#define REG_CNFG_BMUX_Addr          0x17 // Bias lead-off multiplexer configuration register
#define REG_CNFG_BIOZ_Addr          0x18 // BioZ configuration register

#define REG_CNFG_PACE_Addr          0x1A // Pace configuration register

#define REG_CNFG_RTOR1_Addr         0x1D // R-to-R configuration register 1
#define REG_CNFG_RTOR2_Addr         0x1E // R-to-R configuration register 2

#define REG_ECG_FIFO_BURST_Addr     0x20 // ECG FIFO burst register
#define REG_ECG_FIFO_Addr          0x21 // ECG FIFO register
#define REG_BIOZ_FIFO_BURST_Addr    0x22 // BioZ FIFO burst register
#define REG_BIOZ_FIFO_Addr          0x23 // BioZ FIFO register

#define REG_RTOR_Addr               0x25 // R-to-R register

#define REG_PACE0_BURST_Addr        0x30 // Pace 0 burst register
#define REG_PACE0_A_Addr            0x31 // Pace 0 register A
#define REG_PACE0_B_Addr            0x32 // Pace 0 register B
#define REG_PACE0_C_Addr            0x33 // Pace 0 register C
#define REG_PACE1_BURST_Addr        0x34 // Pace 1 burst register
#define REG_PACE1_A_Addr            0x35 // Pace 1 register A
#define REG_PACE1_B_Addr            0x36 // Pace 1 register B
#define REG_PACE1_C_Addr            0x37 // Pace 1 register C
#define REG_PACE2_BURST_Addr        0x38 // Pace 2 burst register
#define REG_PACE2_A_Addr            0x39 // Pace 2 register A
#define REG_PACE2_B_Addr            0x3A // Pace 2 register B
#define REG_PACE2_C_Addr            0x3B // Pace 2 register C
#define REG_PACE3_BURST_Addr        0x3C // Pace 3 burst register
#define REG_PACE3_A_Addr            0x3D // Pace 3 register A
#define REG_PACE3_B_Addr            0x3E // Pace 3 register B
#define REG_PACE3_C_Addr            0x3F // Pace 3 register C
#define REG_PACE4_BURST_Addr        0x40 // Pace 4 burst register
#define REG_PACE4_A_Addr            0x41 // Pace 4 register A
#define REG_PACE4_B_Addr            0x42 // Pace 4 register B
#define REG_PACE4_C_Addr            0x43 // Pace 4 register C
#define REG_PACE5_BURST_Addr        0x44 // Pace 5 burst register
#define REG_PACE5_A_Addr            0x45 // Pace 5 register A
#define REG_PACE5_B_Addr            0x46 // Pace 5 register B
#define REG_PACE5_C_Addr            0x47 // Pace 5 register C

#define REG_NO_OP_7F_Addr           0x7F // No Operation registers


/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

/*!MAX30001 Registers Stucture*/
typedef struct {  

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t _reserved_0_7       :8;      /*!< bit: 0-7  Reserved                                   */
        uint8_t _reserved_8_15      :8;      /*!< bit: 8-16 Reserved                                   */
        uint8_t _reserved_16_23     :8;      /*!< bit: 16-24 Reserved                                  */
        } b;                                 /*!< Structure used for bit access                        */
        uint8_t w[3];                        /*!< Type used for word access                            */
    } REG_NO_OP_00; 

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t LDOFF_NL            :1;      /*!< bit: 0  Lead-off negative electrode status low threshold      */
        uint8_t LDOFF_NH            :1;      /*!< bit: 1  Lead-off negative electrode status high threshold     */
        uint8_t LDOFF_PL            :1;      /*!< bit: 2  Lead-off positive electrode status low threshold      */
        uint8_t LDOFF_PH            :1;      /*!< bit: 3  Lead-off positive electrode status high threshold     */
        uint8_t BCGMN               :1;      /*!< bit: 4  BioZ Current Generator Monitor Negative Output        */
        uint8_t BCGMP               :1;      /*!< bit: 5  BioZ Current Generator Monitor Positive Output        */
        uint8_t _reserved_6         :1;      /*!< bit: 6  Reserved                                              */
        uint8_t _reserved_7         :1;      /*!< bit: 7  Reserved                                              */
        uint8_t PLLINT              :1;      /*!< bit: 8  PLL Unlocked Interrupt                                */
        uint8_t SAMP                :1;      /*!< bit: 9  Sample Synchronization Pulse                          */
        uint8_t RRINT               :1;      /*!< bit: 10 ECG R-to-R Detector R Event Interrupt                 */
        uint8_t LONINT              :1;      /*!< bit: 11 Ultra-Low Power (ULP) Leads-On Detection Interrupt    */ 
        uint8_t PEDGE               :1;      /*!< bit: 12 PACE Edge Detection Interrupt                         */
        uint8_t POVF                :1;      /*!< bit: 13 PACE FIFO Overflow                                    */
        uint8_t PINT                :1;      /*!< bit: 14 PACE FIFO Interrupt                                   */
        uint8_t BCGMON              :1;      /*!< bit: 15 BioZ Current Generator Monitor                        */
        uint8_t BUNDR               :1;      /*!< bit: 16 BioZ Under Range                                      */
        uint8_t BOVR                :1;      /*!< bit: 17 BioZ Over Range                                       */
        uint8_t BOVF                :1;      /*!< bit: 18 BioZ FIFO Overflow                                    */
        uint8_t BINT                :1;      /*!< bit: 19 BioZ FIFO Interrupt                                   */
        uint8_t DCLOFFINT           :1;      /*!< bit: 20 DC Lead-Off Detection Interrupt                       */
        uint8_t FSTINT              :1;      /*!< bit: 21 ECG Fast Recovery Mode                                */
        uint8_t EOVF                :1;      /*!< bit: 22 ECG FIFO Overflow                                     */
        uint8_t EINT                :1;      /*!< bit: 23 ECG FIFO Interrupt                                    */
        } b;                                 /*!< Structure used for bit access                                 */
        uint8_t w[3];                        /*!< Type used for word access                                     */
    } REG_STATUS;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t INTB_TYPE           :2;      /*!< bit: 0-1  INTB Port Type (EN_INT Selections)                  */
        uint8_t _reserved_2_7       :6;      /*!< bit: 2-7  Reserved                                            */
        uint8_t EN_PLLINT           :1;      /*!< bit: 8  Interrupt Enables for PLLint                          */
        uint8_t EN_SAMP             :1;      /*!< bit: 9  Interrupt Enables for SAMP                            */
        uint8_t EN_RRINT            :1;      /*!< bit: 10 Interrupt Enables for RRINT                           */
        uint8_t EN_LONINT           :1;      /*!< bit: 11 Interrupt Enables for LONINT                          */
        uint8_t EN_PEDGE            :1;      /*!< bit: 12 Interrupt Enables for PEDGE                           */
        uint8_t EN_POVF             :1;      /*!< bit: 13 Interrupt Enables for POVF                            */
        uint8_t EN_PINT             :1;      /*!< bit: 14 Interrupt Enables for PINT                            */
        uint8_t EN_BCGMON           :1;      /*!< bit: 15 Interrupt Enables for BCGMON                          */
        uint8_t EN_BUNDR            :1;      /*!< bit: 16 Interrupt Enables for BUNDR                           */
        uint8_t EN_BOVR             :1;      /*!< bit: 17 Interrupt Enables for BOVR                            */
        uint8_t EN_BOVF             :1;      /*!< bit: 18 Interrupt Enables for BOVF                            */
        uint8_t EN_BINT             :1;      /*!< bit: 19 Interrupt Enables for BINT                            */
        uint8_t EN_DCLOFFINT        :1;      /*!< bit: 20 Interrupt Enables for DCLOFFINT                       */
        uint8_t EN_FSTINT           :1;      /*!< bit: 21 Interrupt Enables for FSTINT                          */
        uint8_t EN_EOVF             :1;      /*!< bit: 22 Interrupt Enables for EOVF                            */
        uint8_t EN_EINT             :1;      /*!< bit: 23 Interrupt Enables for EINT                            */
        } b;                                 /*!< Structure used for bit access                                 */
        uint8_t w[3];                        /*!< Type used for word access                                     */
    } REG_EN_INT;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t INTB_TYPE           :2;      /*!< bit: 0-1  INTB Port Type (EN_INT Selections)                  */
        uint8_t _reserved_2_7       :6;      /*!< bit: 2-7  Reserved                                            */
        uint8_t EN_PLLINT           :1;      /*!< bit: 8  Interrupt Enables for PLLint                          */
        uint8_t EN_SAMP             :1;      /*!< bit: 9  Interrupt Enables for SAMP                            */
        uint8_t EN_RRINT            :1;      /*!< bit: 10 Interrupt Enables for RRINT                           */
        uint8_t EN_LONINT           :1;      /*!< bit: 11 Interrupt Enables for LONINT                          */
        uint8_t EN_PEDGE            :1;      /*!< bit: 12 Interrupt Enables for PEDGE                           */
        uint8_t EN_POVF             :1;      /*!< bit: 13 Interrupt Enables for POVF                            */
        uint8_t EN_PINT             :1;      /*!< bit: 14 Interrupt Enables for PINT                            */
        uint8_t EN_BCGMON           :1;      /*!< bit: 15 Interrupt Enables for BCGMON                          */
        uint8_t EN_BUNDR            :1;      /*!< bit: 16 Interrupt Enables for BUNDR                           */
        uint8_t EN_BOVR             :1;      /*!< bit: 17 Interrupt Enables for BOVR                            */
        uint8_t EN_BOVF             :1;      /*!< bit: 18 Interrupt Enables for BOVF                            */
        uint8_t EN_BINT             :1;      /*!< bit: 19 Interrupt Enables for BINT                            */
        uint8_t EN_DCLOFFINT        :1;      /*!< bit: 20 Interrupt Enables for DCLOFFINT                       */
        uint8_t EN_FSTINT           :1;      /*!< bit: 21 Interrupt Enables for FSTINT                          */
        uint8_t EN_EOVF             :1;      /*!< bit: 22 Interrupt Enables for EOVF                            */
        uint8_t EN_EINT             :1;      /*!< bit: 23 Interrupt Enables for EINT                            */
        } b;                                 /*!< Structure used for bit access                                 */
        uint8_t w[3];                        /*!< Type used for word access                                     */
    } REG_EN_INT2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t SAMP_IT             :2;      /*!< bit: 0-1  Sample Synchronization Pulse (SAMP) Frequency       */
        uint8_t CLR_SAMP            :1;      /*!< bit: 2  Sample Synchronization Pulse (SAMP) Clear Behavior    */
        uint8_t CLR_PEDGE           :1;      /*!< bit: 3  PACE Edge Detection Clear Behavior                    */
        uint8_t CLR_RRINT           :2;      /*!< bit: 4-5  ECG R-to-R Detector R Event Clear Behavior          */
        uint8_t CLR_FAST            :1;      /*!< bit: 6  FAST MODE Interrupt Clear Behavior                    */
        uint8_t _reserved_7         :1;      /*!< bit: 7  Reserved                                              */
        uint8_t _reserved_8_15      :8;      /*!< bit: 8-15  Reserved                                           */
        uint8_t BFIT                :3;      /*!< bit: 16-18 BioZ FIFO Interrupt Threshold                      */
        uint8_t EFIT                :5;      /*!< bit: 19  ECG FIFO Interrupt Threshold                         */
        } b;                                 /*!< Structure used for bit access                                 */
        uint8_t w[3];                        /*!< Type used for word access                                     */
    } REG_MNGR_INT;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t BLOFF_LO_IT         :8;      /*!< bit: 0-7  BioZ AC Lead Off Under-Range Threshold               */
        uint8_t BLOFF_HI_IT         :8;      /*!< bit: 8-15 BioZ AC Lead Off Over-Range Threshold                */
        uint8_t FAST_TH             :6;      /*!< bit: 16-21 Automatic Fast Recovery Threshold                   */
        uint8_t FAST                :2;      /*!< bit: 22-23 ECG Channel Fast Recovery Mode Selection            */
        } b;                                 /*!< Structure used for bit access                                  */
        uint8_t w[3];                        /*!< Type used for word access                                      */
    } REG_MNGR_DYN;

    uint8_t _unused_06[3];       // Unused register
    uint8_t _unused_07[3];       // Unused register

    uint8_t REG_SW_RST[3];      // Software reset register
    uint8_t REG_SYNCH[3];       // Synchronization register
    uint8_t REG_FIFO_RST[3];    // FIFO reset register

    uint8_t _unused_0B[3];      // Unused register
    uint8_t _unused_0C[3];      // Unused register
    uint8_t _unused_0D[3];      // Unused register
    uint8_t _unused_0E[3];      // Unused register

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t _reserved_0_7       :8;      /*!< bit: 0-7  Reserved                                           */
        uint8_t _reserved_8_15      :8;      /*!< bit: 8-16 Reserved                                           */
        uint8_t REV_ID              :4;      /*!< bit: 16-23 Revision ID                                       */       
        uint8_t _reserved_20_23     :4;      /*!< bit: 20-23 Reserved                                          */
        } b;                                 /*!< Structure used for bit access                                */
        uint8_t w[3];                        /*!< Type used for word access                                    */
    } REG_INFO;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t RBIASN              :1;      /*!< bit: 0  Enables Resistive Bias on Negative Input              */
        uint8_t RBIASP              :1;      /*!< bit: 1  Enables Resistive Bias on Positive Input              */
        uint8_t RBIASV              :2;      /*!< bit: 2-3  Resistive Bias Mode Value Selection                 */
        uint8_t EN_RBIAS            :2;      /*!< bit: 4-5  Enable and Select Resistive Lead Bias Mode          */
        uint8_t VTH                 :2;      /*!< bit: 6-7  DC Lead-Off Voltage Threshold Selection             */
        uint8_t IMAG                :3;      /*!< bit: 8-10  DC Lead-Off Current Magnitude Selection            */
        uint8_t DCLOFF_IPOL         :1;      /*!< bit: 11  DC Lead-Off Current Polarity                         */
        uint8_t EN_DCLOFF           :2;      /*!< bit: 12-13  DC Lead-Off Detection Enable                      */
        uint8_t EN_BLOFF            :2;      /*!< bit: 14-15  BioZ Lead-Off Detection Enable                    */
        uint8_t _reserved_16        :1;      /*!< bit: 16  Reserved                                             */
        uint8_t EN_PACE             :1;      /*!< bit: 17  PACE Channel Enable                                  */
        uint8_t EN_BIOZ             :1;      /*!< bit: 18  BioZ Channel Enable                                  */
        uint8_t EN_ECG              :1;      /*!< bit: 19  ECG Channel Enable                                   */
        uint8_t FMSTR               :2;      /*!< bit: 20-21  Master Clock Frequency                            */
        uint8_t EN_ULP_LON          :2;      /*!< bit: 22-23  Ultra-Low Power Lead-On Detection Enable          */
        } b;                                 /*!< Structure used for bit access                                 */
        uint8_t w[3];                        /*!< Type used for word access                                     */
    } REG_CNFG_GEN;

    uint8_t _unused_11[3];      // Unused register

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint16_t THIGH              :11;     /*!< bit: 0-10  Calibration Source Time High Selection             */
        uint8_t FIFTY               :1;      /*!< bit: 11  Calibration Source Duty Cycle Mode Selection         */
        uint8_t FCAL                :3;      /*!< bit: 12-14  Calibration Source Frequency Selection (FCAL)     */
        uint8_t _reserved_15_19     :5;      /*!< bit: 15-19  Reserved                                          */
        uint8_t VMAG                :1;      /*!< bit: 20  Calibration Source Magnitude Selection               */
        uint8_t VMODE               :1;      /*!< bit: 21  Calibration Source Mode Selection                    */
        uint8_t EN_VCAL             :1;      /*!< bit: 22  Calibration Source (VCALP and VCALN) Enable          */
        uint8_t _reserved_23        :1;      /*!< bit: 23  Reserved                                             */
        } b;                                 /*!< Structure used for bit access                                 */
        uint8_t w[3];                        /*!< Type used for word access                                     */
    } REG_CNFG_CAL;

    uint8_t _unused_13[3];      // Unused register

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint16_t _reserved_0_15     :16;     /*!< bit: 0-15  Reserved                                           */
        uint8_t ECG_CALN_SEL        :2;      /*!< bit: 16-17 ECGN Calibration Selection                         */
        uint8_t ECG_CALP_SEL        :2;      /*!< bit: 18-19 ECGP Calibration Selection                         */
        uint8_t ECG_OPENN           :1;      /*!< bit: 20 Open the ECGN Input Switch                            */
        uint8_t ECG_OPENP           :1;      /*!< bit: 21 Open the ECGP Input Switch                            */
        uint8_t _reserved_22        :1;      /*!< bit: 22 Reserved                                              */
        uint8_t ECG_POL             :1;      /*!< bit: 23 ECG Input Polarity Selection                          */
        } b;                                 /*!< Structure used for bit access                                 */
        uint8_t w[3];                        /*!< Type used for word access                                     */
    } REG_CNFG_EMUX;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint16_t _reserved_0_11     :12;      /*!< bit: 0-11  Reserved                                              */
        uint8_t ECG_DLPF            :2;       /*!< bit: 12-13 ECG Channel Digital Low-Pass Filter Cutoff Frequency  */
        uint8_t ECG_DHPF            :1;       /*!< bit: 14 ECG Channel Digital High-Pass Filter Cutoff Frequency    */
        uint8_t _reserved_15        :1;       /*!< bit: 15 Reserved                                                 */
        uint8_t ECG_GAIN            :2;       /*!< bit: 16-17 ECG Channel Gain Setting                              */
        uint8_t _reserved_18_21     :4;       /*!< bit: 18-21 Reserved                                              */
        uint8_t ECG_RATE            :2;       /*!< bit: 22-23 ECG Data Rate                                         */
        } b;                                 /*!< Structure used for bit access                                     */
        uint8_t w[3];                        /*!< Type used for word access                                         */
    } REG_CNFG_ECG;

    uint8_t _unused_16[3];      // Unused register

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t BMUX_FBIST          :2;      /*!< bit: 0-1  BioZ RMOD BIST Frequency Selection                      */
        uint8_t _reserved_2_3       :2;      /*!< bit: 2-3  Reserved                                                */
        uint8_t BMUX_RMOD           :3;      /*!< bit: 4-6  BioZ RMOD BIST Modulated Resistance Selection           */
        uint8_t _reserved_7         :1;      /*!< bit: 7  Reserved                                                  */
        uint8_t BMUX_RNOM           :3;      /*!< bit: 8-10  BioZ RMOD BIST Nominal Resistance Selection            */
        uint8_t BMUX_EN_BIST        :1;      /*!< bit: 11  BioZ Modulated Resistance Built-In-Self-Test Mode Enable */
        uint8_t BMUX_CG_MODE        :2;      /*!< bit: 12-13  BioZ Current Generator Mode Selection                 */
        uint8_t _reserved_14_15     :2;      /*!< bit: 14-15  Reserved                                              */
        uint8_t BMUX_CALN_SEL       :2;      /*!< bit: 16-17  BIN Calibration Selection                             */
        uint8_t BMUX_CALP_SEL       :2;      /*!< bit: 18-19  BIP Calibration Selection                             */
        uint8_t BMUX_OPENN          :1;      /*!< bit: 20  Open the BIN Input Switch                                */
        uint8_t BMUX_OPENP          :1;      /*!< bit: 21  Open the BIP Input Switch                                */
        uint8_t _reserved_22_23     :2;      /*!< bit: 22-23  Reserved                                              */
        } b;                                 /*!< Structure used for bit access                                     */
        uint8_t w[3];                        /*!< Type used for word access                                         */
    } REG_CNFG_BMUX;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t BIOZ_PHOFF          :4;      /*!< bit: 0-3  BioZ Current Generator Modulation Phase Offset          */
        uint8_t BIOZ_CGMAG          :3;      /*!< bit: 4-6  BioZ Current Generator Magnitude                        */
        uint8_t BIOZ_CGMON          :1;      /*!< bit: 7  BioZ Current Generator Monitor                            */
        uint8_t BIOZ_FCGEN          :4;      /*!< bit: 8-11  BioZ Current Generator Modulation Frequency            */
        uint8_t BIOZ_DLPF           :2;      /*!< bit: 12-13  BioZ Channel Digital Low-Pass Filter Cutoff Frequency */
        uint8_t BIOZ_DHPF           :2;      /*!< bit: 14-15  BioZ Channel Digital High-Pass Filter Cutoff Frequency*/
        uint8_t BIOZ_GAIN           :2;      /*!< bit: 16-17  BioZ Channel Gain Setting                             */
        uint8_t LN_BIOZ             :1;      /*!< bit: 18  BioZ Channel Instrumentation Amplifier (INA) Power Mode  */
        uint8_t EXT_RBIAS           :1;      /*!< bit: 19  External Resistor Bias Enable                            */
        uint8_t BIOZ_AHPF           :3;      /*!< bit: 20-22  BioZ/PACE Channel Analog High-Pass Filter Cutoff Frequency and Bypass */
        uint8_t BIOZ_RATE           :1;      /*!< bit: 23  BioZ Data Rate                                           */
        } b;                                 /*!< Structure used for bit access                                     */
        uint8_t w[3];                        /*!< Type used for word access                                         */
    } REG_CNFG_BIOZ;

    uint8_t _unused_19[3];      // Unused register

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t PACE_DACN           :4;      /*!< bit: 0-3  PACE Detector Negative Comparator Threshold             */
        uint8_t PACE_DACP           :4;      /*!< bit: 4-7  PACE Detector Positive Comparator Threshold             */
        uint8_t _reserved_8_11      :4;      /*!< bit: 8-11  Reserved                                               */
        uint8_t AOUT                :2;      /*!< bit: 12-13  PACE Single Ended Analog Output Buffer Signal Monitoring Selection   */
        uint8_t AOUT_LBW            :1;      /*!< bit: 14  PACE Analog Output Buffer Bandwidth Mode                 */
        uint8_t _reserved_15        :1;      /*!< bit: 15  Reserved                                                 */
        uint8_t PACE_GAIN           :3;      /*!< bit: 16-18  PACE Channel Gain Setting                             */
        uint8_t DIFF_OFF            :1;      /*!< bit: 19  PACE Differentiator (Derivative) Mode                    */
        uint8_t _reserved_20_22     :3;      /*!< bit: 20-22  Reserved                                              */  
        uint8_t PACE_POL            :1;      /*!< bit: 23  PACE Input Polarity Selection                            */
        } b;                                 /*!< Structure used for bit access                                     */
        uint8_t w[3];                        /*!< Type used for word access                                         */
    } REG_CNFG_PACE;

    uint8_t _unused_1B[3];      // Unused register
    uint8_t _unused_1C[3];      // Unused register

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t _reserved_0_7       :8;      /*!< bit: 0-7  Reserved                                                */
        uint8_t PTSF                :4;      /*!< bit: 8-11  R-to-R Peak Threshold Scaling Factor                   */
        uint8_t PAVG                :2;      /*!< bit: 12-13  R-to-R Average Mode Selection                         */
        uint8_t _reserved_14        :1;      /*!< bit: 14  Reserved                                                 */
        uint8_t EN_RTOR             :1;      /*!< bit: 15  ECG R-to-R Detection Enable                              */
        uint8_t RGAIN               :4;      /*!< bit: 16-19  R-to-R Gain Setting                                   */
        uint8_t WNDW                :4;      /*!< bit: 20-23  Width of the averaging window                         */
        } b;                                 /*!< Structure used for bit access                                     */
        uint8_t w[3];                        /*!< Type used for word access                                         */
    } REG_CNFG_RTOR1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t _reserved_0_7       :8;      /*!< bit: 0-7  Reserved                                                */
        uint8_t RHSF                :3;      /*!< bit: 8-10  R-to-R Interval Hold Off Scaling Factor                */
        uint8_t _reserved_11        :1;      /*!< bit: 11  Reserved                                                 */
        uint8_t RAVG                :2;      /*!< bit: 12-13  R-to-R Interval Averaging Weight Factor               */
        uint8_t _reserved_14_15     :2;      /*!< bit: 14-15  Reserved                                              */
        uint8_t HOFF                :6;      /*!< bit: 16-21  R-to-R Minimum Hold Off                               */
        uint8_t _reserved_22_23     :2;      /*!< bit: 22-23  Reserved                                              */
        } b;                                 /*!< Structure used for bit access                                     */
        uint8_t w[3];                        /*!< Type used for word access                                         */
    } REG_CNFG_RTOR2;

    uint8_t _unused_1F[3];      // Unused register

    uint8_t REG_ECG_FIFO_BURST[3];  // ECG FIFO burst register
    uint8_t REG_ECG_FIFO[3];        // ECG FIFO register
    uint8_t REG_BIOZ_FIFO_BURST[3]; // BioZ FIFO burst register
    uint8_t REG_BIOZ_FIFO[3];       // BioZ FIFO register

    uint8_t _unused_24[3];      // Unused register

    uint8_t REG_RTOR[3];        // R-to-R register

    uint8_t _unused_26[3];      // Unused register
    uint8_t _unused_27[3];      // Unused register
    uint8_t _unused_28[3];      // Unused register
    uint8_t _unused_29[3];      // Unused register
    uint8_t _unused_2A[3];      // Unused register
    uint8_t _unused_2B[3];      // Unused register
    uint8_t _unused_2C[3];      // Unused register
    uint8_t _unused_2D[3];      // Unused register
    uint8_t _unused_2E[3];      // Unused register
    uint8_t _unused_2F[3];      // Unused register

    uint8_t REG_PACE0_BURST[3];  // Pace 0 burst register
    uint8_t REG_PACE0_A[3];      // Pace 0 register A
    uint8_t REG_PACE0_B[3];      // Pace 0 register B
    uint8_t REG_PACE0_C[3];      // Pace 0 register C
    uint8_t REG_PACE1_BURST[3];  // Pace 1 burst register
    uint8_t REG_PACE1_A[3];      // Pace 1 register A
    uint8_t REG_PACE1_B[3];      // Pace 1 register B
    uint8_t REG_PACE1_C[3];      // Pace 1 register C
    uint8_t REG_PACE2_BURST[3];  // Pace 2 burst register
    uint8_t REG_PACE2_A[3];      // Pace 2 register A
    uint8_t REG_PACE2_B[3];      // Pace 2 register B
    uint8_t REG_PACE2_C[3];      // Pace 2 register C
    uint8_t REG_PACE3_BURST[3];  // Pace 3 burst register
    uint8_t REG_PACE3_A[3];      // Pace 3 register A
    uint8_t REG_PACE3_B[3];      // Pace 3 register B
    uint8_t REG_PACE3_C[3];      // Pace 3 register C
    uint8_t REG_PACE4_BURST[3];  // Pace 4 burst register
    uint8_t REG_PACE4_A[3];      // Pace 4 register A
    uint8_t REG_PACE4_B[3];      // Pace 4 register B
    uint8_t REG_PACE4_C[3];      // Pace 4 register C
    uint8_t REG_PACE5_BURST[3];  // Pace 5 burst register
    uint8_t REG_PACE5_A[3];      // Pace 5 register A
    uint8_t REG_PACE5_B[3];      // Pace 5 register B
    uint8_t REG_PACE5_C[3];      // Pace 5 register C

}__attribute__((packed)) MAX30001_REG;

/*! Foramt of the data to be stored in the flash */
typedef struct {
    int16_t header;
    int32_t rawtime_bin;
    int16_t time_ms_bin;
    uint8_t len;
    uint8_t index;
    uint8_t measurements[6];
} __attribute__((packed)) MAX30001_StorageFormat;

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/

#ifndef _MAX30001_C_SRC



#endif  /* _MAX30001_C_SRC */

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

/**
 * @brief  This function initializes the MAX30001 spi instance
 * 
 * @param  void
 * 
 * @return 0 if successful, -1 if failed, -2 if not present 
 */
int MAX30001_init(void);

/**
 * @brief  This function writes a value to a MAX30001 register
 * 
 * @param  addr : register address
 * @param  data : pointer to data to write
 * 
 * @return 0 if successful, -1 otherwise 
 */
int MAX30001_normal_read(uint8_t addr, uint8_t *data);

/**
 * @brief  This function reads a value from a MAX30001 register
 * 
 * @param  addr : register address
 * @param  data : pointer to data register
 * 
 * @return 0 if successful, -1 otherwise 
 */
int MAX30001_normal_write(uint8_t addr, uint8_t *data);

/**
 * @brief  This function reads a burst of values from a MAX30001 register
 * 
 * @param  addr : register address
 * @param  data : pointer to data register
 * @param  len : length of data to read
 * 
 * @return 0 if successful, -1 otherwise 
 */
int MAX30001_burst_read(uint8_t addr, uint8_t *data, uint8_t len);

/**
 * @brief  This function start the continuous reading thread
 * 
 * @param  void
 * 
 * @return 0 if successful, -1 otherwise 
 */
void MAX30001_start_thread(void);

/**
 * @brief  This function stop the continuous reading thread
 * 
 * @param  void
 * 
 * @return 0 if successful, -1 otherwise 
 */
void MAX30001_stop_thread(void);

/**
 * @brief  This function configures the MAX30001 registers
 * 
 * @param  void
 * 
 * @return 0 if successful, -1 otherwise 
 */
void MAX30001_config(void);

/**
 * @brief  This function checks if the MAX30001 is present
 * 
 * @param  void
 * 
 * @return 0 if successful, -1 otherwise 
 */
int MAX30001_check_present(void);

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
