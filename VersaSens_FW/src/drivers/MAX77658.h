/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : MAX77658.h                                                   **
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
* @file   MAX77658.h
* @date   DD/MM/YY
* @brief  This is the main header of MAX77658.c
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _MAX77658_H
#define _MAX77658_H

/****************************************************************************/
/**                                                                        **/
/**                            MODULES USED                                **/
/**                                                                        **/
/****************************************************************************/

// #include "sdk_common.h"
#include <zephyr/types.h>
#include "twim_inst.h"

/****************************************************************************/
/**                                                                        **/
/**                       DEFINITIONS AND MACROS                           **/
/**                                                                        **/
/****************************************************************************/

/*!Device Address*/
#define PMU_MAIN_ADDR                0x48   
#define PMU_FUEL_GAUGE_ADDR          0x36

/*!I2C Configuration*/
#define MAX_SIZE_TRANSFER           20  

/*!MAX77658 TWI pins*/
#define MAX77658_SCL_PIN            47
#define MAX77658_SDA_PIN            33

/*!Registers addresses*/

/*!Global registers*/
#define REG_INT_GLBL0_ADDR          0x00
#define REG_INT_GLBL1_ADDR          0x04
#define REG_ERCFLAG_ADDR            0x05
#define REG_STAT_GLBL_ADDR          0x06
#define REG_INTM_GLBL0_ADDR         0x08
#define REG_INTM_GLBL1_ADDR         0x09
#define REG_CNFG_GLBL_ADDR          0x10
#define REG_CNFG_GPIO0_ADDR         0x11
#define REG_CNFG_GPIO1_ADDR         0x12
#define REG_CNFG_GPIO2_ADDR         0x13
#define REG_CID_ADDR                0x14
#define REG_CNFG_WDT_ADDR           0x17

/*!Charger registers*/
#define REG_INT_CHG_ADDR            0x01
#define REG_STAT_CHG_A_ADDR         0x02
#define REG_STAT_CHG_B_ADDR         0x03
#define REG_INT_M_CHG_ADDR          0x07
#define REG_CNFG_CHG_A_ADDR         0x20
#define REG_CNFG_CHG_B_ADDR         0x21
#define REG_CNFG_CHG_C_ADDR         0x22
#define REG_CNFG_CHG_D_ADDR         0x23
#define REG_CNFG_CHG_E_ADDR         0x24
#define REG_CNFG_CHG_F_ADDR         0x25
#define REG_CNFG_CHG_G_ADDR         0x26
#define REG_CNFG_CHG_H_ADDR         0x27
#define REG_CNFG_CHG_I_ADDR         0x28

/*!SBB registers*/
#define REG_CNFG_SBB_TOP_ADDR       0x38
#define REG_CNFG_SBB0_A_ADDR        0x39
#define REG_CNFG_SBB0_B_ADDR        0x3A
#define REG_CNFG_SBB1_A_ADDR        0x3B
#define REG_CNFG_SBB1_B_ADDR        0x3C
#define REG_CNFG_SBB2_A_ADDR        0x3D
#define REG_CNFG_SBB2_B_ADDR        0x3E
#define REG_CNFG_DVS_SBB0_A_ADDR    0x3F

/*!LDO registers*/
#define REG_CNFG_LDO0_A_ADDR        0x48
#define REG_CNFG_LDO0_B_ADDR        0x49
#define REG_CNFG_LDO1_A_ADDR        0x4A
#define REG_CNFG_LDO1_B_ADDR        0x4B


/*!Fuel Gauge registers*/
#define REG_Status_ADDR             0x00
#define REG_VAlrtTh_ADDR            0x01
#define REG_TAlrtTh_ADDR            0x02
#define REG_SAlrtTh_ADDR            0x03
#define REG_FullSOCThr_ADDR         0x13
#define REG_DesignCap_ADDR          0x18
#define REG_Config_ADDR             0x1D
#define REG_IChgTerm_ADDR           0x1E
#define REG_DevName_ADDR            0x21
#define REG_LearnCfg_ADDR           0x28
#define REG_FilterCfg_ADDR          0x29
#define REG_VEmpty_ADDR             0x3A
#define REG_Power_ADDR              0xB1
#define REG_AvgPower_ADDR           0xB3
#define REG_IAlrtTh_ADDR            0xB4
#define REG_Config2_ADDR            0xBB

#define REG_Temp_ADDR               0x08
#define REG_Vcell_ADDR              0x09
#define REG_Current_ADDR            0x0A
#define REG_AvgCurrent_ADDR         0x0B
#define REG_AvgTA_ADDR              0x16
#define REG_AvgVCell_ADDR           0x19
#define REG_MaxMinTemp_ADDR         0x1A
#define REG_MaxMinVolt_ADDR         0x1B
#define REG_MaxMinCurr_ADDR         0x1C
#define REG_AIN0_ADDR               0x27
#define REG_Timer_ADDR              0x3E
#define REG_TimerH_ADDR             0xBE

#define REG_RepCap_ADDR             0x05
#define REG_RepSOC_ADDR             0x06
#define REG_AvSOC_ADDR              0x0E
#define REG_FullCapRep_ADDR         0x10
#define REG_TTE_ADDR                0x11
#define REG_RCell_ADDR              0x14
#define REG_Cycles_ADDR             0x17
#define REG_AvCap_ADDR              0x1F
#define REG_TTF_ADDR                0x20

/****************************************************************************/
/**                                                                        **/
/**                       TYPEDEFS AND STRUCTURES                          **/
/**                                                                        **/
/****************************************************************************/

/*!MAX77658 Registers Stucture*/
typedef struct {  

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {                                                                                      
        uint8_t GPI0_F              :1;     /*!< bit: 0  GPI Falling Interrupt                                 */
        uint8_t GPI0_R              :1;     /*!< bit: 1  GPI Rising Interrupt                                  */
        uint8_t nEN_F               :1;     /*!< bit: 2  nEN Falling Interrupt                                 */
        uint8_t nEN_R               :1;     /*!< bit: 3  nEN Rising Interrupt                                  */
        uint8_t TJAL1_R             :1;     /*!< bit: 4  Thermal Alarm 1 Rising Interrupt                      */
        uint8_t TJAL2_R             :1;     /*!< bit: 5  Thermal Alarm 2 Rising Interrupt                      */
        uint8_t DOD1_R              :1;     /*!< bit: 6  LDO Dropout Detector Rising Interrupt                 */
        uint8_t DOD2_R              :1;     /*!< bit: 7  LDO Dropout Detector Rising Interrupt                 */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_INT_GLBL0;

    union
    {                                                                                                    
        struct                                                                                             
        {                                                                                      
        uint8_t THM_I               :1;     /*!< bit: 0  Thermistor Related Interrupt                          */
        uint8_t CHG_I               :1;     /*!< bit: 1  Charger Related Interrupt                             */
        uint8_t CHGIN_I             :1;     /*!< bit: 2  CHGIN Related Interrupt                               */
        uint8_t TJ_REG_I            :1;     /*!< bit: 3  Junction Temperature RegulationInterrupt              */
        uint8_t CHGIN_CTRL_I        :1;     /*!< bit: 4  CHGIN Control-Loop Related Interrupt                  */
        uint8_t SYS_CTRL_I          :1;     /*!< bit: 5  Minimum System Voltage Regulation-Loop Related Interrupt   */
        uint8_t SYS_CNFG_I          :1;     /*!< bit: 6  System Voltage Configuration Error Interrupt          */
        uint8_t _reserved_7         :1;     /*!< bit: 7  Reserved                                              */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_INT_CHG;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {                                                                                      
        uint8_t THM_DTLS            :3;     /*!< bit: 0-2  Battery Temperature Details                         */
        uint8_t TJ_REG_STAT         :1;     /*!< bit: 3  Maximum Junction Temperature Regulation Loop Status   */
        uint8_t VSYS_MIN_STAT       :1;     /*!< bit: 4  Minimum System Voltage Regulation Loop Status         */
        uint8_t ICHGIN_LIM_STAT     :1;     /*!< bit: 5  Input Current Limit Loop Status                       */
        uint8_t VCHGIN_MIN_STAT     :1;     /*!< bit: 6  Minimum Input Voltage Regulation Loop Status          */
        uint8_t _reserved_7         :1;     /*!< bit: 7  Reserved                                              */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_STAT_CHG_A;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {                                                                                      
        uint8_t TIME_SUS            :1;     /*!< bit: 0  Time Suspended Indicator                              */
        uint8_t CHG                 :1;     /*!< bit: 1  Quick Charger Status                                  */
        uint8_t CHGIN_DTLS          :2;     /*!< bit: 2-3  CHGIN Status Details                                */
        uint8_t CHG_DTLS            :4;     /*!< bit: 4-5  Charger Details                                     */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_STAT_CHG_B;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {                                                                                      
        uint8_t GPI1_F              :1;     /*!< bit: 0  GPI Falling Interrupt                                 */
        uint8_t GPI1_R              :1;     /*!< bit: 1  GPI Rising Interrupt                                  */
        uint8_t SBB0_F              :1;     /*!< bit: 2  SBB0 Fault Indicator                                  */
        uint8_t SBB1_F              :1;     /*!< bit: 3  SBB1 Fault Indicator                                  */
        uint8_t SBB2_F              :1;     /*!< bit: 4  SBB2 Fault Indicator                                  */
        uint8_t LDO0_F              :1;     /*!< bit: 5  LDO0 Fault Interrupt                                  */
        uint8_t LDO1_F              :1;     /*!< bit: 6  LDO1 Fault Interrupt                                  */
        uint8_t _reserved_7         :1;     /*!< bit: 7  Reserved                                              */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_INT_GLBL1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {                                                                                      
        uint8_t TOVLD               :1;     /*!< bit: 0  Thermal Overload                                      */
        uint8_t SYSOVLO             :1;     /*!< bit: 1  SYS Domain Overvoltage Lockout                        */
        uint8_t SYSUVLO             :1;     /*!< bit: 2  SYS Domain Undervoltage Lockout                       */
        uint8_t MRST_F              :1;     /*!< bit: 3  Manual Reset Timer                                    */
        uint8_t SFT_OFF_F           :1;     /*!< bit: 4  Software OFF Flag                                     */
        uint8_t SFT_CRST_F          :1;     /*!< bit: 5  Software Cold Reset Flag                              */
        uint8_t EVT_WDT_SFT_23OR    :1;     /*!< bit: 6  Watchdog Timer Expired Flag                           */
        uint8_t SBB_FAULT           :1;     /*!< bit: 7  SBB Fault Shutdown Flag                               */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_ERCFLAG;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {                                                                                      
        uint8_t STAT_IRQ            :1;     /*!< bit: 0  Software Version of the nIRQ MOSFET GateDrive         */
        uint8_t STAT_EN             :1;     /*!< bit: 1  Debounced Status for the nEN Input                    */
        uint8_t TJAL1_S             :1;     /*!< bit: 2  Thermal Alarm 1 Status                                */
        uint8_t TJAL2_S             :1;     /*!< bit: 3  Thermal Alarm 2 Status                                */
        uint8_t DOD1_S              :1;     /*!< bit: 4  LDO1 Dropout Detector Rising Status                   */
        uint8_t DOD2_S              :1;     /*!< bit: 5  LDO2 Dropout Detector Rising Status                   */
        uint8_t BOK                 :1;     /*!< bit: 6  BOK Interrupt Status                                  */
        uint8_t DIDM                :1;     /*!< bit: 7  Device Identification Bits for Metal Options          */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_STAT_GLBL;

    union
    {                                                                                                    
        struct                                                                                             
        {                                                                                      
        uint8_t THM_M               :1;     /*!< bit: 0  Prevents the THM_I bit from causing hardware IRQs     */
        uint8_t CHG_M               :1;     /*!< bit: 1  Prevents the CHG_I bit from causing hardware IRQs     */
        uint8_t CHGIN_M             :1;     /*!< bit: 2  Prevents the CHGIN_I bit from causing hardware IRQs   */
        uint8_t TJ_REG_M            :1;     /*!< bit: 3  Prevents the TJ_REG_I bit from causing hardware IRQs  */
        uint8_t CHGIN_CTRL_M        :1;     /*!< bit: 4  Prevents the CHGIN_CTRL_I bit from causing hardware IRQs   */
        uint8_t SYS_CTRL_M          :1;     /*!< bit: 5  Prevents the SYS_CTRL_I bit from causing hardware IRQs     */
        uint8_t SYS_CNFG_M          :1;     /*!< bit: 6  Prevents the SYS_CNFG_I bit from causing hardware IRQs     */
        uint8_t DIS_AICL            :1;     /*!< bit: 7  Active Input Current Loop Disable                     */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_INT_M_CHG;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {                                                                                      
        uint8_t GPI0_FM             :1;     /*!< bit: 0  GPI Falling Interrupt Mask                            */
        uint8_t GPI0_RM             :1;     /*!< bit: 1  GPI Rising Interrupt Mask                             */
        uint8_t nEN_FM              :1;     /*!< bit: 2  nEN Falling Interrupt Mask                            */
        uint8_t nEN_RM              :1;     /*!< bit: 3  nEN Rising Interrupt Mask                             */
        uint8_t TJAL1_RM            :1;     /*!< bit: 4  Thermal Alarm 1 Rising Interrupt Mask                 */
        uint8_t TJAL2_RM            :1;     /*!< bit: 5  Thermal Alarm 2 Rising Interrupt Mask                 */
        uint8_t DOD1_RM             :1;     /*!< bit: 6  LDO Dropout Detector Rising Interrupt Mask            */
        uint8_t DOD2_RM             :1;     /*!< bit: 7  LDO Dropout Detector Rising Interrupt Mask            */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_INTM_GLBL0;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {                                                                                      
        uint8_t GPI1_FM             :1;     /*!< bit: 0  GPI Falling Interrupt Mask                            */
        uint8_t GPI1_RM             :1;     /*!< bit: 1  GPI Rising Interrupt Mask                             */
        uint8_t SBB0_FM             :1;     /*!< bit: 2  SBB0 Fault Indicator Mask                             */
        uint8_t SBB1_FM             :1;     /*!< bit: 3  SBB1 Fault Indicator Mask                             */
        uint8_t SBB2_FM             :1;     /*!< bit: 4  SBB2 Fault Indicator Mask                             */
        uint8_t LDO0_FM             :1;     /*!< bit: 5  LDO0 Fault Interrupt Mask                             */
        uint8_t LDO1_FM             :1;     /*!< bit: 6  LDO1 Fault Interrupt Mask                             */
        uint8_t _reserved_7         :1;     /*!< bit: 7  Reserved                                              */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_INTM_GLBL1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {                                                                                      
        uint8_t SFT_CTRL            :2;     /*!< bit: 0-1  Software Reset Functions                            */
        uint8_t DBEN_nEN            :1;     /*!< bit: 2  Debounce Timer Enable for the nEN Pin                 */
        uint8_t nEN_MODE            :2;     /*!< bit: 3  nEN Input (ON-KEY) Default Configuration Mode         */
        uint8_t SBIA_LPM            :1;     /*!< bit: 5  Main Bias Low-Power Mode Software Request             */
        uint8_t T_MRST              :1;     /*!< bit: 6  Sets the Manual Reset Time                            */
        uint8_t PU_DIS              :1;     /*!< bit: 7  nEN Internal Pullup Resistor                          */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_GLBL;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t DIR                 :1;     /*!< bit: 0  GPIO Direction for GPIO0                              */                                                                                      
        uint8_t DI                  :1;     /*!< bit: 1  GPIO Digital Input Value for GPI0                     */
        uint8_t DRV                 :1;     /*!< bit: 2  General Purpose Output Driver Type for GPO0           */
        uint8_t DO                  :1;     /*!< bit: 3  GPIO Digital Output Value for GPO0                    */
        uint8_t DBEN_GPI            :1;     /*!< bit: 4  General Purpose Input Debounce Timer Enable for GPI0  */
        uint8_t ALT_GPIO0           :1;     /*!< bit: 5  Alternate Mode Enable for GPIO0                       */
        uint8_t _reserved_6         :1;     /*!< bit: 6  Reserved                                              */
        uint8_t SBB_F_SHUTDN        :1;     /*!< bit: 7  SBB Shutdown from SBB Fault                           */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_GPIO0;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t DIR                 :1;     /*!< bit: 0  GPIO Direction for GPIO1                              */                                                                                      
        uint8_t DI                  :1;     /*!< bit: 1  GPIO Digital Input Value for GPI1                     */
        uint8_t DRV                 :1;     /*!< bit: 2  General Purpose Output Driver Type for GPO1           */
        uint8_t DO                  :1;     /*!< bit: 3  GPIO Digital Output Value for GPO1                    */
        uint8_t DBEN_GPI            :1;     /*!< bit: 4  General Purpose Input Debounce Timer Enable for GPI1  */
        uint8_t ALT_GPIO1           :1;     /*!< bit: 5  Alternate Mode Enable for GPIO1                       */
        uint8_t _reserved_6_7       :2;     /*!< bit: 6-7  Reserved                                            */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_GPIO1;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t DIR                 :1;     /*!< bit: 0  GPIO Direction for GPIO2                              */                                                                                      
        uint8_t DI                  :1;     /*!< bit: 1  GPIO Digital Input Value for GPI2                     */
        uint8_t DRV                 :1;     /*!< bit: 2  General Purpose Output Driver Type for GPO2           */
        uint8_t DO                  :1;     /*!< bit: 3  GPIO Digital Output Value for GPO2                    */
        uint8_t DBEN_GPI            :1;     /*!< bit: 4  General Purpose Input Debounce Timer Enable for GPI2  */
        uint8_t ALT_GPIO2           :1;     /*!< bit: 5  Alternate Mode Enable for GPIO2                       */
        uint8_t _reserved_6_7       :2;     /*!< bit: 6-7  Reserved                                            */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_GPIO2;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t CID                 :4;     /*!< bit: 0-3  Chip ID                                             */  
        uint8_t _reserved_4_7       :4;     /*!< bit: 4-7  Reserved                                            */                                                                                    
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CID;

    uint8_t _unused_15_16[2];              /*!< Reserved                                                        */

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t WDT_LOCK            :1;     /*!< bit: 0  Factory-Set Safety Bit for the Watchdog Timer         */
        uint8_t WDT_EN              :1;     /*!< bit: 1  Watchdog Timer Enable                                 */
        uint8_t WDT_CLR             :1;     /*!< bit: 2  Watchdog Timer Clear                                  */
        uint8_t WDT_MODE            :1;     /*!< bit: 3  Watchdog Timer Expired Action                         */
        uint8_t WDT_PER             :2;     /*!< bit: 4-5  Watchdog Timer Period                               */     
        uint8_t _reserved_6_7       :2;     /*!< bit: 6-7  Reserved                                            */      
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_WDT;

    uint8_t _unused_18_1F[8];              /*!< Reserved                                                        */

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t THM_COLD            :2;     /*!< bit: 0-1  Sets the TCOLD JEITA Temperature Threshold          */
        uint8_t THM_COOL            :2;     /*!< bit: 2-3  Sets the TCOOL JEITA Temperature Threshold          */
        uint8_t THM_WARM            :2;     /*!< bit: 4-5  Sets the TWARM JEITA Temperature Threshold          */
        uint8_t THM_HOT             :2;     /*!< bit: 6-7  Sets the THOT JEITA Temperature Threshold           */                                                                                      
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_CHG_A;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t CHG_EN              :1;     /*!< bit: 0  Charger Enable                                        */ 
        uint8_t I_PQ                :1;     /*!< bit: 1  Sets the prequalification charge current              */   
        uint8_t ICHGIN_LIM          :3;     /*!< bit: 2-4  CHGIN Input Current Limit                           */  
        uint8_t VCHGIN_MIN          :3;     /*!< bit: 5-7  CHGIN Minimum Regulation Voltage                    */                                                                     
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_CHG_B;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t T_TOPOFF            :3;     /*!< bit: 0-2  Top-Off Timer Value                                 */  
        uint8_t I_TERM              :2;     /*!< bit: 3-4  Charger Termination Current                         */  
        uint8_t CHG_PQ              :3;     /*!< bit: 5-7  Battery Prequalification Voltage Threshold          */                                                                                                                                                                                                     
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_CHG_C;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t VSYS_REG            :5;     /*!< bit: 0-4  System Voltage Regulation                           */
        uint8_t TJ_REG              :3;     /*!< bit: 5-7  Sets the die junction temperature regulation point  */                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_CHG_D;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t T_FAST_CHG          :2;     /*!< bit: 0-1  Sets the fast-charge safety timer                   */ 
        uint8_t CHG_CC              :6;     /*!< bit: 2-7  Sets the fast-charge constant current value         */                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_CHG_E;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t _reserved_0         :1;     /*!< bit: 0  Reserved                                              */
        uint8_t _reserved_1         :1;     /*!< bit: 1  Reserved                                              */
        uint8_t CHG_CC_JEITA        :6;     /*!< bit: 2-7  Sets the modified IFAST-CHG-JEITA                   */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_CHG_F;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t FUS_M               :1;     /*!< bit: 0  Forced USB Suspend Mask                               */
        uint8_t USBS                :1;     /*!< bit: 1  USB Suspend Mode                                      */
        uint8_t CHG_CV              :6;     /*!< bit: 2-7  Sets fast-charge battery regulation voltage         */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_CHG_G;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t CHR_TH_EN           :1;     /*!< bit: 0  Charger Restart Threshold Enable                      */
        uint8_t SYS_BAT_PRT         :1;     /*!< bit: 1  System Battery Protection                             */
        uint8_t CHG_CV_JEITA        :6;     /*!< bit: 2-7  Sets the modified VFAST-CHG-JEITA                   */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_CHG_H;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t MUX_SEL             :4;     /*!< bit: 0-3  Selects the analog channel to connect to AMUX       */
        uint8_t IMON_DISCHG_SCALE   :4;     /*!< bit: 4-7  Selects the battery discharge current fullscale current value */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_CHG_I;

    uint8_t _unused_29_37[14];             /*!< Reserved                                                        */

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t DRV_SBB             :2;     /*!< bit: 0-1  SIMO Buck-Boost (all channels) Drive Strength Trim  */
        uint8_t _reserved_2_5       :4;     /*!< bit: 2-5  Reserved                                            */
        uint8_t IPK_1P5A            :1;     /*!< bit: 6  SBB2 Inductor Current Limit Offset                    */
        uint8_t DIS_LPM             :1;     /*!< bit: 7  Disables the automatic low-power mode for each SIMO channel */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_SBB_TOP;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t TV_SBB0             :8;     /*!< bit: 0-7  SIMO Buck-Boost Channel 0 Target Output Voltage     */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_SBB0_A;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t EN_SBB0             :3;     /*!< bit: 0-2  Enable Control for SIMO Buck-Boost Channel 0        */
        uint8_t ADE_SBB0            :1;     /*!< bit: 3  SIMO Buck-Boost Channel 0 Active-Discharge Enable     */
        uint8_t IP_SBB0             :2;     /*!< bit: 4-5  SIMO Buck-Boost Channel 0 Peak Current Limit        */
        uint8_t OP_MODE             :2;     /*!< bit: 6-7  Operation Mode of SBB0                              */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_SBB0_B;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t TV_SBB1             :8;     /*!< bit: 0-7  SIMO Buck-Boost Channel 1 Target Output Voltage     */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_SBB1_A;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t EN_SBB1             :3;     /*!< bit: 0-2  Enable Control for SIMO Buck-Boost Channel 1        */
        uint8_t ADE_SBB1            :1;     /*!< bit: 3  SIMO Buck-Boost Channel 1 Active-Discharge Enable     */
        uint8_t IP_SBB1             :2;     /*!< bit: 4-5  SIMO Buck-Boost Channel 1 Peak Current Limit        */
        uint8_t OP_MODE             :2;     /*!< bit: 6-7  Operation Mode of SBB1                              */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_SBB1_B;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t TV_SBB2             :8;     /*!< bit: 0-7  SIMO Buck-Boost Channel 2 Target Output Voltage     */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_SBB2_A;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t EN_SBB2             :3;     /*!< bit: 0-2  Enable Control for SIMO Buck-Boost Channel 2        */
        uint8_t ADE_SBB2            :1;     /*!< bit: 3  SIMO Buck-Boost Channel 2 Active-Discharge Enable     */
        uint8_t IP_SBB2             :2;     /*!< bit: 4-5  SIMO Buck-Boost Channel 2 Peak Current Limit        */
        uint8_t OP_MODE             :2;     /*!< bit: 6-7  Operation Mode of SBB2                              */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_SBB2_B;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t TV_SBB0_DVS        :8;      /*!< bit: 0-7  SIMO Buck-Boost Channel 0 Target Output Voltage     */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_DVS_SBB0_A;

    uint8_t _unused_40_47[8];              /*!< Reserved                                                        */

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t TV_LDO0             :7;     /*!< bit: 0-6  LDO0 Target Output Voltage                          */
        uint8_t TV_OFS_LDO0         :1;     /*!< bit: 7  LDO0 Output Voltage Offset                            */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_LDO0_A;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t EN_LDO0             :3;     /*!< bit: 0-2  Enable Control for LDO0                             */
        uint8_t ADE_LDO0            :1;     /*!< bit: 3  LDO0 Active-Discharge Enable                          */
        uint8_t LDO0_MD             :1;     /*!< bit: 4  Operation Mode of LDO0                                */
        uint8_t _reserved_5_7       :3;     /*!< bit: 5-7  Reserved                                            */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_LDO0_B;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t TV_LDO1             :7;     /*!< bit: 0-6  LDO1 Target Output Voltage                          */
        uint8_t TV_OFS_LDO1         :1;     /*!< bit: 7  LDO1 Output Voltage Offset                            */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_LDO1_A;

    union                                                                                                
    {                                                                                                    
        struct                                                                                             
        {
        uint8_t EN_LDO1             :3;     /*!< bit: 0-2  Enable Control for LDO1                             */
        uint8_t ADE_LDO1            :1;     /*!< bit: 3  LDO1 Active-Discharge Enable                          */
        uint8_t LDO1_MD             :1;     /*!< bit: 4  Operation Mode of LDO1                                */
        uint8_t _reserved_5_7       :3;     /*!< bit: 5-7  Reserved                                            */
        } b;                                /*!< Structure used for bit access                                 */
        uint8_t w;                          /*!< Type used for word access                                     */
    } REG_CNFG_LDO1_B;

}__attribute__((packed)) MAX77658_REG;


/*!MAX77658 Fuel Gauge Registers Stucture*/
typedef struct {  

    union 
    {
        struct
        {
        uint8_t _reserved_0         :1;     /*!< bit: 0  Reserved                                             */
        uint8_t POR                 :1;     /*!< bit: 1  Power-On Reset                                       */
        uint8_t Imn                 :1;     /*!< bit: 2  Minimum Current-Alert Threshold Exceeded             */
        uint8_t Bst                 :1;     /*!< bit: 3  Battery Status                                       */
        uint8_t _reserved_4_5       :2;     /*!< bit: 4-5  Reserved                                           */
        uint8_t Imx                 :1;     /*!< bit: 6  Maximum Current-Alert Threshold Exceeded             */
        uint8_t dSOCi               :1;     /*!< bit: 7  1% SOC Change Alert                                  */
        uint8_t Vmn                 :1;     /*!< bit: 8  Minimum VALRT Threshold Exceeded                     */
        uint8_t Tmn                 :1;     /*!< bit: 9  Minimum TALRT Threshold Exceeded                     */
        uint8_t Smn                 :1;     /*!< bit: 10  Minimum SOCALRT Threshold Exceeded                  */
        uint8_t Bi                  :1;     /*!< bit: 11  Battery Insertion                                   */
        uint8_t Vmx                 :1;     /*!< bit: 12  Maximum VALRT Threshold Exceeded                    */
        uint8_t Tmx                 :1;     /*!< bit: 13  Maximum TALRT Threshold Exceeded                    */
        uint8_t Smx                 :1;     /*!< bit: 14  Maximum SOCALRT Threshold Exceeded                  */
        uint8_t Br                  :1;     /*!< bit: 15  Battery Removal                                     */
        } b;                                /*!< Structure used for bit access                                */
        uint16_t w;                         /*!< Type used for word access                                    */
    } REG_Status;

    union
    {
        struct
        {
        uint8_t VMIN                :8;     /*!< bit: 0-7  Minimum Voltage Reading                            */
        uint8_t VMAX                :8;     /*!< bit: 8-15  Maximum Voltage Reading                           */
        } b;                                /*!< Structure used for bit access                                */
        uint16_t w;                         /*!< Type used for word access                                    */
    } REG_VAlrtTh;

    union
    {
        struct
        {
        uint8_t TMIN                :8;     /*!< bit: 0-7  Sets an alert threshold for minimum temperature    */
        uint8_t TMAX                :8;     /*!< bit: 8-15  Sets an alert threshold for maximum temperature   */
        } b;                                /*!< Structure used for bit access                                */
        uint16_t w;                         /*!< Type used for word access                                    */
    } REG_TAlrtTh;

    union
    {
        struct
        {
        uint8_t SMIN                :8;     /*!< bit: 0-7  Sets an alert for minimum SOC                      */
        uint8_t SMAX                :8;     /*!< bit: 8-15  Sets an alert for maximum SOC                     */
        } b;                                /*!< Structure used for bit access                                */
        uint16_t w;                         /*!< Type used for word access                                    */
    } REG_SAlrtTh;

    uint16_t _unused_4;             /*!< Reserved                                                       */

    uint16_t REG_RepCap;            /*!< RepCap or reported capacity is a filteredversion of the AvCap register   */
    uint16_t REG_RepSOC;            /*!< RepSOC is the complete calculation for state-of-charge (percent)         */

    uint16_t _unused_7;             /*!< Reserved                                                       */

    uint16_t REG_Temp;              /*!< This is the most recent trimmed temperaturemeasurement         */
    uint16_t REG_Vcell;             /*!< This is the most recent trimmed cell voltageresult             */
    uint16_t REG_Current;           /*!< Current through the SYS FG pin                                 */
    uint16_t REG_AvgCurrent;        /*!< This is the 0.7s to 6.4hr (configurable) IIR average of the current  */

    uint16_t _unused_C_D[2];        /*!< Reserved                                                       */

    uint16_t REG_AvSOC;             /*!< Calculated available percentage of the battery (unfiltered)    */

    uint16_t _unused_F;             /*!< Reserved                                                       */

    uint16_t REG_FullCapRep;        /*!< Full capacity that goes with RepCap                            */
    uint16_t REG_TTE;               /*!< Estimated time-to-empty for the application                    */

    uint16_t _unused_12;            /*!< Reserved                                                       */

    uint16_t REG_FullSOCThr;        /*!< Register gates detection of end-of-charge (Default : 95%)      */
    uint16_t REG_RCell;             /*!< calculated internal resistance of the cell                     */

    uint16_t _unused_15;            /*!< Reserved                                                       */

    uint16_t REG_AvgTA;             /*!< Average of the readings from the Temp register                 */
    uint16_t REG_Cycles;            /*!< Odometer style accumulation of battery cycles                  */
    uint16_t REG_DesignCap;         /*!< Expected cell capacity                                         */
    uint16_t REG_AvgVCell;          /*!< Average of the VCell register readings                         */

    union
    {
        struct
        {
        uint8_t MinTemperature      :8;     /*!< bit: 0-7  Records the minimum Temperature                  */
        uint8_t MaxTemperature      :8;     /*!< bit: 8-15  Records the maximum Temperature                 */
        } b;                                /*!< Structure used for bit access                              */
        uint16_t w;                         /*!< Type used for word access                                  */
    } REG_MaxMinTemp;

    union 
    {
        struct
        {
        uint8_t MinVoltage          :8;     /*!< bit: 0-7  Records the VCELL minimum Voltage                */
        uint8_t MaxVoltage          :8;     /*!< bit: 8-15  Records the VCELL maximum Voltage               */
        } b;                                /*!< Structure used for bit access                              */
        uint16_t w;                         /*!< Type used for word access                                  */
    } REG_MaxMinVolt;

    union
    {
        struct
        {
        uint8_t MinCurrent          :8;     /*!< bit: 0-7  Records the minimum current reading              */
        uint8_t MaxCurrent          :8;     /*!< bit: 8-15  Records the maximum current reading             */
        } b;                                /*!< Structure used for bit access                              */
        uint16_t w;                         /*!< Type used for word access                                  */
    } REG_MaxMinCurr;

    union
    {
        struct
        {
        uint8_t Ber                 :1;     /*!< bit: 0 Enable alert on battery removal                     */
        uint8_t Bei                 :1;     /*!< bit: 1 Enable alert on battery insertion                   */
        uint8_t Aen                 :1;     /*!< bit: 2 Enable alert on fuel-gauge outputs                  */
        uint8_t FTHRM               :1;     /*!< bit: 3 Force Thermistor Bias Switch                        */
        uint8_t ETHRM               :1;     /*!< bit: 4 Enable Thermistor                                   */
        uint8_t _zero_5             :1;     /*!< bit: 5 Bit must be written 0                               */
        uint8_t COMMSH              :1;     /*!< bit: 6 Communication Shutdown                              */
        uint8_t SHDN                :1;     /*!< bit: 7 Shutdown                                            */
        uint8_t Tex                 :1;     /*!< bit: 8 Temperature External                                */
        uint8_t Ten                 :1;     /*!< bit: 9 Enable Temperature Channel                          */
        uint8_t THSH                :1;     /*!< bit: 10 TH Pin Shutdown                                    */
        uint8_t IS                  :1;     /*!< bit: 11 Current ALRT Sticky                                */
        uint8_t VS                  :1;     /*!< bit: 12 Voltage ALRT Sticky                                */
        uint8_t TS                  :1;     /*!< bit: 13 Temperature ALRT Sticky                            */
        uint8_t SS                  :1;     /*!< bit: 14 SOC ALRT Sticky                                    */
        uint8_t TSel                :1;     /*!< bit: 15 Temperature Sensor Select                          */
        } b;                                /*!< Structure used for bit access                              */
        uint16_t w;                         /*!< Type used for word access                                  */
    } REG_Config;

    uint16_t REG_IChgTerm;         /*!< Charge Termination Current                                      */
    uint16_t REG_AvCap;            /*!< Calculated Available Capacity                                   */
    uint16_t REG_TTF;              /*!< Estimated Time-to-full for the application                      */
    uint16_t REG_DevName;          /*!< Device Name (revision information)                              */

    uint16_t _unused_22_26[5];     /*!< Reserved                                                        */

    uint16_t REG_AIN0;             /*!< The external temperature measurement on the TH pin is compared to the BATT pin voltage */

    union
    {
        struct
        {
        uint8_t _zero_0             :1;     /*!< bit: 0 Bit must be written 0                               */
        uint8_t _one_1              :1;     /*!< bit: 1 Bit must be written 1                               */
        uint8_t _one_2              :1;     /*!< bit: 2 Bit must be written 1                               */
        uint8_t _zero_3             :1;     /*!< bit: 3 Bit must be written 0                               */
        uint8_t LS                  :3;     /*!< bit: 4-6 Learn Stage                                       */
        uint8_t _one_7              :1;     /*!< bit: 7 Bit must be written 1                               */
        uint8_t _zero_8             :1;     /*!< bit: 8 Bit must be written 0                               */
        uint8_t _zero_9             :1;     /*!< bit: 9 Bit must be written 0                               */
        uint8_t _one_10             :1;     /*!< bit: 10 Bit must be written 1                              */
        uint8_t _zero_11            :1;     /*!< bit: 11 Bit must be written 0                              */
        uint8_t _zero_12            :1;     /*!< bit: 12 Bit must be written 0                              */
        uint8_t _zero_13            :1;     /*!< bit: 13 Bit must be written 0                              */
        uint8_t _one_14             :1;     /*!< bit: 14 Bit must be written 1                              */
        uint8_t _zero_15            :1;     /*!< bit: 15 Bit must be written 0                              */
        } b;                                /*!< Structure used for bit access                              */
        uint16_t w;                         /*!< Type used for word access                                  */
    } REG_LearnCfg; 

    union 
    {
        struct
        {
        uint8_t NCURR               :4;     /*!< bit: 0-3  Sets the time constant for the AverageCurrent register   */
        uint8_t VOLT                :3;     /*!< bit: 4-6  Sets the time constant for the AvgVCell register */
        uint8_t MIX                 :3;     /*!< bit: 7-10  Sets the time constant for the mixingalgorithm  */
        uint8_t _reserved_11_13     :3;     /*!< bit: 11-13  Reserved                                       */
        uint8_t _one_14             :1;     /*!< bit: 14 Bit must be written 1                              */
        uint8_t _one_15             :1;     /*!< bit: 15 Bit must be written 1                              */
        } b;                                /*!< Structure used for bit access                              */
        uint16_t w;                         /*!< Type used for word access                                  */
    } REG_FilterCfg;

    uint16_t _unused_2A_39[16];     /*!< Reserved                                                        */

    union 
    {
        struct
        {
        uint8_t VR                  :7;     /*!< bit: 0-6 Recovery Voltage                                  */
        uint16_t VE                 :9;     /*!< bit: 7-15 Empty voltage target during load                 */
        } b;                                /*!< Structure used for bit access                              */
        uint16_t w;                         /*!< Type used for word access                                  */
    } REG_VEmpty;
    
    uint16_t _unused_3B_3D[3];      /*!< Reserved                                                       */

    uint16_t REG_Timer;             /*!< Timer increments once every task period                        */

    uint16_t _unused_3F_B0[114];    /*!< Reserved                                                        */

    uint16_t REG_Power;             /*!< Power register                                                  */

    uint16_t _unused_B2;            /*!< Reserved                                                        */

    uint16_t REG_AvgPower;          /*!< Filtered average power                                          */

    union 
    {
        struct
        {
        uint8_t IMIN               :8;      /*!< bit: 0-7  Minimum Current Reading                          */
        uint8_t IMAX               :8;      /*!< bit: 8-15  Maximum Current Reading                         */
        } b;                                /*!< Structure used for bit access                              */
        uint16_t w;                         /*!< Type used for word access                                  */
    } REG_IAlrtTh;

    uint16_t _unused_B5_BA[6];      /*!< Reserved                                                        */

    union 
    {
        struct
        {
        uint8_t _reserved_0         :1;     /*!< bit: 0  Reserved                                           */
        uint8_t CPMode              :1;     /*!< bit: 1  Constant-Power Mode                                */
        uint8_t DRCfg               :2;     /*!< bit: 2-3  Deep Relax Time Configuration                    */
        uint8_t _reserved_4         :1;     /*!< bit: 4  Reserved                                           */
        uint8_t LDMdl               :1;     /*!< bit: 5  Load New Model                                     */
        uint8_t TAlrtEn             :1;     /*!< bit: 6  Temperature Alert Enable                           */
        uint8_t dSOCen              :1;     /*!< bit: 7  SOC Change Alert Enable                            */
        uint8_t POWR                :4;     /*!< bit: 8-11  Sets the time constant for the AvgPower         */
        uint8_t DPEN                :1;     /*!< bit: 12  Dynamic Power Enable                              */
        uint8_t AtRateEN            :1;     /*!< bit: 13  AtRate Enable                                     */
        uint8_t _zero_14            :1;     /*!< bit: 14 Bit must be written 0                              */
        uint8_t _zero_15            :1;     /*!< bit: 15 Bit must be written 0                              */
        } b;                                /*!< Structure used for bit access                              */
        uint16_t w;                         /*!< Type used for word access                                  */
    } REG_Config2;

    uint16_t _unused_BC_BD[2];      /*!< Reserved                                                        */

    uint16_t REG_TimerH;            /*!< Long-duration time count since last POR                         */
    

}__attribute__((packed)) MAX77658_FG_REG;

/*! Format of the data to be stored in the flash */
typedef struct {
    int16_t header;
    int32_t time_s_bin;
    int16_t time_ms_bin;
    int8_t len;
    uint8_t index;
    uint16_t temperature;
    uint16_t voltage;
    uint16_t current;
    uint16_t soc;
} __attribute__((packed)) MAX77658_StorageFormat;

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED VARIABLES                            **/
/**                                                                        **/
/****************************************************************************/

#ifndef _MAX77658_C_SRC



#endif  /* _MAX77658_C_SRC */

/****************************************************************************/
/**                                                                        **/
/**                          EXPORTED FUNCTIONS                            **/
/**                                                                        **/
/****************************************************************************/

/**
 * @brief This function reads a register from the MAX77658 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, reading from a register
 *          on the MAX77658 device. The address of the register to read from is provided as a parameter,
 *          and the data read from the register is stored in a buffer also provided as a parameter.
 * 
 * @param[in]   addr      The address of the register to read from.
 * @param[out]  data      A pointer to the buffer where the read data should be stored.
 * @param[in]   reg_size  The size of the register to read from (not used in the current implementation).
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int MAX77658_read_8bit(uint8_t addr, uint8_t *data);

/**
 * @brief This function writes a byte of data to a register on the MAX77658 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, writing a byte of data to a register
 *          on the MAX77658 device. The address of the register and the data to be written are provided as parameters.
 * 
 * @param[in]   addr  The address of the register to write to.
 * @param[in]   data  The data to write to the register.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int MAX77658_write_8bit(uint8_t addr, uint8_t data);

/**
 * @brief This function reads a sequence of bytes from sequential registers on the MAX77658 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, reading a sequence of bytes from 
 *          sequential registers on the MAX77658 device. The start address of the registers and the number of bytes 
 *          to read are provided as parameters. The read data is stored in the buffer pointed to by the data parameter.
 * 
 * @param[in]   start_addr  The start address of the registers to read from.
 * @param[out]  data        A pointer to a buffer where the read data will be stored.
 * @param[in]   num_bytes   The number of bytes to read.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int MAX77658_read_8bit_seq(uint8_t start_addr, uint8_t *data, uint8_t num_bytes);

/**
 * @brief This function writes a sequence of bytes to sequential registers on the MAX77658 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, writing a sequence of bytes to 
 *          sequential registers on the MAX77658 device. The start address of the registers and the data to be written 
 *          are provided as parameters.
 * 
 * @param[in]   start_addr  The start address of the registers to write to.
 * @param[in]   data        A pointer to the data to be written.
 * @param[in]   num_bytes   The number of bytes to write.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int MAX77658_write_8bit_seq(uint8_t start_addr, uint8_t *data, uint8_t num_bytes);

/**
 * @brief This function writes a sequence of 16-bit words to sequential registers on the MAX77658 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, writing a sequence of 16-bit words to 
 *          sequential registers on the MAX77658 device. The start address of the registers and the data to be written 
 *          are provided as parameters.
 * 
 * @param[in]   I2cInstancePtr  A pointer to the TWIM instance to use for the I2C transfer.
 * @param[in]   start_address   The start address of the registers to write to.
 * @param[in]   data            A pointer to the data to be written.
 * @param[in]   num_words       The number of 16-bit words to write.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int MAX77658_write_16bit(uint8_t start_address, uint16_t *data, size_t num_words);

/**
 * @brief This function reads a sequence of 16-bit words from sequential registers on the MAX77658 device.
 * 
 * @details The function uses the TWIM driver to perform an I2C transfer, reading a sequence of 16-bit words from 
 *          sequential registers on the MAX77658 device. The start address of the registers and the number of 16-bit words 
 *          to read are provided as parameters. The read data is stored in the buffer pointed to by the data parameter.
 * 
 * @param[in]   I2cInstancePtr  A pointer to the TWIM instance to use for the I2C transfer.
 * @param[in]   start_address   The start address of the registers to read from.
 * @param[out]  data            A pointer to a buffer where the read data will be stored.
 * @param[in]   num_words       The number of 16-bit words to read.
 * 
 * @return 0 if the I2C transfer was successful, -1 otherwise.
 */
int MAX77658_read_16bit(uint8_t start_address, uint16_t *data, size_t num_words);


/**
 * @brief This function initializes the MAX77658 device.
 * 
 * @return 0 if the initialization was successful, -1 otherwise.
 */
int MAX77658_init(void);

/**
 * @brief This function configures the MAX77658 device.
 * 
 * @return 0 if the configuration was successful, -1 otherwise.
 */
int MAX77658_config(void);

/**
 * @brief This function start the continuous read operation on the MAX77658 device.
 * 
 * @return 0 if the read operation was successful, -1 otherwise.
 */
int MAX77658_start_continuous_read(void);

/**
 * @brief This function stop the continuous read operation on the MAX77658 device.
 * 
 * @return 0 if the read operation was successful, -1 otherwise.
 */
int MAX77658_stop_continuous_read(void);

/**
 * @brief Check if the battery is charging.
 * 
 * @return true if the battery is charging, false otherwise.
 */
bool get_battery_charging(void);

/**
 * @brief Check if the battery temperature is high.
 * 
 * @return true if the battery temperature is high, false otherwise.
 */
bool get_temperature_high(void);

/**
 * @brief Check if the battery is low.
 * 
 * @return true if the battery is low, false otherwise.
 */
bool get_battery_low(void);

/****************************************************************************/
/**                                                                        **/
/**                          INLINE FUNCTIONS                              **/
/**                                                                        **/
/****************************************************************************/



#endif /* _MAX77658_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/