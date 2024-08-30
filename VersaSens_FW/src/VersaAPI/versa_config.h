/*
                              *******************
******************************* H SOURCE FILE *******************************
**                            *******************                          **
**                                                                         **
** project  : VersaSens                                                        **
** filename : versa_config.h                                                   **
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
* @file   versa_config.h
* @date   DD/MM/YY
* @brief  This is the main header of versa_config
*
* Here typically goes a more extensive explanation of what the header
* defines.
*/

#ifndef _VERSA_CONFIG_H
#define _VERSA_CONFIG_H

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

/***********************      DO NOT MODIFY       ***************************/

/* ADS1298 modes */
#define VCONF_ADS1298_MODE_EEG  1
#define VCONF_ADS1298_MODE_ECG  0

/* ADS1298 freq */
#define VCONF_ADS1298_FS_250    0b110
#define VCONF_ADS1298_FS_500    0b101
#define VCONF_ADS1298_FS_1000   0b100
#define VCONF_ADS1298_FS_2000   0b011
#define VCONF_ADS1298_FS_4000   0b010
#define VCONF_ADS1298_FS_8000   0b001
#define VCONF_ADS1298_FS_16000  0b000

/* ADS1298 gain */
#define VCONF_ADS1298_GAIN_6    0b000
#define VCONF_ADS1298_GAIN_1    0b001
#define VCONF_ADS1298_GAIN_2    0b010
#define VCONF_ADS1298_GAIN_3    0b011
#define VCONF_ADS1298_GAIN_4    0b100
#define VCONF_ADS1298_GAIN_8    0b101
#define VCONF_ADS1298_GAIN_12   0b110

/* MAX30001 modes */
#define VCONF_MAX30001_MODE_ECG         0
#define VCONF_MAX30001_MODE_ECG_BIOZ    1


/***********************      CONFIGURATIONS      ***************************/

/* Enable/Disable sensors */
#define VCONF_ADS1298_EN        1
#define VCONF_BNO086_EN         1
#define VCONF_MAX30001_EN       1
#define VCONF_MAX86178_EN       1
#define VCONF_MLX90632_EN       1
#define VCONF_T5838_EN          1
#define VCONF_MAX77658_EN       1

/* ADS1298 configuration */
#define VCONF_ADS1298_FS        VCONF_ADS1298_FS_1000
#define VCONF_ADS1298_GAIN      VCONF_ADS1298_GAIN_12
#define VCONF_ADS1298_SUBSAMPLING_FACTOR    4

/* MAX30001 configuration */
#define VCONF_MAX30001_MODE     VCONF_MAX30001_MODE_ECG_BIOZ


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

/*! Configuration variables */
extern int vconf_ads1298_en;
extern int vconf_max30001_en;
extern int vconf_bno086_en;
extern int vconf_max86178_en;
extern int vconf_mlx90632_en;
extern int vconf_t5838_en;
extern int vconf_max77658_en;

extern int vconf_ads1298_fs;
extern int vconf_ads1298_gain;

extern int vconf_max30001_mode;


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


#endif /* _VERSA_CONFIG_H */
/****************************************************************************/
/**                                                                        **/
/**                                EOF                                     **/
/**                                                                        **/
/****************************************************************************/