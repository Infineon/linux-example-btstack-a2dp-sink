/******************************************************************************
 * (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * ("Software"), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries ("Cypress") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 ******************************************************************************/

/******************************************************************************
 * File Name: audio_platform_common.h
 *
 * Description: This file contains the declarations of data types and functions
 * for Linux platform specific audio handling.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

#ifndef BT_AUDIO_LINUX_PLATFORM_H_
#define BT_AUDIO_LINUX_PLATFORM_H_


/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#include <stdio.h>
#include <stdint.h>

#include "wiced_bt_trace.h"

/******************************************************************************
 *                                MACROS
 *****************************************************************************/
/* Return Codes */
#define BT_AUDIO_LINUX_SUCCESS              (0)
#define BT_AUDIO_LINUX_ERROR_GENERAL        (1)
#define BT_AUDIO_LINUX_ERROR_WRONG_PARAM    (2)
/* Others Error codes are returned as is from Linux OS calls */

/******************************************************************************
 *                         STRUCTURES AND ENUMERATIONS
 ******************************************************************************/
typedef struct
{
    unsigned int numOfChannels;
    unsigned int sample_rate;
} bt_audio_alsa_config_t;

typedef enum
{
    BT_PROFILE_A2DP = 1, BT_PROFILE_HFP
} bt_profile_type;

/******************************************************************************
 *                         FUNCTION DECLARATIONS
 ******************************************************************************/
int bt_audio_linux_init_alsa (void);

void bt_audio_linux_deinit_alsa (void);

int bt_audio_linux_config_alsa (bt_audio_alsa_config_t *config_param,
                                bt_profile_type bt_prof);

int bt_audio_linux_set_volume (uint8_t volume);

int bt_audio_linux_set_mute_state (int mute_enabled);

int bt_audio_linux_data_write (void *pOut, unsigned long alsa_frames_to_send,
                               long *num_alsa_frames_written);

#endif /* BT_AUDIO_LINUX_PLATFORM_H_ */
