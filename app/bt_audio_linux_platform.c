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
 * File Name: audio_platform_common.c
 *
 * Description: This file contains the definitions of functions related to
 * Linux platform specific audio handling.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#include "bt_audio_linux_platform.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "asoundlib.h"

/******************************************************************************
 *                                GLOBAL VARIABLES
 *****************************************************************************/
static char *alsa_device = "default";

static snd_pcm_t *p_alsa_handle = NULL;
static snd_pcm_hw_params_t *params;
static snd_pcm_uframes_t frames;

static snd_mixer_elem_t *snd_mixer_elem = NULL;
static snd_mixer_t *snd_mixer_handle = NULL;
static snd_mixer_selem_id_t *snd_sid = NULL;
static long vol_max;

snd_pcm_uframes_t buffer_size = 0;
snd_pcm_uframes_t period_size = 0;


/******************************************************************************
 *                                FUNCTION DEFINITIONS
 *****************************************************************************/

/* ****************************************************************************
 * Function Name: alsa_volume_driver_deinit
 *******************************************************************************
 * Summary:
 *          De-initializes the ALSA Volume Driver
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
static int alsa_volume_driver_deinit (void)
{
    int status = BT_AUDIO_LINUX_SUCCESS;
    if (snd_mixer_handle != NULL)
    {
        if (!(snd_mixer_close (snd_mixer_handle)))
        {
            snd_mixer_handle = NULL;
        }
        else
        {
            status = BT_AUDIO_LINUX_ERROR_GENERAL;
            WICED_BT_TRACE ("sound mixer close failed. \n");
        }
    }
    if (snd_sid != NULL)
    {
        snd_mixer_selem_id_free (snd_sid);
        snd_sid = NULL;
    }
    snd_mixer_elem = NULL;
    return status;
}

/* ****************************************************************************
 * Function Name: alsa_volume_driver_init
 *******************************************************************************
 * Summary:
 *          Initializes the ALSA Volume Driver
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
static int alsa_volume_driver_init (void)
{
    long vol_min;
    int status = BT_AUDIO_LINUX_SUCCESS;
    WICED_BT_TRACE ("alsa_volume_driver_init\n");

    if (BT_AUDIO_LINUX_SUCCESS != alsa_volume_driver_deinit ())
    {
        WICED_BT_TRACE ("alsa_volume_driver_deinit failed\n");
        return BT_AUDIO_LINUX_ERROR_GENERAL;
    }

    /* Open the Sound mixer and register the Sound element */
    snd_mixer_open (&snd_mixer_handle, 0);
    if (snd_mixer_handle == NULL)
    {
        WICED_BT_TRACE ("alsa_volume_driver_init snd_mixer_open Failed\n");
        status = BT_AUDIO_LINUX_ERROR_GENERAL;
    }
    else
    {
        snd_mixer_attach (snd_mixer_handle, "default");
        snd_mixer_selem_register (snd_mixer_handle, NULL, NULL);
        snd_mixer_load (snd_mixer_handle);

        snd_mixer_selem_id_malloc (& snd_sid);
        if (snd_sid == NULL)
        {
            alsa_volume_driver_deinit ();
            WICED_BT_TRACE ("alsa_volume_driver_init snd_mixer_selem_id_alloca Failed\n");
            status = BT_AUDIO_LINUX_ERROR_GENERAL;
        }
        else
        {
            for(snd_mixer_elem = snd_mixer_first_elem(snd_mixer_handle); snd_mixer_elem; snd_mixer_elem = snd_mixer_elem_next(snd_mixer_elem)){
                if(snd_mixer_elem_get_type(snd_mixer_elem) == 0 && snd_mixer_selem_is_active(snd_mixer_elem))
                {
                    if (snd_mixer_elem)
                    {
                                snd_mixer_selem_get_playback_volume_range (snd_mixer_elem, & vol_min, &vol_max);
                    }
                    else
                    {
                                alsa_volume_driver_deinit ();
                                WICED_BT_TRACE ("alsa_volume_driver_init snd_mixer_find_selem Failed \n");
                        status = BT_AUDIO_LINUX_ERROR_GENERAL;
                    }
                }
            }
        }
    }
    return status;
}

/* ****************************************************************************
 * Function Name: bt_audio_linux_set_volume
 *******************************************************************************
 * Summary:
 *          Sets the required volume level in the ALSA driver
 *
 * Parameters:
 *          Required volume
 *
 * Return:
 *          status code
 *
 * ***************************************************************************/
int bt_audio_linux_set_volume (uint8_t volume)
{
    int status = BT_AUDIO_LINUX_SUCCESS;

    WICED_BT_TRACE ("alsa_set_volume volume %d", volume);

    if (snd_mixer_elem)
    {
        status = snd_mixer_selem_set_playback_volume_all (
                snd_mixer_elem, volume * vol_max / 100);
    }
    else
    {
        status = BT_AUDIO_LINUX_ERROR_GENERAL;
    }

    return status;
}

/* ****************************************************************************
 * Function Name: bt_audio_linux_set_mute_state
 *******************************************************************************
 * Summary:
 *          Sets the Mute state as required
 *
 * Parameters:
 *          mute_enabled: Value of Enable/ disable
 *
 * Return:
 *          status code
 *
 * ***************************************************************************/
int bt_audio_linux_set_mute_state (int mute_enabled)
{
    int status = BT_AUDIO_LINUX_SUCCESS;
    WICED_BT_TRACE ("audio_driver_mute_state %d \n", mute_enabled);
    if (snd_mixer_elem == NULL)
    {
        status = alsa_volume_driver_init ();
    }
    if ((BT_AUDIO_LINUX_SUCCESS == status) && (snd_mixer_elem))
    {
        status = snd_mixer_selem_set_playback_switch_all (snd_mixer_elem,
                                                 mute_enabled ? 0: 1);
    }
    return status;
}

/* ****************************************************************************
 * Function Name: bt_audio_linux_init_alsa
 *******************************************************************************
 * Summary:
 *          Initializes the ALSA driver
 *
 * Parameters:
 *          None
 *
 * Return:
 *          status code
 *
 * ***************************************************************************/
int bt_audio_linux_init_alsa (void)
{
    int status = BT_AUDIO_LINUX_SUCCESS;

    WICED_BT_TRACE ("bt_audio_linux_init_alsa");

    /* If ALSA PCM driver was already open => close it */
    if (p_alsa_handle != NULL)
    {
        WICED_BT_TRACE (
                "calling snd_pcm_close() to close already opened stream.");
        snd_pcm_close (p_alsa_handle);
        p_alsa_handle = NULL;
    }

    WICED_BT_TRACE ("snd_pcm_open");
    status = snd_pcm_open (& (p_alsa_handle), alsa_device,
                           SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK);

    if (status < BT_AUDIO_LINUX_SUCCESS)
    {
        WICED_BT_TRACE ("snd_pcm_open failed: %s", snd_strerror (status));
    }
    else
    {
        WICED_BT_TRACE ("ALSA driver opened");

        /* Init volume driver */
        status = alsa_volume_driver_init ();
    }

    return status;
}

/* ****************************************************************************
 * Function Name: bt_audio_linux_config_alsa
 *******************************************************************************
 * Summary:
 *          Configures the PCM settings in ALSA
 *
 * Parameters:
 *         config_param: Configuration parameters such as Sample rate & channels
 *          bt_prof: Bluetooth Profile
 * Return:
 *          status code
 *
 * ***************************************************************************/
int bt_audio_linux_config_alsa (bt_audio_alsa_config_t *config_param,
                                bt_profile_type bt_prof)
{
    int status = BT_AUDIO_LINUX_SUCCESS;
    snd_pcm_format_t format = SND_PCM_FORMAT_S16_LE; /* required PCM format */
    snd_pcm_access_t access = SND_PCM_ACCESS_RW_INTERLEAVED; /* required PCM access */
    int soft_resample = 1; /* 0 = disallow alsa-lib resample stream, 1 = allow resampling */
    unsigned int latency; /* required overall latency in Microseconds */

    WICED_BT_TRACE ("config_alsa \n");

    if (p_alsa_handle == NULL)
    {
        WICED_BT_TRACE ("ALSA is not initialized \n");
        status = BT_AUDIO_LINUX_ERROR_GENERAL;
    }
    else
    {
        switch (bt_prof)
        {
        case BT_PROFILE_A2DP:
            format = SND_PCM_FORMAT_S16_LE;
            access = SND_PCM_ACCESS_RW_INTERLEAVED;
            soft_resample = 1;
            latency = 150000; /* 150msec */
            break;

        case BT_PROFILE_HFP:
            format = SND_PCM_FORMAT_S16_LE;
            access = SND_PCM_ACCESS_RW_INTERLEAVED;
            soft_resample = 1;
            latency = 150000; /* 150msec */
            break;
        }

        /* Configure ALSA driver with PCM parameters */
        status = snd_pcm_set_params (p_alsa_handle, format, access,
                                     config_param->numOfChannels,
                                     config_param->sample_rate, soft_resample,
                                     latency);

        if (status < 0)
        {
            WICED_BT_TRACE ("snd_pcm_set_params failed: %s",
                            snd_strerror (status));
        }
        else
        {
            snd_pcm_get_params (p_alsa_handle, & buffer_size, & period_size);
            WICED_BT_TRACE ("snd_pcm_get_params150ms bs %d ps %d", buffer_size,
                            period_size);
        }
    }
    return status;
}

/* ****************************************************************************
 * Function Name: bt_audio_linux_deinit_alsa
 *******************************************************************************
 * Summary:
 *          De-initializes the ALSA driver
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 * ***************************************************************************/
void bt_audio_linux_deinit_alsa (void)
{
    WICED_BT_TRACE ("deinit_alsa");
    if (p_alsa_handle != NULL)
    {
        WICED_BT_TRACE ("snd_pcm_close");
        snd_pcm_close (p_alsa_handle);
        p_alsa_handle = NULL;
    }
    alsa_volume_driver_deinit ();
}

/* ****************************************************************************
 * Function Name: bt_audio_linux_data_write
 *******************************************************************************
 * Summary:
 *          Writes the supplied PCM frames to ALSA driver
 *
 * Parameters:
 *          pOut: The PCM buffer to be written
 *          alsa_frames_to_send: Number of frames to be written
 *          num_alsa_frames_written: Number of frames that were written
 *
 * Return:
 *          status code
 *
 * ***************************************************************************/
int bt_audio_linux_data_write (void *pOut, unsigned long alsa_frames_to_send,
                               long *num_alsa_frames_written)
{
    int status = BT_AUDIO_LINUX_SUCCESS;
    int i = 0;
    snd_pcm_sframes_t alsa_frames = 0;

    if ( (p_alsa_handle == NULL) || (pOut == NULL))
    {
        WICED_BT_TRACE ("ALSA is not configured or Input is Invalid.");
        status = BT_AUDIO_LINUX_ERROR_GENERAL;
    }
    else
    {
        alsa_frames = snd_pcm_writei (p_alsa_handle, pOut, alsa_frames_to_send);

        if (alsa_frames < 0)
        {
            alsa_frames = snd_pcm_recover (p_alsa_handle, alsa_frames, 0);
        }
        if (alsa_frames < 0)
        {
            WICED_BT_TRACE ("app_avk_uipc_cback snd_pcm_writei failed %s",
                            snd_strerror (alsa_frames));
        }
        if (alsa_frames > 0 && alsa_frames < alsa_frames_to_send)
        {
            WICED_BT_TRACE (
                    "app_avk_uipc_cback Short write (expected %li, wrote %li)",
                    (long) alsa_frames_to_send, alsa_frames);
        }
    }
    *num_alsa_frames_written = alsa_frames;
    return status;
}

