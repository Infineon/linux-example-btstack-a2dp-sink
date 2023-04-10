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
 * File Name: a2dp_sink.c
 *
 * Description: A2DP Sink application source file.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/

/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#include "wiced_bt_a2dp_sink.h"
#include "bt_audio_linux_platform.h"
#include "sbc_decoder.h"
#include "sbc_dec_func_declare.h"
#include "sbc_dct.h"
#include "sbc_types.h"
#include "a2dp_sink.h"

/******************************************************************************
 *                                MACROS
 ******************************************************************************/

#define A2DP_SINK_NVRAM_ID              WICED_NVRAM_VSID_START

#define WICED_HS_EIR_BUF_MAX_SIZE       (264U)

#define MSBC_STATIC_MEM_SIZE            (1920U) /* Bytes */
#define MSBC_SCRATCH_MEM_SIZE           (2048U) /* Bytes */

#define SBC_SYNC_BYTE                   (0x9C)

/* Channel Mode Values */
#define CH_MODE_MONO                    (0)
#define CH_MODE_DUAL                    (1)
#define CH_MODE_STEREO                  (2)
#define CH_MODE_JOINT_STEREO            (3)

/* Audio Sample Rate in Hz*/
#define AUDIO_SAMPLE_RATE_48kHz            (48000U)
#define AUDIO_SAMPLE_RATE_44_1kHz          (44100U)
#define AUDIO_SAMPLE_RATE_32kHz            (32000U)
#define AUDIO_SAMPLE_RATE_16kHz            (16000U)

/******************************************************************************
 *                                GLOBAL VARIABLES
 ******************************************************************************/
uint8_t pincode[4] = { 0x30, 0x30, 0x30, 0x30 };
extern const uint8_t a2dp_sink_sdp_db[A2DP_SINK_SDP_DB_SIZE];
extern const wiced_bt_cfg_settings_t a2dp_sink_cfg_settings;
extern wiced_bt_device_address_t bt_device_address;
static SINT32 staticMem[MSBC_STATIC_MEM_SIZE / sizeof (SINT32)];
static SINT32 scratchMem[MSBC_SCRATCH_MEM_SIZE / sizeof (SINT32)];
static int              PcmBytesPerFrame;
static SBC_DEC_PARAMS   strDecParams = { 0 };
static unsigned long    audio_frames_to_write = 0;
long                    audio_frames_written = 0;
uint16_t                sample_rate = 0;
tAV_APP_CB              av_app_cb;

/* Stack configuration */
extern const uint8_t                        a2dp_sink_sdp_db[A2DP_SINK_SDP_DB_SIZE];
extern const wiced_bt_cfg_settings_t        a2dp_sink_cfg_settings;
extern wiced_bt_a2dp_config_data_t          bt_audio_config;

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/* ****************************************************************************
 * Function Name: get_control_event_name
 *******************************************************************************
 * Summary:
 *          Gives the string equivalent for the Event type
 *
 * Parameters:
 *          event - Control event called back
 *
 * Return:
 *          Pointer to String containing the Event Type
 *
 * ***************************************************************************/
static const char *get_control_event_name(wiced_bt_a2dp_sink_event_t event)
{
    switch((int)event)
    {
    CASE_RETURN_STR(WICED_BT_A2DP_SINK_CONNECT_EVT)
    CASE_RETURN_STR(WICED_BT_A2DP_SINK_DISCONNECT_EVT)
    CASE_RETURN_STR(WICED_BT_A2DP_SINK_START_IND_EVT)
    CASE_RETURN_STR(WICED_BT_A2DP_SINK_START_CFM_EVT)
    CASE_RETURN_STR(WICED_BT_A2DP_SINK_SUSPEND_EVT)
    CASE_RETURN_STR(WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT)
    }

    return NULL;
}

/* ****************************************************************************
 * Function Name: get_state_name
 ******************************************************************************
 * Summary:
 *          Gives the string equivalent for the State
 *
 * Parameters:
 *          state - AV State
 *
 * Return:
 *          Pointer to String containing the State Name
 *
 * ***************************************************************************/
static const char *get_state_name(AV_STATE state)
{
    switch((int)state)
    {
    CASE_RETURN_STR(AV_STATE_IDLE)
    CASE_RETURN_STR(AV_STATE_CONFIGURED)
    CASE_RETURN_STR(AV_STATE_CONNECTED)
    CASE_RETURN_STR(AV_STATE_STARTED)
    }

    return NULL;
}

/* ****************************************************************************
 * Function Name: a2dp_sink_get_frame_len
 ******************************************************************************
 * Summary:
 *          Calculates the frame length based on the CODEC parameters chosen
 *
 * Parameters:
 *          pHeader: Header containing the SBC parameters
 *
 * Return:
 *          Frame Length
 *
 * ***************************************************************************/
static uint16_t a2dp_sink_get_frame_len (uint8_t *pHeader)
{
    uint8_t *pframeByte0;
    uint8_t *pframeByte1;
    uint8_t *pframeByte2;
    uint8_t  blocks;
    uint8_t  channelMode;
    uint8_t  subbands;
    uint8_t  bitpool;
    uint8_t  num_channels;
    uint8_t  join;
    uint32_t FrameLen;
    uint32_t tmp;

    pframeByte0 = pHeader;
    FrameLen = 0;

    if (*pframeByte0 != SBC_SYNC_BYTE)
    {
        WICED_BT_TRACE ("a2dp_sink_get_frame_len(): Bad Sync Byte: 0x%02x !!!", *pframeByte0);
        return (0);
    }

    pframeByte1 = pframeByte0 + 1;
    pframeByte2 = pframeByte1 + 1;

    /* Derive the Number of blocks and, frequency,
     * channel mode according to the Frame Header */

    /* Table 12.12: Syntax of frame_header from Advanced Audio Distribution /
     * Profile Specification
     *
     * frame_header()
     * {
     *    syncword                8
     *    sampling_frequency      2
     *    blocks                  2
     *    channel_mode            2
     *    allocation_method       1
     *    subbands                1
     *    bitpool                 8
     *    crc_check               8
     *    If (channel_mode==JOINT_STEREO)
     *    {
     *       for (sb=0;sb<nrof_subbands-1;sb++)
     *       {
     *           join[sb] 1 UiMsbf
     *       }
     *       RFA 1 UiMsbf
     *    }
     * }
     */

    blocks = (*(unsigned char *)pframeByte1 & 0x30) >> 4;

    /* Check and assign the SBC equivalent block alignment */
    switch (blocks)
    {
    case 0x0:
        blocks = SBC_BLOCK_0;
        break;
    case 0x1:
        blocks = SBC_BLOCK_1;
        break;
    case 0x2:
        blocks = SBC_BLOCK_2;
        break;
    case 0x3:
        blocks = SBC_BLOCK_3;
        break;
    }
    channelMode    = (*(unsigned char *)pframeByte1 & 0x0C) >> 2;
    num_channels = (channelMode == SBC_MONO) ? SBC_DUAL : SBC_STEREO;

    subbands    = (*(unsigned char *)pframeByte1 & 0x01);
    subbands = (subbands == 0) ? SBC_SUB_BANDS_4 : SBC_SUB_BANDS_8;

    /* Refer to 12.9 Calculation of Bit Rate and Frame Length in
     * A2DP Specification A2DP_v1.3.2.pdf
     * for the formula and calculations
     */
    bitpool = (*(unsigned char *)pframeByte2 & 0xff);

    FrameLen = 4 + (4 * subbands * num_channels) / 8;

    join = (channelMode == CH_MODE_JOINT_STEREO) ? 1 : 0;

    if (channelMode < 2) /* channel Mode is not Join */
    {
        tmp =  (blocks * num_channels * bitpool);
    }
    else
    {
        tmp =  (join * subbands) + (blocks * bitpool);
    }

    FrameLen += tmp / 8 + ((tmp % 8) ? 1 : 0);

    return ((uint16_t)FrameLen);
}

/* ****************************************************************************
 * Function Name: a2dp_config_audio_driver
 ******************************************************************************
 * Summary:
 *          Configures the Platform Audio Driver
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
static void a2dp_config_audio_driver (void)
{
    int status = BT_AUDIO_LINUX_SUCCESS;
    bt_audio_alsa_config_t alsa_config;
    wiced_bt_a2dp_codec_info_t *codec_info = &av_app_cb.codec_config;

    WICED_BT_TRACE("a2dp_config_audio_driver \n");

    if ((codec_info->cie.sbc.ch_mode == A2D_SBC_IE_CH_MD_STEREO) ||
            (codec_info->cie.sbc.ch_mode == A2D_SBC_IE_CH_MD_JOINT))
    {
        alsa_config.numOfChannels = 2;
    }
    else
    {
        alsa_config.numOfChannels = 1;
    }

    if (codec_info->cie.sbc.samp_freq == A2D_SBC_IE_SAMP_FREQ_44)
    {
        alsa_config.sample_rate = AUDIO_SAMPLE_RATE_44_1kHz;
    }
    else if (codec_info->cie.sbc.samp_freq == A2D_SBC_IE_SAMP_FREQ_48)
    {
        alsa_config.sample_rate = AUDIO_SAMPLE_RATE_48kHz;
    }
    else if (codec_info->cie.sbc.samp_freq == A2D_SBC_IE_SAMP_FREQ_32)
    {
        alsa_config.sample_rate = AUDIO_SAMPLE_RATE_32kHz;
    }
    else
    {
        alsa_config.sample_rate = AUDIO_SAMPLE_RATE_16kHz;
    }

    status = bt_audio_linux_config_alsa(&alsa_config, BT_PROFILE_A2DP);

    if (status == BT_AUDIO_LINUX_SUCCESS)
    {
        WICED_BT_TRACE("ALSA successfully configured \n");
    }
    else
    {
        WICED_BT_TRACE("ALSA Configuration failed \n");
    }
}

/* ****************************************************************************
 * Function Name: a2dp_audio_decoder_init
 ******************************************************************************
 * Summary:
 *          Initializes the SBC Decoder
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
static void a2dp_audio_decoder_init()
{
    wiced_bt_a2dp_codec_info_t *codec_info = &av_app_cb.codec_config;

    WICED_BT_TRACE("a2dp_audio_decoder_init");

    memset (staticMem, 0, sizeof (staticMem));
    memset (scratchMem, 0, sizeof (scratchMem));
    memset (&strDecParams, 0, sizeof (strDecParams));

    /* Fill the Decoder parameters */
    strDecParams.s32StaticMem  = staticMem;
    strDecParams.s32ScratchMem = scratchMem;

    strDecParams.numOfBlocks = codec_info->cie.sbc.block_len;

    if ((codec_info->cie.sbc.ch_mode == A2D_SBC_IE_CH_MD_STEREO) ||
            (codec_info->cie.sbc.ch_mode == A2D_SBC_IE_CH_MD_JOINT))
    {
        strDecParams.numOfChannels = 2;
    }
    else
    {
        strDecParams.numOfChannels = 1;
    }

    if (codec_info->cie.sbc.num_subbands == A2D_SBC_IE_SUBBAND_4)
    {
        strDecParams.numOfSubBands = 4;
    }
    else if (codec_info->cie.sbc.num_subbands == A2D_SBC_IE_SUBBAND_8)
    {
        strDecParams.numOfSubBands = 8;
    }

    if (codec_info->cie.sbc.alloc_mthd == A2D_SBC_IE_ALLOC_MD_S)
    {
        strDecParams.allocationMethod = SBC_SNR;
    }
    else if (codec_info->cie.sbc.alloc_mthd == A2D_SBC_IE_ALLOC_MD_L)
    {
        strDecParams.allocationMethod = SBC_LOUDNESS;
    }

    if (codec_info->cie.sbc.samp_freq == A2D_SBC_IE_SAMP_FREQ_44)
    {
        sample_rate = AUDIO_SAMPLE_RATE_44_1kHz;
    }
    else if (codec_info->cie.sbc.samp_freq == A2D_SBC_IE_SAMP_FREQ_48)
    {
        sample_rate = AUDIO_SAMPLE_RATE_48kHz;
    }
    else
    {
        sample_rate = AUDIO_SAMPLE_RATE_32kHz;
    }

    SBC_Decoder_decode_Init (&strDecParams);

    WICED_BT_TRACE("nblocks %d nchannels %d nsubbands %d ameth %d freq %d",
                   strDecParams.numOfBlocks , strDecParams.numOfChannels,
                   strDecParams.numOfSubBands, strDecParams.allocationMethod,
                   sample_rate);

    PcmBytesPerFrame = strDecParams.numOfBlocks * strDecParams.numOfChannels *
                        strDecParams.numOfSubBands * 2;

    /* Calculate the audio frames to write */
    audio_frames_to_write = PcmBytesPerFrame / strDecParams.numOfChannels;
    audio_frames_to_write /= 2; /*Bits per sample is 16 */
}

/* ****************************************************************************
 * Function Name: a2dp_sink_control_cback
 ******************************************************************************
 * Summary:
 *          Control callback supplied by  the a2dp sink profile code.
 *
 * Parameters:
 *          event - control event called back
 *          p_data - event data
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_sink_control_cback( wiced_bt_a2dp_sink_event_t event,
                                     wiced_bt_a2dp_sink_event_data_t* p_data )
{
    int status = BT_AUDIO_LINUX_SUCCESS;
    WICED_BT_TRACE( "[%s] Event: (%d) %s state: (%d) %s\n\r", __FUNCTION__, event,
                    get_control_event_name(event),
                    av_app_cb.state, get_state_name(av_app_cb.state));

    switch(event)
    {
    case WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT:
        /**< Codec config event, received when codec config for a streaming
         * session is updated */
        /* Maintain State */
        av_app_cb.state = AV_STATE_CONFIGURED;

        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Sink Control Callback with event data pointer NULL \n");
            return;
        }
        /* Save the configuration to setup the decoder if necessary. */
        memcpy( &av_app_cb.codec_config, &p_data->codec_config.codec, sizeof( wiced_bt_a2dp_codec_info_t ) );

        WICED_BT_TRACE(" a2dp sink codec configuration done\n");

        status = bt_audio_linux_init_alsa();

        if (status == BT_AUDIO_LINUX_SUCCESS)
        {
            WICED_BT_TRACE("ALSA successfully initialized \n");

            WICED_BT_TRACE(" Initializing SBC Decoder ...\n");
            a2dp_audio_decoder_init();

            WICED_BT_TRACE(" Now configuring Audio Driver...\n");
            a2dp_config_audio_driver();
        }
        else
        {
            WICED_BT_TRACE("ALSA Init failed \n");
        }
        break;

    case WICED_BT_A2DP_SINK_CONNECT_EVT:
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Sink Control Callback with event data pointer NULL \n");
            return;
        }
        /**< Connected event, received on establishing connection to a peer
         * device. Ready to stream. */
        if (p_data->connect.result == WICED_SUCCESS)
        {
            uint16_t settings = HCI_ENABLE_ROLE_SWITCH;

            WICED_BT_TRACE( "[%s] connected to addr: <%B> Handle:%d\n\r", __FUNCTION__, p_data->connect.bd_addr,
                            p_data->connect.handle );

            /* Save the address of the remote device on remote connection */
            memcpy(av_app_cb.peerBda, p_data->connect.bd_addr, sizeof(wiced_bt_device_address_t));

            /* Maintain State */
            av_app_cb.state = AV_STATE_CONNECTED;

            WICED_BT_TRACE(" a2dp sink connected \n");
        }
        else
        {
            WICED_BT_TRACE("a2dp Connection failed \n");
        }
        break;

    case WICED_BT_A2DP_SINK_DISCONNECT_EVT:
        /**< Disconnected event, received on disconnection from a peer device */
        /* Maintain State */
        av_app_cb.state = AV_STATE_IDLE;
        bt_audio_linux_deinit_alsa();
        WICED_BT_TRACE(" a2dp sink disconnected \n");
        break;

    case WICED_BT_A2DP_SINK_START_IND_EVT:
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Sink Control Callback with event data pointer NULL \n");
            return;
        }
        /**< Start stream indication, Send response */
        WICED_BT_TRACE("  a2dp sink start indication from Peer @handle: %d, %x \n", p_data->start_ind.handle,
                       p_data->start_ind.label );
        if (!wiced_bt_a2dp_sink_send_start_response( p_data->start_ind.handle, p_data->start_ind.label,
                                                      A2D_SUCCESS))
        {
            /* Maintain State */
            av_app_cb.state = AV_STATE_STARTED;
            WICED_BT_TRACE(" a2dp sink streaming started \n");
        }
        break;

    case WICED_BT_A2DP_SINK_START_CFM_EVT:
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Sink Control Callback with event data pointer NULL \n");
            return;
        }
        /**< Start stream event, received when audio streaming is about to
         * start */
        /* Maintain State */
        av_app_cb.state = AV_STATE_STARTED;
        WICED_BT_TRACE(" a2dp sink streaming started handle:%d\n", p_data->start_cfm.handle);
        break;

    case WICED_BT_A2DP_SINK_SUSPEND_EVT:
        /**< Suspend stream event, received when audio streaming is
         * suspended */
        /* Maintain State */
        av_app_cb.state = AV_STATE_CONNECTED;
        WICED_BT_TRACE(" a2dp sink streaming suspended \n");
        break;

    default:
        break;
    }
}

/* ****************************************************************************
 * Function Name: a2dp_sink_data_cback
 ******************************************************************************
 * Summary:
 *          Data supplied by  the a2dp sink profile code.
 *
 * Parameters:
 *          p_a2dp_data   - A2DP media data
 *          a2dp_data_len - A2DP data length
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_sink_data_cback( uint8_t* p_rx_media, uint32_t media_len )
{
    int status = BT_AUDIO_LINUX_SUCCESS;
    uint8_t         i = 0, num_frames = 0;
    uint16_t        frame_len = 0;
    uint16_t        *pOut = NULL;

    WICED_BT_TRACE("Received a2dp data len %d Num frames %d", media_len, (*p_rx_media & 0x0F));

    num_frames =  *p_rx_media & 0x0F;
    p_rx_media++;
    media_len--;

    for (i = 0; i < num_frames; i++)
    {
        /* Get frame length. stop if any error */
        if ((frame_len = a2dp_sink_get_frame_len (p_rx_media)) == 0)
        {
            break;
        }

        if (frame_len > media_len)
        {
            WICED_BT_TRACE ("a2dp_sink_proc_stream_data(): media too short %d %d %d %d", num_frames, i,
                            frame_len, media_len);
            break;
        }

        pOut = (uint16_t *) wiced_bt_get_buffer(PcmBytesPerFrame);

        if (pOut == NULL)
        {
            WICED_BT_TRACE ("a2dp_sink_proc_stream_data(): NO BUFFER !!!  ");
            break;
        }

        SBC_Decoder_decoder (&strDecParams, p_rx_media, frame_len, (SINT16 *)pOut);

        status = bt_audio_linux_data_write((void *)pOut, audio_frames_to_write,
                                           &audio_frames_written);

        if (status != BT_AUDIO_LINUX_SUCCESS)
        {
            WICED_BT_TRACE ("bt_audio_linux_data_write () failed. Status: %d", status);
        }

        p_rx_media += frame_len;
        media_len  -= frame_len;
        wiced_bt_free_buffer(pOut);
    }
}

/******************************************************************************
 * Function Name: a2dp_sink_write_eir
 *******************************************************************************
 * Summary:
 *          Prepare extended inquiry response data.  Current version publishes
 *          audio sink services.
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 ******************************************************************************/
void a2dp_sink_write_eir( void )
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;

    pBuf = (uint8_t*)wiced_bt_get_buffer( WICED_HS_EIR_BUF_MAX_SIZE );
    WICED_BT_TRACE( "a2dp_sink_write_eir %x\n", pBuf );

    if ( !pBuf )
    {
        return;
    }
    p = pBuf;

    length = strlen( (char *)a2dp_sink_cfg_settings.device_name );

    *p++ = length + 1;
    *p++ = BT_EIR_COMPLETE_LOCAL_NAME_TYPE;   /* EIR type full name */
    memcpy( p, a2dp_sink_cfg_settings.device_name, length );
    p += length;
    *p++ = ( 1 * 2 ) + 1;     /* length of services + 1 */
    *p++ =   BT_EIR_COMPLETE_16BITS_UUID_TYPE;
    /* EIR type full list of 16 bit service UUIDs */
    *p++ =   UUID_SERVCLASS_AUDIO_SINK        & 0xff;
    *p++ = ( UUID_SERVCLASS_AUDIO_SINK >> 8 ) & 0xff;
    *p++ = 0;

    /* print EIR data */
    WICED_BT_TRACE_ARRAY( ( uint8_t* )( pBuf+1 ), MIN( p-( uint8_t* )pBuf,100 ), "EIR :" );
    wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) );
}

/******************************************************************************
 * Function Name: a2dp_sink_write_nvram
 *******************************************************************************
 * Summary:
 *          Write NVRAM function is called to store information in the NVRAM.
 *
 * Parameters:
 *          nvram_id: NVRAM Id
 *          data_len: Length of the data to be written
 *          p_data: Data to be written
 *
 * Return:
 *          Number of bytes written
 *
 *****************************************************************************/
uint16_t a2dp_sink_write_nvram( int nvram_id, int data_len, void *p_data)
{
    wiced_result_t  result;
    int             bytes_written = 0;

    if (p_data != NULL)
    {
        bytes_written = wiced_hal_write_nvram( nvram_id, data_len, (uint8_t*)p_data, &result );
        WICED_BT_TRACE("NVRAM ID:%d written :%d bytes result:%d\n", nvram_id, bytes_written, result);
    }

    return (bytes_written);
}

/******************************************************************************
 * Function Name: a2dp_sink_read_nvram
 *******************************************************************************
 * Summary:
 *          Read data from the NVRAM and return in the passed buffer
 *
 * Parameters:
 *          nvram_id: NVRAM Id
 *          data_len: Length of the data to be read
 *          p_data: Data buffer pointer to hold read data
 *
 * Return:
 *          Number of bytes read
 *
 *****************************************************************************/
uint16_t a2dp_sink_read_nvram( int nvram_id, void *p_data, int data_len)
{
    uint16_t        read_bytes = 0;
    wiced_result_t  result;

    if ((p_data != NULL) && (data_len >= sizeof(wiced_bt_device_link_keys_t)))
    {
        read_bytes = wiced_hal_read_nvram( nvram_id, sizeof(wiced_bt_device_link_keys_t), p_data, &result );
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d result:%d\n", nvram_id,
                       sizeof(wiced_bt_device_link_keys_t), read_bytes, result );
    }
    return (read_bytes);
}

/******************************************************************************
 * Function Name: a2dp_sink_app_init()
 *******************************************************************************
 * Summary:
 *   This function handles application level initialization tasks and is
 *   called from the BT management callback once the Bluetooth stack enabled
 *   event (BTM_ENABLED_EVT) is triggered
 *   This function is executed in the BTM_ENABLED_EVT management callback.
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 ******************************************************************************/
void a2dp_sink_app_init( void )
{
    wiced_result_t result = WICED_BT_SUCCESS;
    av_app_cb.state = AV_STATE_IDLE;

    /* Register with the A2DP sink profile code */
    result = wiced_bt_a2dp_sink_init( &bt_audio_config,
                                         a2dp_sink_control_cback );
    if (result == WICED_BT_SUCCESS)
    {
        wiced_bt_a2dp_sink_register_data_cback(a2dp_sink_data_cback);
    }
}

/******************************************************************************
 * Function Name: a2dp_sink_management_callback()
 *******************************************************************************
 * Summary:
 *   This is a Bluetooth stack event handler function to receive management
 *   events from the Bluetooth stack and process as per the application.
 *
 * Parameters:
 *   wiced_bt_management_evt_t event : BLE event code of one byte length
 *   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management
 *   event structures
 *
 * Return:
 *  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 ******************************************************************************/

wiced_result_t a2dp_sink_management_callback( wiced_bt_management_evt_t event,
                                              wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_ble_advert_mode_t          *p_mode; /* Advertisement Mode */
    uint8_t                             *p_keys; /* Paired event Link keys */
    wiced_bt_device_address_t           bda = {0};
    wiced_result_t                      result = WICED_BT_SUCCESS;

    wiced_bt_power_mgmt_notification_t *p_power_mgmt_notification;
    wiced_bt_dev_encryption_status_t   *p_encryption_status;
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    int                                 pairing_result;
    const uint8_t                      *link_key;

    WICED_BT_TRACE( "A2dp sink management callback: %x\n", event );

    switch( event )
    {
        /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        if (WICED_BT_SUCCESS == p_event_data->enabled.status)
        {
            /* Bluetooth is enabled */
            wiced_bt_set_local_bdaddr((uint8_t *)bt_device_address, BLE_ADDR_PUBLIC);
            wiced_bt_dev_read_local_addr(bda);
            WICED_BT_TRACE("Bluetooth Enabled \n");
            WICED_BT_TRACE("Local Bluetooth Address: %02X:%02X:%02X:%02X:%02X:%02X\n", bda[0], bda[1], bda[2],
                    bda[3], bda[4], bda[5] );  

            /* Enable pairing */
            wiced_bt_set_pairable_mode(WICED_TRUE, 0);

            a2dp_sink_write_eir();

            /* create SDP records */
            wiced_bt_sdp_db_init((uint8_t*) a2dp_sink_sdp_db,
                sizeof(a2dp_sink_sdp_db));

           /* start the a2dp application */
           a2dp_sink_app_init();

           /* Making the sink device discoverable and connectable */
           wiced_bt_dev_set_discoverability(BTM_GENERAL_DISCOVERABLE,
                BTM_DEFAULT_DISC_WINDOW, BTM_DEFAULT_DISC_INTERVAL);
           wiced_bt_dev_set_connectability(WICED_TRUE, BTM_DEFAULT_CONN_WINDOW,
                BTM_DEFAULT_CONN_INTERVAL);
        }
        else
        {
            WICED_BT_TRACE("Bluetooth Enable failed \n");
        }
        break;

    case BTM_DISABLED_EVT:
        WICED_BT_TRACE("Bluetooth Disabled \n");
        break;

    case BTM_PIN_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        WICED_BT_TRACE("remote address= ");
        print_bd_address(*p_event_data->pin_request.bd_addr);

        wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr,
                result/*WICED_BT_SUCCESS*/, 4, &pincode[0]);
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        WICED_BT_TRACE("Numeric_value: %d \n",
                p_event_data->user_confirmation_request.numeric_value);
        wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS,
                p_event_data->user_confirmation_request.bd_addr);
        break;

    case BTM_PASSKEY_NOTIFICATION_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        WICED_BT_TRACE("PassKey Notification. BDA: ");
        print_bd_address(p_event_data->user_passkey_notification.bd_addr);
        WICED_BT_TRACE("PassKey Notification.  Key %d \n",
                p_event_data->user_passkey_notification.passkey);
        wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS,
                p_event_data->user_passkey_notification.bd_addr);
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* Use the default security for BR/EDR*/
        WICED_BT_TRACE("Pairing Capabilities Request, bda %B\n",
                p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
        p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap =
                BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_br_edr_request.auth_req =
                BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap      =
                BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data          =
                BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req          =
                BTM_LE_AUTH_REQ_SC_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size      =
                0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys         =
                BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys         =
                BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        p_pairing_cmpl = &p_event_data->pairing_complete;

        if( p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR )
        {
            pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
        }
        else
        {
            pairing_result = p_pairing_cmpl->pairing_complete_info.ble.status;
        }
        WICED_BT_TRACE("Pairing complete %d \n",pairing_result );
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* save keys to NVRAM */
        p_keys = (uint8_t*)&p_event_data->paired_device_link_keys_update;

        /* This application supports a single paired host, we can save keys
         * under the same NVRAM ID overwriting previous pairing if any */
        a2dp_sink_write_nvram(A2DP_SINK_NVRAM_ID,
                sizeof(wiced_bt_device_link_keys_t), p_keys);
        link_key =
                p_event_data->paired_device_link_keys_update.key_data.br_edr_key;
        WICED_BT_TRACE(
                " LinkKey:%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                link_key[0], link_key[1], link_key[2], link_key[3], link_key[4],
                link_key[5], link_key[6], link_key[7], link_key[8], link_key[9],
                link_key[10], link_key[11], link_key[12], link_key[13],
                link_key[14], link_key[15]);
        break;

    case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* read existing key from the NVRAM  */
        if (a2dp_sink_read_nvram(A2DP_SINK_NVRAM_ID,
                &p_event_data->paired_device_link_keys_request,
                sizeof(wiced_bt_device_link_keys_t)) != 0)
        {
            result = WICED_BT_SUCCESS;
        }
        else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE("Key retrieval failure\n");
        }
        break;

    case BTM_POWER_MANAGEMENT_STATUS_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
        WICED_BT_TRACE(
                "Power mgmt status event: bd ( %B ) status:%d hci_status:%d\n",
                p_power_mgmt_notification->bd_addr,
                p_power_mgmt_notification->status,
                p_power_mgmt_notification->hci_status);
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* save keys to NVRAM */
        p_keys = (uint8_t*) &p_event_data->local_identity_keys_update;
        wiced_hal_write_nvram(A2DP_SINK_NVRAM_ID,
                sizeof(wiced_bt_local_identity_keys_t), p_keys, &result);
        WICED_BT_TRACE("local keys save to NVRAM ");
        WICED_BT_TRACE("  result: %d \n", result);
        break;

    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* read keys from NVRAM */
        p_keys = (uint8_t*) &p_event_data->local_identity_keys_request;
        wiced_hal_read_nvram(A2DP_SINK_NVRAM_ID,
                sizeof(wiced_bt_local_identity_keys_t), p_keys, &result);
        WICED_BT_TRACE("local keys read from NVRAM ");
        WICED_BT_TRACE("  result: %d \n", result);
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        p_encryption_status = &p_event_data->encryption_status;
        WICED_BT_TRACE( "Encryption Status Event:  res %d", p_encryption_status->result);
        break;

    case BTM_SECURITY_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,
                WICED_BT_SUCCESS);
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        p_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE( "Advertisement State Change: %d\n", *p_mode);
        break;

    case BTM_BLE_CONNECTION_PARAM_UPDATE:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* Connection parameters updated */
        if (WICED_SUCCESS == p_event_data->ble_connection_param_update.status) {
            WICED_BT_TRACE("Supervision Time Out = %d\n",
                    (p_event_data->ble_connection_param_update.supervision_timeout
                            * 10));
        }
        break;

    default:
        WICED_BT_TRACE("Unhandled Bluetooth Management Event: 0x%x %s\n", event,
        get_bt_event_name(event));
        break;
    }

    return result;
}
/* [] END OF FILE */
