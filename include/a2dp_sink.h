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
 * File Name: a2dp_sink.h
 *
 * Description: Definitions for constants used in the a2dp sink
 * application and function prototypes.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/
#ifndef A2DP_SINK_H
#define A2DP_SINK_H

/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include <pthread.h>
#include <ctype.h>
#include <string.h>
#include <sys/time.h>
#include "string.h"

#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "wiced_timer.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_types.h"
#include "wiced_bt_l2c.h"
#include "wiced_bt_ble.h"

#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_sbc.h"
#include "wiced_bt_a2dp_sink.h"
#include "wiced_bt_a2dp_defs.h"
#include "wiced_bt_trace.h"
#include "hcidefs.h"
#include "wiced_hal_nvram.h"

#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#include "wiced_result.h"
#include "wiced_timer.h"

#include "app_bt_utils/app_bt_utils.h"
/******************************************************************************
 *                                MACROS
 *****************************************************************************/

#define A2DP_SINK_SDP_DB_SIZE (81U)

/******************************************************************************
 *                        STRUCTURES AND ENUMERATIONS
 ******************************************************************************/

typedef enum
{
    AV_STATE_IDLE, /* Initial state (channel is unused) */
    AV_STATE_CONFIGURED, /* Remote has sent configuration request */
    AV_STATE_CONNECTED, /* Signaling Channel is connected and active */
    AV_STATE_STARTED, /* Data streaming */
} AV_STATE;

typedef enum
{
    AV_STREAM_STATE_STOPPED,
    AV_STREAM_STATE_STARTING,
    AV_STREAM_STATE_STARTED,
    AV_STREAM_STATE_STOPPING
} AV_STREAM_STATE;

typedef struct
{
    wiced_bt_device_address_t peerBda; /* Peer bd address */
    AV_STATE state; /* AVDT State machine state */
    AV_STREAM_STATE audio_stream_state; /* Audio Streaming to host state */
    wiced_bt_a2dp_codec_info_t codec_config; /* Codec configuration information */
} tAV_APP_CB;

/****************************************************************************
 *                              FUNCTION DECLARATIONS
 ***************************************************************************/

uint16_t a2dp_sink_write_nvram (int nvram_id, int data_len, void *p_data);
uint16_t a2dp_sink_read_nvram (int nvram_id, void *p_data, int data_len);
wiced_result_t a2dp_sink_management_callback (
        wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data);
void a2dp_sink_app_init( void );
void a2dp_sink_write_eir( void );

void a2dp_sink_control_cback (wiced_bt_a2dp_sink_event_t event,
                              wiced_bt_a2dp_sink_event_data_t *p_data);
void a2dp_sink_data_cback (uint8_t *p_rx_media, uint32_t media_len);
#endif  /* A2DP_APP_H */

