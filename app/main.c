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
 * File Name: main.c
 *
 * Description: Entry file for A2DP Sink application.
 *
 * Related Document: See README.md
 *
 ******************************************************************************/
/******************************************************************************
 *                                INCLUDES
 *****************************************************************************/
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "platform_linux.h"
#include "a2dp_sink.h"
#include "app_bt_utils/app_bt_utils.h"
#include "utils_arg_parser.h"

/******************************************************************************
 *                                MACROS
 *****************************************************************************/
#define A2DP_SINK_NVRAM_ID              WICED_NVRAM_VSID_START

#define WICED_HS_EIR_BUF_MAX_SIZE       (264U)
#define MAX_PATH                        (256U)
#define LOCAL_BDA_LEN                   (50U)
#define IP_ADDR_LEN                     (16U)
#define BT_STACK_HEAP_SIZE              (0xF000U)

#define IP_ADDR                         "000.000.000.000"

extern wiced_bt_device_address_t bt_device_address;

/******************************************************************************
 *                                EXTERNS
 *****************************************************************************/
/* Stack configuration */
extern const wiced_bt_cfg_settings_t a2dp_sink_cfg_settings;

/* ***************************************************************************
 *                              GLOBAL VARIABLES
 * **************************************************************************/
wiced_bt_heap_t *p_default_heap = NULL;

/******************************************************************************
 *                          FUNCTION DEFINITIONS
 ******************************************************************************/

/******************************************************************************
 * Function Name: hci_control_proc_rx_cmd()
 *******************************************************************************
 * Summary:
 *          Function to handle HCI receive
 *
 * Parameters:
 *          uint8_t* p_buffer  : rx buffer
 *          uint32_t length     : rx buffer length
 *
 * Return:
 *          status code
 *
 ******************************************************************************/
uint32_t hci_control_proc_rx_cmd (uint8_t *p_buffer, uint32_t length)
{
  return 0;
}

/******************************************************************************
 * Function Name: application_start()
 *******************************************************************************
 * Summary:
 *          BT stack initialization function
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 ******************************************************************************/
void application_start (void)
{
  wiced_result_t wiced_result = WICED_BT_SUCCESS;

#ifdef WICED_BT_TRACE_ENABLE
  /* Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints */
  /* wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE); */

  /* Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart() */
  wiced_set_debug_uart (WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif /* WICED_BT_TRACE_ENABLE */

  /* Register call back and configuration with stack */
    wiced_result = wiced_bt_stack_init(a2dp_sink_management_callback, &a2dp_sink_cfg_settings);

    WICED_BT_TRACE("AIROC CYW5557x A2DP Sink Start");

  /* Check if stack initialization was successful */

  if (WICED_BT_SUCCESS == wiced_result)
  {
    /* Create a buffer heap, make it the default heap.  */
        p_default_heap = wiced_bt_create_heap("app", NULL, BT_STACK_HEAP_SIZE,
            NULL, WICED_TRUE);
    if (NULL == p_default_heap)
    {
            fprintf(stderr, "create default heap error: size %d\n", BT_STACK_HEAP_SIZE);
    }
  }

  if ((WICED_BT_SUCCESS == wiced_result) && (NULL != p_default_heap))
  {
		WICED_BT_TRACE ("Bluetooth Stack Initialization Successful...");
  }
    else /* Exit App if stack init was not successful or heap creation failed */
  {
		WICED_BT_TRACE ("Bluetooth Stack Initialization or heap creation failed!! Exiting App...");
    exit (EXIT_FAILURE);
  }
}

/******************************************************************************
 * Function Name: APPLICATION_START()
 *******************************************************************************
 * Summary:
 *           BT stack initialization function wrapper
 *
 * Parameters:
 *           None
 *
 * Return:
 *          None
 *
 ******************************************************************************/
void
APPLICATION_START (void)
{
  application_start ();
}

/******************************************************************************
 * Function Name: main()
 *******************************************************************************
 * Summary:
 *          Application entry function
 *
 * Parameters:
 *          int argc           : argument count
 *          char *argv[]        : list of arguments
 *
 * Return:
 *          Status code
 *
 ******************************************************************************/
int
main (int argc, char *argv[])
{
  int filename_len = 0;
  char fw_patch_file[MAX_PATH];
  char hci_port[MAX_PATH];
  char peer_ip_addr[IP_ADDR_LEN] = "000.000.000.000";
  uint32_t hci_baudrate = 0;
  uint32_t patch_baudrate = 0;
  int btspy_inst = 0;
  uint8_t btspy_is_tcp_socket = 0;	/* Throughput calculation thread handler */
  pthread_t throughput_calc_thread_handle;	/* Audobaud configuration GPIO bank and pin */
  cybt_controller_autobaud_config_t autobaud;
  memset (fw_patch_file, 0, MAX_PATH);
  memset (hci_port, 0, MAX_PATH);
  int ip = 0;
  if (PARSE_ERROR ==
      arg_parser_get_args (argc, argv, hci_port, bt_device_address, &hci_baudrate,
			   &btspy_inst, peer_ip_addr, &btspy_is_tcp_socket,
			   fw_patch_file, &patch_baudrate, &autobaud))
    {
      return EXIT_FAILURE;
    }
  filename_len = strlen (argv[0]);
  if (filename_len >= MAX_PATH)
    {
      filename_len = MAX_PATH - 1;
    }

  cy_platform_bluetooth_init (fw_patch_file, hci_port, hci_baudrate,
			      patch_baudrate, &autobaud);

	if (fw_patch_file[0])
	{
		WICED_BT_TRACE ("Waiting for downloading patch...");
		wait_controller_reset_ready();
	}

  do
    {
        fprintf(stdout, "A2dP Sink Application\n");
        fprintf(stdout, "=============================================\n");
        fprintf(stdout, " Press 0, then <ENTER> to Exit \n");
        fprintf(stdout, "=============================================\n");
        ip = getchar();

      switch (ip)
	{
	case '0':
	  /* App exits */
	  break;
	default:
	  break;
	}
    }while (ip != '0');

   fprintf(stdout, "Exiting...\n");
   wiced_bt_delete_heap(p_default_heap);
  return EXIT_SUCCESS;
}
