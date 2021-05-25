/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *  dfu.c
 *
 * Device Firmware Upgrade via HCI
 *
 * The DFU application upgrades the device firmwarevia the HCI interface.
 * Similar to the OTA Firmware Upgrade, this is done in a fail safe manner.
 * If an unexpected reset or power loss occurs, then the upgrade is not completed
 * and the previous firmware remains in place.
 *
 * This application uses the same firmware update library as used by the
 * OTA Firmware Upgrade app.
 *
 * To demonstrate the app, work through the following steps.
 *
 */

/******************************************************************************
 *                                Includes
 ******************************************************************************/
#include "sparcommon.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_transport.h"
#include "wiced_platform.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_timer.h"
#include "wiced_bt_hci_firmware_upgrade.h"
#include "hci_control_api.h"
#include "ota_fw_upgrade.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#ifdef OTA_SECURE_FIRMWARE_UPGRADE
#include "bt_types.h"
#include "p_256_multprecision.h"
#include "p_256_ecc_pp.h"

// If secure version of the OTA firmware upgrade is used, the app should be linked with the ecdsa256_pub.c
// which exports the public key
extern Point    ecdsa256_public_key;
#endif

/******************************************************************************
 *                                Structures
 ******************************************************************************/
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];

/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
static void     hci_dfu_init(void);
wiced_result_t  hci_dfu_app_management_cback(wiced_bt_management_evt_t event,
                                wiced_bt_management_evt_data_t *p_event_data);
static void     hci_dfu_timeout(uint32_t count);
uint32_t        hci_dfu_handle_command( uint8_t *p_data, uint32_t length );
static void     hci_dfu_control_transport_status( wiced_transport_type_t type );
static void     hci_control_misc_handle_get_version( void );
static void     hci_dfu_read_config(void);
static void     hci_dfu_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
void            hci_dfu_status_callback(uint8_t status);

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
wiced_timer_t dfu_timer;                        /* Seconds timer instance */
#define TRANS_UART_BUFFER_SIZE          1024

const wiced_transport_cfg_t  transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
         //   .baud_rate =  HCI_UART_DEFAULT_BAUD,
            .baud_rate =  115200,
        },
    },
    .rx_buff_pool_cfg =
    {
        .buffer_size  = TRANS_UART_BUFFER_SIZE,
        .buffer_count = 1
    },
    .p_status_handler   = hci_dfu_control_transport_status,
    .p_data_handler     = hci_dfu_handle_command,
    .p_tx_complete_cback= NULL,
};
uint16_t        dfu_conn_id = 0;
wiced_transport_buffer_pool_t*  host_trans_pool;
int dfu_length = 0;
int dfu_erase_sector_size = OTA_FW_UPGRADE_CHUNK_SIZE_TO_COMMIT;

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 Function name:
 application_start

 Function Description:
 @brief    Starting point of your application

 @param void

 @return void
 */
APPLICATION_START( )
{

    wiced_transport_init( &transport_cfg );
    // create special pool for sending data to the MCU
    host_trans_pool = wiced_transport_create_buffer_pool(TRANS_UART_BUFFER_SIZE, 2);
#ifdef WICED_BT_TRACE_ENABLE
    /* Set to PUART to see traces on peripheral uart(puart) */
#if defined TEST_HCI_CONTROL || defined NO_PUART_SUPPORT
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#else
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
#ifdef CYW20706A2
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif
#endif
#endif
    WICED_BT_TRACE("DFU Sample Application\r\n");

    wiced_bt_stack_init(hci_dfu_app_management_cback,
                        &wiced_bt_cfg_settings,
                        wiced_bt_cfg_buf_pools);
}


/*
 Function name:
 hci_dfu_app_management_cback

 Function Description:
 @brief    Callback function that will be invoked by application_start()

 @param  event           Bluetooth management event type
 @param  p_event_data    Pointer to the the bluetooth management event data

 @return timer success/failure status of the callback function
 */
wiced_result_t
hci_dfu_app_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_SUCCESS;

    WICED_BT_TRACE("Received Event : %d\n\n\r", event);

    switch(event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        hci_dfu_init();
        break;

    default:
        WICED_BT_TRACE("Unknown Event\n");
        break;
    }

    return result;
}

/*
 * Application initialization occurring after the stack is enabled.
 */
void hci_dfu_init()
{
#ifdef WICED_BT_TRACE_ENABLE
    wiced_result_t status;

    WICED_BT_TRACE("Init dfu\n");
    /* Starting the app timer */
    wiced_init_timer(&dfu_timer, hci_dfu_timeout, 0, WICED_SECONDS_PERIODIC_TIMER);
    status = wiced_start_timer(&dfu_timer, 1);
    if (status != WICED_SUCCESS)
    {
        WICED_BT_TRACE("%s: wiced_start_timer failed, status:%d \n", __func__, status);
    }

    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace(hci_dfu_trace_callback);
#endif
    /* Firmware upgrade Initialization */
#ifdef OTA_SECURE_FIRMWARE_UPGRADE
    if (!wiced_hci_fw_upgrade_init(&ecdsa256_public_key, hci_dfu_status_callback, NULL))
#else
    if (!wiced_hci_fw_upgrade_init(NULL, hci_dfu_status_callback, NULL))
#endif
    {
        WICED_BT_TRACE("Firmware Upgrade Init failure !!! \n");
    }
}

/*
 * The function invoked on timeout of app seconds timer.
 */
void hci_dfu_timeout(uint32_t count)
{
    static uint32_t timer_count = 0;
    WICED_BT_TRACE("dfu tick: %d\n", timer_count++);
}

void hci_dfu_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    wiced_transport_send_hci_trace(host_trans_pool, type, length, p_data);
}

/*
 * This callback receives status updates from the firmware upgrade library.
 */
void hci_dfu_status_callback(uint8_t status)
{
    WICED_BT_TRACE("status callback %d\n", status);
    switch(status)
    {
        case HCI_FW_UPGRADE_STATUS_STARTED:
            // now prepared for download
            wiced_transport_send_data( HCI_CONTROL_DFU_EVENT_STARTED, NULL, 0 );
            break;
        case HCI_FW_UPGRADE_STATUS_ABORTED:
            // response now aborted
            wiced_transport_send_data( HCI_CONTROL_DFU_EVENT_ABORTED, NULL, 0 );
            break;
        case HCI_FW_UPGRADE_STATUS_COMPLETED:
            // operation completed
            wiced_deinit_timer(&dfu_timer);
            wiced_transport_send_data( HCI_CONTROL_DFU_EVENT_VERIFIED, NULL, 0 );
            break;
        case HCI_FW_UPGRADE_STATUS_VERIFICATION_START:
            // verification started
            wiced_transport_send_data( HCI_CONTROL_DFU_EVENT_VERIFICATION, NULL, 0 );
            break;
        default:
            WICED_BT_TRACE( "hci_dfu_status_callback unknown status %d\n", status );
            break;
    }
}

/*
 * hci_control_transport_status
 * This callback function is called when the MCU opens the Wiced UART
 */
static void hci_dfu_control_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE( "hci_dfu_control_transport_status %x \n", type );

    // Tell Host that App is started
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );
}

/*
 * Handle command received over the UART.  First buffer of the command is the opcode
 * of the operation.  Rest are parameters specific for particular command.
 *
 */
uint32_t hci_dfu_handle_command( uint8_t *p_data, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t* p_rx_buf = p_data;
    uint32_t rc = 0;

    if ( !p_rx_buf )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    //Expected minimum 4 byte as the wiced header
    if( length < 4 )
    {
        WICED_BT_TRACE("invalid params\n");
        wiced_transport_free_buffer( p_rx_buf );
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    (void) payload_len; // makes compiler happy
    opcode = p_data[0] + ( p_data[1] << 8 );     // Get opcode
    payload_len = p_data[2] + ( p_data[3] << 8 );     // Get len
    p_data += 4;
    length -= 4;

    WICED_BT_TRACE("Cmd:%04X\n", opcode);

    switch ( opcode )
    {
    case HCI_CONTROL_DFU_COMMAND_READ_CONFIG:
        WICED_BT_TRACE("Read config\n");
        hci_dfu_read_config();
        break;

    case HCI_CONTROL_DFU_COMMAND_WRITE_COMMAND:
        WICED_BT_TRACE("Write command: %d %d len %d\n", *p_data, *(p_data+1), length);
        if (!hci_fw_upgrade_handle_command(dfu_conn_id, p_data, length))
        {
            if(*p_data != WICED_HCI_UPGRADE_COMMAND_ABORT)
            {
                WICED_BT_TRACE("hci_handle_command failed.\n");
                rc = HCI_CONTROL_STATUS_FAILED;
            }
        }
        break;

    case HCI_CONTROL_DFU_COMMAND_WRITE_DATA:
        WICED_BT_TRACE("Write data %B\n", p_data);
        if (!hci_fw_upgrade_handle_data(dfu_conn_id, p_data, length))
        {
            WICED_BT_TRACE("hci_handle_data failed.\n");
            rc = HCI_CONTROL_STATUS_FAILED;
            break;
        }
        WICED_BT_TRACE("Send data event to HCI\n");
        wiced_transport_send_data( HCI_CONTROL_DFU_EVENT_DATA, NULL, 0 );
        break;

    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
        WICED_BT_TRACE( "Get Version:\n");
        hci_control_misc_handle_get_version();
        break;

    default:
        WICED_BT_TRACE( "Ignored\n");
        break;
    }

    //Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_rx_buf );
    return rc;
}

/*
 * Handle response to read config command received over the UART.
 * Respond with desired block size. The HCI central should set all commands
 * to write download data to this size. This provides for efficient
 * data handling.
 */
void hci_dfu_read_config(void)
{
    uint8_t   tx_buf[12];
    uint8_t   cmd = 0;

    tx_buf[cmd++] = dfu_erase_sector_size & 0xff;
    tx_buf[cmd++] = (dfu_erase_sector_size >> 8) & 0xff;
    tx_buf[cmd++] = (dfu_erase_sector_size >> 16) & 0xff;
    tx_buf[cmd++] = (dfu_erase_sector_size >> 24) & 0xff;
    wiced_transport_send_data( HCI_CONTROL_DFU_EVENT_CONFIG, tx_buf, cmd );
}

/* Handle get version command */
void hci_control_misc_handle_get_version( void )
{
    uint8_t   tx_buf[15];
    uint8_t   cmd = 0;

// If this is 20819 or 20820, we do detect the device from hardware
#define RADIO_ID    0x006007c0
#define RADIO_20820 0x80
#define CHIP_20820  20820
#define CHIP_20819  20819
#if (CHIP==CHIP_20819) || (CHIP==CHIP_20820)
    uint32_t chip = CHIP_20819;
    if (*(UINT32*) RADIO_ID & RADIO_20820)
    {
        chip = CHIP_20820;
    }
#else
    uint32_t  chip = CHIP;
#endif

    tx_buf[cmd++] = WICED_SDK_MAJOR_VER;
    tx_buf[cmd++] = WICED_SDK_MINOR_VER;
    tx_buf[cmd++] = WICED_SDK_REV_NUMBER;
    tx_buf[cmd++] = WICED_SDK_BUILD_NUMBER & 0xFF;
    tx_buf[cmd++] = (WICED_SDK_BUILD_NUMBER>>8) & 0xFF;
    tx_buf[cmd++] = chip & 0xFF;
    tx_buf[cmd++] = (chip>>8) & 0xFF;
    tx_buf[cmd++] = (chip>>24) & 0xFF;
    tx_buf[cmd++] = 0; // not used

    /* Send MCU app the supported features */
    tx_buf[cmd++] = HCI_CONTROL_GROUP_DFU;

    wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd );
}
