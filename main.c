/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */
//#define STEP_1 1
//#define STEP_2 2
//#define STEP_3 3
//#define STEP_4 4
//#define STEP_5 5
//#define STEP_6 6

#include <stdint.h>
#include <string.h>

#include "nrf_power.h"
#include "nrf_bootloader_info.h"
#include "ble_dfu.h"
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"

#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "ble_hrs.h"
//#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_queue.h"
#include "nrf_drv_clock.h"
#include "app_sdcard.h"

//#if defined (UART_PRESENT)
//#include "nrf_uart.h"
//#endif
//#if defined (UARTE_PRESENT)
//#include "nrf_uarte.h"
//#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_twi.h"
#include "nrf_drv_saadc.h"
//#include "nrf_drv_ppi.h"
//#include "nrf_drv_timer.h"

#include "app_mpu.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"

#include "ble_bas.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "TJUBMFE-ECG"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                1024                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                0                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                1024                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                1024                                         /**< UART RX buffer size. */

#define SDC_SCK_PIN     15  ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN    18  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN    8  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN      1  ///< SDC chip select (CS) pin.

#define FILE_NAME   "heart.csv"
#define TEST_STRING "SD card example."

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
BLE_BAS_DEF(m_bas);                                                 /**< Structure used to identify the battery service. */
BLE_HRS_DEF(m_hrs);                                                 /**< Heart rate service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

APP_TIMER_DEF(adc_timer);
APP_TIMER_DEF(batt_timer);
//static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);

//#define SAMPLES_IN_BUFFER 12
//static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];

//static nrf_ppi_channel_t     m_ppi_channel;

#define QUEUE_SIZE 1200

NRF_QUEUE_DEF(int16_t, m_ecg_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(int16_t, m_accx_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(int16_t, m_accy_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(int16_t, m_accz_queue, QUEUE_SIZE, NRF_QUEUE_MODE_OVERFLOW);

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

static uint8_t poor_signal = 0, heart_rate = 0;
static bool is_connected = 0;
static FIL file;

static void batt_timer_handler(void * p_context)
{
    ret_code_t err_code;
		nrf_saadc_value_t ssaadc_val;
	
		nrf_drv_saadc_sample_convert(1,&ssaadc_val);
		
		uint8_t battery_level = (ssaadc_val - 4000) / 800;

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
		
}

static void tf_disk_init(){

    static FATFS fs;
//    static DIR dir;
//    static FILINFO fno;
    
    FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;

    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    NRF_LOG_INFO("Initializing disk 0 (SDC)...");
    for (uint32_t retries = 300; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        NRF_LOG_INFO("Disk initialization failed.");
        return;
    }

    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_INFO("Capacity: %d MB", capacity);

    NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed.");
        return;
    }

//    NRF_LOG_INFO("\r\n Listing directory: /");
//    ff_result = f_opendir(&dir, "/");
//    if (ff_result)
//    {
//        NRF_LOG_INFO("Directory listing failed!");
//        return;
//    }

//    do
//    {
//        ff_result = f_readdir(&dir, &fno);
//        if (ff_result != FR_OK)
//        {
//            NRF_LOG_INFO("Directory read failed.");
//            return;
//        }

//        if (fno.fname[0])
//        {
//            if (fno.fattrib & AM_DIR)
//            {
//                NRF_LOG_RAW_INFO("   <DIR>   %s",(uint32_t)fno.fname);
//            }
//            else
//            {
//                NRF_LOG_RAW_INFO("%9lu  %s", fno.fsize, (uint32_t)fno.fname);
//            }
//        }
//    }
//    while (fno.fname[0]);
//    NRF_LOG_RAW_INFO("");

}

static void tf_write_string(char* str, size_t size)
{
		FRESULT ff_result;
    uint32_t bytes_written;
	
    //NRF_LOG_INFO("Writing to file " FILE_NAME "...");

    ff_result = f_write(&file, str, size - 1, (UINT *) &bytes_written);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Write failed\r\n.");
    }
    else
    {
        //NRF_LOG_INFO("%d bytes written.", bytes_written);
    }

    
    return;
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{

}


static void saadc_init(void)
{
    ret_code_t err_code;
	
    //nrf_saadc_channel_config_t channel_0_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);
		
	nrf_saadc_channel_config_t channel_0_config =
	{                                                                    \
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,                       \
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,                       \
    .gain       = NRF_SAADC_GAIN1_3,                                 \
    .reference  = NRF_SAADC_REFERENCE_INTERNAL,                      \
    .acq_time   = NRF_SAADC_ACQTIME_10US,                            \
    .mode       = NRF_SAADC_MODE_DIFFERENTIAL,                       \
    .pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN4),                        \
    .pin_n      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN5)                         \
};
	
		nrf_saadc_channel_config_t channel_1_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);
	
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    APP_ERROR_CHECK(err_code);

		err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
    APP_ERROR_CHECK(err_code);

//    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
//    APP_ERROR_CHECK(err_code);

//    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
//    APP_ERROR_CHECK(err_code);

}

static void adc_timer_handler(void * p_context)
{
	
	
	nrf_saadc_value_t ssaadc_val;
	
	nrf_drv_saadc_sample_convert(0,&ssaadc_val);
	
	//NRF_LOG_INFO("%d",ssaadc_val);
	
	nrf_queue_push(&m_ecg_queue,&ssaadc_val);
	
	accel_values_t acc_values;
  app_mpu_read_accel(&acc_values);
	nrf_queue_push(&m_accx_queue,&acc_values.x);
	nrf_queue_push(&m_accy_queue,&acc_values.y);
	nrf_queue_push(&m_accz_queue,&acc_values.z);
	

}

static void mpu_init(void)
{
    ret_code_t ret_code;
    // Initiate MPU driver
    ret_code = app_mpu_init();
    APP_ERROR_CHECK(ret_code); // Check for errors in return value
	  ret_code = app_mpu_accel_only_mode();
    APP_ERROR_CHECK(ret_code); // Check for errors in return value
	
    // Setup and configure the MPU with intial values
    app_mpu_config_t p_mpu_config = MPU_DEFAULT_CONFIG(); // Load default values
    p_mpu_config.smplrt_div = 0;   // Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 19 gives a sample rate of 50Hz
    p_mpu_config.accel_config.afs_sel = AFS_8G; // Set accelerometer full scale range to 2G
	
	
    ret_code = app_mpu_config(&p_mpu_config); // Configure the MPU with above values
    APP_ERROR_CHECK(ret_code); // Check for errors in return value 
	
////		// Enable magnetometer
////		app_mpu_magn_config_t magnetometer_config;
////		magnetometer_config.mode = CONTINUOUS_MEASUREMENT_100Hz_MODE;
////		magnetometer_config.resolution = 0;
////    ret_code = app_mpu_magnetometer_init(&magnetometer_config);
////    APP_ERROR_CHECK(ret_code); // Check for errors in return valuecd
	
}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
	  err_code = app_timer_create(&adc_timer, APP_TIMER_MODE_REPEATED, adc_timer_handler);
    APP_ERROR_CHECK(err_code); 
		err_code = app_timer_create(&batt_timer, APP_TIMER_MODE_REPEATED, batt_timer_handler);
     APP_ERROR_CHECK(err_code); 
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
//        uint32_t err_code;

//        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
//        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

//        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
//        {
//            do
//            {
//                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
//                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
//                {
//                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
//                    APP_ERROR_CHECK(err_code);
//                }
//            } while (err_code == NRF_ERROR_BUSY);
//        }
        //if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        //{
        //    while (app_uart_put('\n') == NRF_ERROR_BUSY);
        //}
    }

}
/**@snippet [Handling the data received over BLE] */
/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);


static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};

static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}


static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            // Prevent device from advertising on disconnect.
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
		ble_bas_init_t    	bas_init;
	  ble_hrs_init_t     hrs_init;
    nrf_ble_qwr_init_t qwr_init = {0};
		ble_dfu_buttonless_init_t dfus_init = {0};
		uint8_t            body_sensor_location;
    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
		
		dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
		
		/* YOUR_JOB: Add code to initialize the services used by the application.*/
       // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_CHEST;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    hrs_init.hrm_cccd_wr_sec = SEC_OPEN;
    hrs_init.bsl_rd_sec      = SEC_OPEN;

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

		
    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
//	
//		nrf_gpio_pin_clear(ECG_CS_PIN);
//    app_uart_close();

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}



///**@brief   Function for handling app_uart events.
// *
// * @details This function will receive a single character from the app_uart module and append it to
// *          a string. The string will be be sent over BLE when the last character received was a
// *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
// */
///**@snippet [Handling the data received over UART] */

//int Parse_Payload( unsigned char *payload, unsigned char pLength ) {
//    unsigned char bytesParsed = 0;
//    unsigned char code;
//    unsigned char length;
//    unsigned char extendedCodeLevel;
//		ret_code_t err_code;
//		
//		int16_t ECGTemp;
//		accel_values_t acc_values;
//	
//    /* Loop until all bytes are parsed from the payload[] array... */
//    bool sensor_contact_detected = false;
//		while( bytesParsed < pLength ) {
//        /* Parse the extendedCodeLevel, code, and length */
//        extendedCodeLevel = 0;
//        while( payload[bytesParsed] == 0x55 ) {
//            extendedCodeLevel++;
//            bytesParsed++;
//        }
//        code = payload[bytesParsed++];
//        if( code & 0x80 ) length = payload[bytesParsed++];
//        else length = 1;
//				
//				//NRF_LOG_INFO("CODE:0x%02X,Payload:0x%02X,0x%02X",code,payload[bytesParsed],payload[bytesParsed + 1]);
//				
//				switch(code){
//					case 0x02:
//						poor_signal = payload[bytesParsed + 1];
//						NRF_LOG_INFO("POOR_SIGNAL Quality:%03d",poor_signal);
//            if(poor_signal > 100)
//							sensor_contact_detected = true;
//						else
//							sensor_contact_detected = false;
//					if(is_connected)
//						ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);

//					break;
//					case 0x03:
//						heart_rate = payload[bytesParsed + 1];
//						NRF_LOG_INFO("HEART_RATE Value:%03d",heart_rate);
//						if(is_connected){
//							err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
//								if ((err_code != NRF_SUCCESS) &&
//										(err_code != NRF_ERROR_INVALID_STATE) &&
//										(err_code != NRF_ERROR_RESOURCES) &&
//										(err_code != NRF_ERROR_BUSY) &&
//										(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//									 )
//								{
//										APP_ERROR_HANDLER(err_code);
//								}
//						}
//						
//					break;
//					
//					case 0x80:
//						ECGTemp = payload[bytesParsed] << 8 | payload[bytesParsed + 1];
//						nrf_queue_push(&m_ecg_queue,&ECGTemp);
//					
//						app_mpu_read_accel(&acc_values);
//						nrf_queue_push(&m_accx_queue,&acc_values.x);
//						nrf_queue_push(&m_accy_queue,&acc_values.y);
//						nrf_queue_push(&m_accz_queue,&acc_values.z);
//					break;
//					default:
//						break;
//				
//				}
//				
//        bytesParsed += length;
//    }
//    return( 0 );
//}

//void parseECG(uint8_t recv_char){
//	

//static uint8_t STEP = STEP_1;
//static uint8_t offset_t = 0;
//static uint8_t plength = 0;
//static uint8_t recv_payload[170];
//int checksum = 0;
//int i;
//    switch(STEP){

//        case STEP_1: //SYNC-1
//						//NRF_LOG_INFO("SYNC-1,%x",recv_char);
//            if(recv_char == 0xaa)
//                STEP = STEP_2;
//            return;
//        case STEP_2: //SYNC-2
//						//NRF_LOG_INFO("SYNC-2,%x",recv_char);
//            if(recv_char == 0xaa){
//                STEP = STEP_3;
//            } else {
//                STEP = STEP_1;
//							NRF_LOG_INFO("BMD101 Parse Error");
//            }
//            return;
//        case STEP_3: //PLENGTH
//					//NRF_LOG_INFO("PLENGTH,%x",recv_char);
//            if(recv_char == 0xaa){
//                STEP = STEP_3;
//            } else if (recv_char > 170){
//                STEP = STEP_1;
//							NRF_LOG_INFO("BMD101 Parse Error(Too many chars)");
//            } else {
//                plength = recv_char;
//                memset(recv_payload,0,sizeof(recv_payload));
//                offset_t = 0;
//                STEP = STEP_4;
//            }
//            return;
//        case STEP_4: //Recv PLENGTH
//					//NRF_LOG_INFO("Recv PLENGTH,%x",recv_char);
//						recv_payload[offset_t] = recv_char;
//						offset_t++;
//            if( offset_t == plength ){
//                STEP = STEP_5;
//            }
//            return;
//        case STEP_5: //Checksum
//					//NRF_LOG_INFO("Checksum,%x",recv_char);
//						checksum = 0;
//            for(i=0;i<plength;i++){
//                checksum += recv_payload[i];
//            }
//            checksum &= 0xff;
//            checksum = ~checksum & 0xff;
//            if(checksum == recv_char){
//              Parse_Payload(recv_payload,plength);  
//							STEP = STEP_1;
//            }else{
//                STEP = STEP_1;
//							NRF_LOG_INFO("BMD101 Parse Error(Checksum)");
//            }
//            return;
//    }
//}


//void uart_event_handle(app_uart_evt_t * p_event)
//{
//    uint8_t receive_char;
//    switch (p_event->evt_type)
//    {
//        case APP_UART_DATA_READY:
//        
//            app_uart_get(&receive_char);
//            //byte_queue_push(&receive_char);
//				//nrf_queue_push(&m_byte_queue, &receive_char);
//				parseECG(receive_char);
//            break;

//        case APP_UART_COMMUNICATION_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_communication);
//            break;

//        case APP_UART_FIFO_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_code);
//            break;

//        default:
//            break;
//    }
//}
///**@snippet [Handling the data received over UART] */



///**@brief  Function for initializing the UART module.
// */
///**@snippet [UART Initialization] */
//static void uart_init(void)
//{
//    uint32_t                     err_code;
//    app_uart_comm_params_t const comm_params =
//    {
//        .rx_pin_no    = RX_PIN_NUMBER,
//        .tx_pin_no    = TX_PIN_NUMBER,
//        .rts_pin_no   = RTS_PIN_NUMBER,
//        .cts_pin_no   = CTS_PIN_NUMBER,
//        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
//        .use_parity   = false,
//#if defined (UART_PRESENT)
//        .baud_rate    = NRF_UART_BAUDRATE_57600
//#else
//        .baud_rate    = NRF_UARTE_BAUDRATE_57600
//#endif
//    };

//    APP_UART_FIFO_INIT(&comm_params,
//                       UART_RX_BUF_SIZE,
//                       UART_TX_BUF_SIZE,
//                       uart_event_handle,
//                       APP_IRQ_PRIORITY_LOWEST,
//                       err_code);
//    APP_ERROR_CHECK(err_code);
//}
/**@snippet [UART Initialization] */

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
						
            NRF_LOG_INFO("Connected");
						is_connected = 1;
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            
            NRF_LOG_INFO("Disconnected");
						is_connected = 0;
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}




/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.*/
       uint32_t err_code;
       err_code = app_timer_start(adc_timer, APP_TIMER_TICKS(4), NULL);
       APP_ERROR_CHECK(err_code);
			err_code = app_timer_start(batt_timer, APP_TIMER_TICKS(1000), NULL);
       APP_ERROR_CHECK(err_code);
}



/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;
		NRF_POWER->DCDCEN = 1;
    // Initialize.
		nrf_drv_clock_init();
		//nrf_gpio_cfg_output(ECG_CS_PIN);
		//nrf_gpio_pin_clear(ECG_CS_PIN);
		saadc_init();

	
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
		
    // Start execution.
    advertising_start();
	
		tf_disk_init();
		
		tf_write_string(TEST_STRING,sizeof(TEST_STRING));
		
		
		//nrf_gpio_pin_set(ECG_CS_PIN);
		mpu_init();
		
		application_timers_start();
		//uart_init();
		
		static int offset = 0;
		static int16_t heheh[120];
		static char tf_str[50];
		static char write_str[1500];
		memset(write_str,0,1500);
		//nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    // Enter main loop.
		FRESULT ff_result;
		ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
						if (ff_result != FR_OK)
						{
								NRF_LOG_INFO("Unable to open or create file: " FILE_NAME ".");
								
						}
    for (;;)
    {
			
			
			
					ret_code_t ret = nrf_queue_pop(&m_ecg_queue,&heheh[offset*4]);
					if(ret == NRF_SUCCESS){
						nrf_queue_pop(&m_accx_queue,&heheh[offset*4+1]);
						nrf_queue_pop(&m_accy_queue,&heheh[offset*4+2]);
						nrf_queue_pop(&m_accz_queue,&heheh[offset*4+3]);
						
						//ecg,accx,accy,accz,heart,contact
						
						memset(tf_str,0,50);
						size_t llength = sprintf(tf_str,"%d,%d,%d,%d,%d,%d,\r\n",
																					heheh[offset*4],
																					heheh[offset*4+1],
																					heheh[offset*4+2],
																					heheh[offset*4+3],
																					heart_rate,poor_signal);
						tf_write_string(tf_str,llength);
						strcat(write_str,tf_str);
					
						
						offset++;
					}
					
					
						
					
					 
					if(offset == 30){
						
						
						//tf_write_string(write_str,strlen(write_str));
						memset(write_str,0,1500);
						
						if(is_connected){			
						uint16_t llength = 240;
						ble_nus_data_send(&m_nus, (uint8_t*)heheh, &llength, m_conn_handle);
						}
						offset = 0;
						(void) f_close(&file);
						ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
						if (ff_result != FR_OK)
						{
								NRF_LOG_INFO("Unable to open or create file: " FILE_NAME ".");
								
						}
									
					}
			
			
			
      idle_state_handle();

    }
}


/**
 * @}
 */
