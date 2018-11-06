/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>

#include "esp_system.h"
#include "esp_types.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "brzo_i2c.h"
#include "hdc1080.h"
#include "dht.h"


#define HDC1080_DEF_INTERVAL_MS 10000

/*
 * [BH1750] Mode command - HiRes2
 * Update measure delay if you change mode
 */
#define LIGHT_MEASURE_MODE 0x21
/*
 * [BH1750] Measure timeout in ms for selected mode
 */
#define LIGHT_MEASURE_DELAY_MS 120
/*
 * [I2C] Address light sensor GY-302/BH1750
 */
#define I2C_ADDRESS_BH1750 0x23
/*
 * [I2C] Bus speed in kbps for light sensor GY-302/BH1750
 */
#define I2C_SPEED_BH1750 100

h_brzo_i2c_bus volatile i2c_bus_1;

h_hdc1080 volatile hdc1080_1 = NULL;

void hdc1080_task(void *params)
{
	union
	{
		uint8_t b[4];
		uint16_t w[2];
		uint32_t raw;
	} buff;
	float t, h, light;
	int dt, dh;
	char str[32];
	uint8_t result;

	for (;;)
	{
		vTaskDelay(HDC1080_DEF_INTERVAL_MS / portTICK_PERIOD_MS);
		if (hdc1080_get_raw(hdc1080_1, &(buff.raw)))
		{
			ESP_LOGD(
					__FUNCTION__,
					"[HDC1080] Raw data read: 0x%02x 0x%02x 0x%02x 0x%02x",
					buff.b[0],
					buff.b[1],
					buff.b[2],
					buff.b[3]);
			/*
			 * Temperature value
			 * Care about byte order!!!
			 */
			t = (float) ((buff.b[0] << 8) | buff.b[1]) * 165 / 65536 - 40;

			/*
			 * Humidity value
			 * Care about byte order!!!
			 */
			h = (float) ((buff.b[2] << 8) | buff.b[3]) * 100 / 65536;

			sprintf(str, "t = %.2f\th = %.2f", t, h);
			ESP_LOGI(__FUNCTION__, "[HDC1080] Sensor read:\t%s", str);
		}
		else
		{
			ESP_LOGE(__FUNCTION__, "[HDC1080] Sensor read raw error");
		}

		/*
		 * [DHT22] Get measurement values
		 */
		if (dht_read(&dt, &dh) == 0)
		{
			/*
			 * Temperature value
			 */
			t = (float) dt / 10;

			/*
			 * Humidity value
			 */
			h = (float) dh / 10;

			sprintf(str, "t = %.2f\th = %.2f", t, h);
			ESP_LOGI(__FUNCTION__, "[DHT22] Sensor read:\t%s", str);
		}
		else
		{
			ESP_LOGE(__FUNCTION__, "[DHT22] Sensor read error");
		}

		/*
		 * [BH1750] Power on sensor
		 */

		/*
		 * Start I2C transaction
		 */
		taskENTER_CRITICAL();
		brzo_i2c_start_transaction(
				i2c_bus_1,
				I2C_ADDRESS_BH1750,
				I2C_SPEED_BH1750);
		brzo_i2c_ack_polling(i2c_bus_1, 100);

		/*
		 * Power on command
		 */
		buff.b[0] = 0x01;
		brzo_i2c_write(i2c_bus_1, buff.b, 1, false);
		brzo_i2c_ack_polling(i2c_bus_1, 100);
		/*
		 * Stop I2C transaction
		 */
		result = brzo_i2c_end_transaction(i2c_bus_1);
		/*
		 * Restore interrupts after transaction
		 */
		taskEXIT_CRITICAL();

		if (result == 0)
		{
			/*
			 * Sleep 100 us for device up
			 */
			vTaskDelay (100);

			ESP_LOGD(__FUNCTION__, "[BH1750] Sensor woken up, send measure command");

			/*
			 * [BH1750] Set mode
			 */
			/*
			 * Start I2C transaction
			 */
			taskENTER_CRITICAL();
			brzo_i2c_start_transaction(
					i2c_bus_1,
					I2C_ADDRESS_BH1750,
					I2C_SPEED_BH1750);
			brzo_i2c_ack_polling(i2c_bus_1, 100);

			/*
			 * Mode comand
			 */
			buff.b[0] = LIGHT_MEASURE_MODE;
			brzo_i2c_write(i2c_bus_1, buff.b, 1, false);
			brzo_i2c_ack_polling(i2c_bus_1, 100);

			/*
			 * Stop I2C transaction
			 */
			result = brzo_i2c_end_transaction(i2c_bus_1);

			/*
			 * Restore interrupts after transaction
			 */
			taskEXIT_CRITICAL();

			if (result == 0)
			{
				/*
				 * Wait for measure delay time
				 */
				vTaskDelay (LIGHT_MEASURE_DELAY_MS/portTICK_RATE_MS);

				ESP_LOGD(__FUNCTION__, "[BH1750] Measure delay done, read light value");

				/*
				 * [BH1750] Set mode
				 */

				/*
				 * Start I2C transaction
				 */
				taskENTER_CRITICAL();
				brzo_i2c_start_transaction(
						i2c_bus_1,
						I2C_ADDRESS_BH1750,
						I2C_SPEED_BH1750);
				brzo_i2c_ack_polling(i2c_bus_1, 100);

				/*
				 * Read value
				 */
				brzo_i2c_read(i2c_bus_1, buff.b, 2, false);

				/*
				 * Stop I2C transaction
				 */
				result = brzo_i2c_end_transaction(i2c_bus_1);

				/*
				 * Restore interrupts after transaction
				 */
				taskEXIT_CRITICAL();

				if (result == 0)
				{
					light = (buff.b[0] * 256 + buff.b[1]) / 1.2;
					sprintf(str, "%.02f", light);
					ESP_LOGI(__FUNCTION__, "[BH1750] Sensor read = %s lux", str);
				}
				else
				{
					ESP_LOGE(__FUNCTION__, "[BH1750] Sensor read error 0x%02x", result);
				}
			}
			else
			{
				ESP_LOGE(__FUNCTION__, "[BH1750] Send command error 0x%02x", result);
			}
		}
		else
		{
			ESP_LOGE(__FUNCTION__, "[BH1750] Sensor wake up error 0x%02x", result);
		}
	}
}

/******************************************************************************
 * FunctionName : app_main
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void app_main(void)
{
 	/*
	 * Disable WiFi
	 */
	esp_wifi_stop();
	esp_wifi_deinit();

//	/*
//	 * Enable debug output to UART1
//	 */
//	uart_config_t uart_config;
//	uart_config.baud_rate = 74880;
//	uart_config.data_bits = UART_DATA_8_BITS;
//	uart_config.parity = UART_PARITY_DISABLE;
//	uart_config.stop_bits = UART_STOP_BITS_1;
//	uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
//	uart_config.rx_flow_ctrl_thresh = 120;
//	ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

	ESP_LOGI(__FUNCTION__, "=== Firmware SDK version: %s, CPU clock: %d", esp_get_idf_version(), system_get_cpu_freq());
	/*
	 * [I2C] Init bus instance (SDA = GPIO4, SCL = GPIO5)
	 */
	if ((i2c_bus_1 = brzo_i2c_setup(PERIPHS_IO_MUX_GPIO4_U, 4, FUNC_GPIO4,
			PERIPHS_IO_MUX_GPIO5_U, 5, FUNC_GPIO5, 1000)) != NULL)
	{
		uint16_t i2c_address;
		ESP_LOGI(__FUNCTION__, "[I2C] Start scanning bus");
		for (i2c_address = 0; i2c_address < 256; i2c_address++)
		{
			brzo_i2c_start_transaction(i2c_bus_1, (uint8_t) i2c_address, 400);
			brzo_i2c_ack_polling(i2c_bus_1, 500);
			if (brzo_i2c_end_transaction(i2c_bus_1) == 0)
				ESP_LOGI(__FUNCTION__, "[I2C] Device present: 0x%02x", i2c_address);
		}
		ESP_LOGI(__FUNCTION__, "[I2C] Scanning bus completed");
		/*
		 * [HDC1080] Init sensor instance (default parameters)
		 */
		ESP_LOGI(__FUNCTION__, "[HDC1080] Initializing sensor");
		if ((hdc1080_1 = hdc1080_setup(i2c_bus_1, HDC1080_I2C_DEF_ADDRESS,
				HDC1080_I2C_DEF_SPEED, HDC1080_I2C_DEF_ACK_TIMEOUT)) != NULL)
		{
			/*
			 * [HDC1080] Get base sensor properties
			 */
			ESP_LOGI(__FUNCTION__, "[HDC1080] Manufacturer 0x%04x",
					hdc1080_get_manufacturer(hdc1080_1));
			ESP_LOGI(__FUNCTION__, "[HDC1080] Device ID 0x%04x",
					hdc1080_get_device(hdc1080_1));
			/*
			 * TODO: Resolve int64_t format vatiable
			 */
//			ESP_LOGI(__FUNCTION__, "[HDC1080] Serial 0x%010x\n",
//					hdc1080_get_serial(hdc1080_1));
		}

		/*
		 * [HDC1080] Buffer for operations with registers
		 */
		uint16_t buffer;

		/*
		 * [HDC1080] Get current status
		 */
		if (hdc1080_get_status(hdc1080_1, &buffer))
			ESP_LOGI(__FUNCTION__, "[HDC1080] Current status: 0x%04x", buffer);

		/*
		 * Initialize HDC1080 sensor with configuration:
		 * -- acquisition mode = temperature+humidity (bit 12 = 1)
		 * -- 14 bit temperature resolution (bit 10 = 0)
		 * -- 14 bit humidity resolution (bits 9:8 = 00)
		 * -- other bits by default as described in documentation
		 */
		buffer = HDC1080_REG_CONFIG_WRITE_MASK & HDC1080_CONFIG_MODE_ALL;

		/*
		 * [HDC1080] Write new configuration
		 */
		if (hdc1080_set_config(hdc1080_1, buffer))
			ESP_LOGI(__FUNCTION__, "[HDC1080] Configuration updated, "
					"waiting 20 mS for sensor start");

		/*
		 * [HDC1080] Sleep 20 ms for sensor boot properly
		 */
		vTaskDelay(20 / portTICK_PERIOD_MS);

		/*
		 * [HDC1080] Check updated status
		 */
		if (hdc1080_get_status(hdc1080_1, &buffer))
		{
			ESP_LOGI(__FUNCTION__, "[HDC1080] New status: 0x%04x", buffer);
			ESP_LOGI(__FUNCTION__, "[HDC1080] Sensor initialized");

		}
//    	else
//    		is_hdc1080_present = false;
	}

	/*
	 * [DHT22] Init sensor
	 */
	dht_init();

	/*
	 * Start HDC1080 periodical task
	 */
	ESP_LOGI(__FUNCTION__, "[HDC1080] Starting HDC1080 task");
	static uint8_t ucParameterToPass;
	TaskHandle_t xHandle = NULL;

	xTaskCreate(
			hdc1080_task,
			(char *) "HDC1080_Task",
			2048,
			&ucParameterToPass,
			tskIDLE_PRIORITY,
			&xHandle);
	if( xHandle == NULL )
	{
		ESP_LOGI(__FUNCTION__, "Error when starting HDC1080 task");
	}
}
