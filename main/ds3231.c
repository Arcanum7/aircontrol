/* Driver for DS3231 high precision RTC module
 *
 * Part of esp-open-rtos
 * Copyright (C) 2015 Richard A Burton <richardaburton@gmail.com>
 * Copyright (C) 2016 Bhuvanchandra DV <bhuvanchandra.dv@gmail.com>
 * MIT Licensed as described in the file LICENSE
*/

/**
 * Driver for DS3231 high precision RTC module
 * Source adapted to ESP8266_RTOS_SDK v3.0
 * Used modified brzo_i2c driver library to work with I2C bus
 *
 * Copyright (C) 2018 Michael Kolomiets <michael.kolomiets@gmail.com>
 *
 * Reference documentation
 * https://datasheets.maximintegrated.com/en/ds/DS3231.pdf
 *
 * Original license in the file LICENSE.ds3231
 */

#include "ds3231.h"

#include "esp_system.h"
#include "esp_attr.h"
#include "esp_libc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "brzo_i2c.h"

/*
 * Structure that describes HDC1080 device instance
 */
typedef struct
{
	h_brzo_i2c_bus i2c_bus;
	uint8_t i2c_address;
	uint16_t i2c_frequency;
	uint16_t i2c_ack_timeout;
} ds3231_device_t;

/*
 * Forward function declarations
 */

/*
 * Convert binary coded decimal to normal decimal
 */
uint8_t bcd_to_dec(uint8_t bcd);

/*
 * Convert normal decimal to binary coded decimal
 */
uint8_t dec_to_bcd(uint8_t dec);


h_ds3231 ds3231_setup(h_brzo_i2c_bus i2c_bus,
		uint8_t i2c_address, uint16_t i2c_frequency, uint16_t i2c_ack_timeout)
{
	ds3231_device_t *device = NULL;

	if ((device = os_malloc(sizeof(ds3231_device_t))) != NULL)
	{
		device->i2c_bus = i2c_bus;
		device->i2c_address = i2c_address;
		device->i2c_frequency = i2c_frequency;
		device->i2c_ack_timeout = i2c_ack_timeout;
	}

	return NULL;
}

void ds3231_free(h_ds3231 device)
{
	if (device == NULL)
		return;

	ds3231_device_t *d = (ds3231_device_t *) device;

	os_free(d);
}

bool ds3231_get_raw(h_ds3231 device, uint8_t reg, uint8_t * p_buff, uint8_t length)
{
	bool result = false;

	if (device == NULL)
		return false;

	ds3231_device_t *d = (ds3231_device_t *) device;

	/*
	 * Disable interrupts before transaction
	 */
	taskENTER_CRITICAL();

	/*
	 * Start I2C transaction
	 */
	brzo_i2c_start_transaction(d->i2c_bus, d->i2c_address, d->i2c_frequency);
	brzo_i2c_ack_polling(d->i2c_bus, d->i2c_ack_timeout);

	/*
	 * Send register address
	 */
	brzo_i2c_write(d->i2c_bus, &reg, 1, false);

	/*
	 * End transaction
	 */
	if (brzo_i2c_end_transaction(d->i2c_bus) == 0)
	{
		/*
		 * Start I2C transaction
		 */
		brzo_i2c_start_transaction(d->i2c_bus, d->i2c_address,
				d->i2c_frequency);
		brzo_i2c_ack_polling(d->i2c_bus, d->i2c_ack_timeout);

		/*
		 * Receive register data first byte (MSB)
		 */
		brzo_i2c_read(d->i2c_bus, p_buff, length, false);

		/*
		 * End transaction
		 */
		if (brzo_i2c_end_transaction(d->i2c_bus) == 0)
		{
			result = true;
		}
	}
	/*
	 * Restore interrupts after transaction
	 */
	taskEXIT_CRITICAL();

	return result;
}

bool ds3231_set_raw(h_ds3231 device, uint8_t reg, uint8_t * p_buff, uint8_t length)
{
	bool result = true;

	if (device == NULL)
		return false;

	ds3231_device_t *d = (ds3231_device_t *) device;

	/*
	 * Disable interrupts before transaction
	 */
	taskENTER_CRITICAL();

	/*
	 * Start I2C transaction
	 */
	brzo_i2c_start_transaction(d->i2c_bus, d->i2c_address, d->i2c_frequency);
	brzo_i2c_ack_polling(d->i2c_bus, d->i2c_ack_timeout);

	/*
	 * Send register address
	 */
	brzo_i2c_write(d->i2c_bus, &reg, 1, true);

	/*
	 * Send register data
	 */
	brzo_i2c_write(d->i2c_bus, p_buff, length, false);

	/*
	 * End transaction
	 */
	if (brzo_i2c_end_transaction(d->i2c_bus) != 0)
		result = false;

	/*
	 * Restore interrupts after transaction
	 */
	taskEXIT_CRITICAL();

	return result;
}

bool ds3231_get_bit(h_ds3231 device, uint8_t reg, uint8_t bitmask,
		uint8_t * value)
{
	uint8_t buff;
	bool result = false;

	if (device == NULL)
		return false;

	ds3231_device_t *d = (ds3231_device_t *) device;

	/*
	 * Disable interrupts before transaction
	 */
	taskENTER_CRITICAL();

	/*
	 * Start I2C transaction
	 */
	brzo_i2c_start_transaction(d->i2c_bus, d->i2c_address, d->i2c_frequency);
	brzo_i2c_ack_polling(d->i2c_bus, d->i2c_ack_timeout);

	/*
	 * Send register address
	 */
	brzo_i2c_write(d->i2c_bus, &reg, 1, false);

	/*
	 * End transaction
	 */
	if (brzo_i2c_end_transaction(d->i2c_bus) == 0)
	{
		/*
		 * Start I2C transaction
		 */
		brzo_i2c_start_transaction(d->i2c_bus, d->i2c_address,
				d->i2c_frequency);
		brzo_i2c_ack_polling(d->i2c_bus, d->i2c_ack_timeout);

		/*
		 * Receive register data
		 */
		brzo_i2c_read(d->i2c_bus, &buff, 1, false);

		/*
		 * End transaction
		 */
		if (brzo_i2c_end_transaction(d->i2c_bus) == 0)
		{
			*value = (buff & bitmask) == bitmask ? 1 : 0;
			result = true;
		}
	}
	/*
	 * Restore interrupts after transaction
	 */
	taskEXIT_CRITICAL();

	return result;
}

bool ds3231_set_bit(h_ds3231 device, uint8_t reg, uint8_t bitmask, uint8_t value)
{
	uint8_t buff;
	bool result = false;

	if (device == NULL)
		return false;

	ds3231_device_t *d = (ds3231_device_t *) device;

	/*
	 * Disable interrupts before transaction
	 */
	taskENTER_CRITICAL();

	/*
	 * Start I2C transaction
	 */
	brzo_i2c_start_transaction(d->i2c_bus, d->i2c_address, d->i2c_frequency);
	brzo_i2c_ack_polling(d->i2c_bus, d->i2c_ack_timeout);

	/*
	 * Send register address
	 */
	brzo_i2c_write(d->i2c_bus, &reg, 1, false);

	/*
	 * End transaction
	 */
	if (brzo_i2c_end_transaction(d->i2c_bus) == 0)
	{
		/*
		 * Start I2C transaction
		 */
		brzo_i2c_start_transaction(d->i2c_bus, d->i2c_address,
				d->i2c_frequency);
		brzo_i2c_ack_polling(d->i2c_bus, d->i2c_ack_timeout);

		/*
		 * Receive register data
		 */
		brzo_i2c_read(d->i2c_bus, &buff, 1, false);

		/*
		 * End transaction
		 */
		if (brzo_i2c_end_transaction(d->i2c_bus) == 0)
		{
			result = true;

			/*
			 * Set bit value
			 */
			if (value == 0)
			{
				/*
				 * Clear bit to 0
				 */
				buff &= !bitmask;
			}
			else
			{
				/*
				 * Set bit to 1
				 */
				buff |= bitmask;
			}
			/*
			 * Start I2C transaction
			 */
			brzo_i2c_start_transaction(d->i2c_bus, d->i2c_address, d->i2c_frequency);
			brzo_i2c_ack_polling(d->i2c_bus, d->i2c_ack_timeout);

			/*
			 * Send register address
			 */
			brzo_i2c_write(d->i2c_bus, &reg, 1, true);

			/*
			 * Send register data
			 */
			brzo_i2c_write(d->i2c_bus, &buff, 1, false);

			/*
			 * End transaction
			 */
			if (brzo_i2c_end_transaction(d->i2c_bus) != 0)
				result = false;
		}
	}
	/*
	 * Restore interrupts after transaction
	 */
	taskEXIT_CRITICAL();

	return result;
}

bool ds3231_get_time(h_ds3231 device, struct tm * time)
{
    uint8_t data[7];

	if (device == NULL)
		return false;
	ds3231_device_t *d = (ds3231_device_t *) device;

    /*
     * Read time value
     */
    if (ds3231_get_raw(d, DS3231_REG_TIME, data, 6))
    {
        /*
         * Convert to unix time structure
         */
        time->tm_sec = bcd_to_dec(data[0]);
        time->tm_min = bcd_to_dec(data[1]);
        if (data[2] & DS3231_TIME_12HOUR_FLAG)
        {
            /*
             * 12H clock
             */
            time->tm_hour = bcd_to_dec(data[2] & DS3231_TIME_12HOUR_MASK) - 1;
            /*
             * AM/PM
             */
            if (data[2] & DS3231_TIME_PM_FLAG)
            	time->tm_hour += 12;
        }
        else
        {
            /*
             * 24H clock
             */
            time->tm_hour = bcd_to_dec(data[2]);
        }
        time->tm_wday = bcd_to_dec(data[3]) - 1;
        time->tm_mday = bcd_to_dec(data[4]);
        time->tm_mon  = bcd_to_dec(data[5] & DS3231_TIME_MONTH_MASK) - 1;
        time->tm_year = bcd_to_dec(data[6]) + 100;
        time->tm_isdst = 0;

        return true;
    }

    return false;
}

bool ds3231_set_time(h_ds3231 device, struct tm * time)
{
    uint8_t data[7] = { 0, 0, 0, 0, 0, 0, 0 };

	if (device == NULL)
		return false;
	ds3231_device_t *d = (ds3231_device_t *) device;

    /*
     * Time & date value
     */
    data[0] = dec_to_bcd(time->tm_sec);
    data[1] = dec_to_bcd(time->tm_min);
   	data[2] = dec_to_bcd(time->tm_hour);
    data[3] = dec_to_bcd(time->tm_wday + 1);
    data[4] = dec_to_bcd(time->tm_mday);
    data[5] = dec_to_bcd(time->tm_mon + 1);
    data[6] = dec_to_bcd(time->tm_year - 100);

    return ds3231_set_raw(d, DS3231_REG_TIME, data, 7);
}

bool ds3231_set_alarm1(h_ds3231 device, struct tm * time,
		ds3231_alarm1_rate_t rate)
{
    uint8_t data[4] = {
    		DS3231_ALARM_MATCH_FLAG, DS3231_ALARM_MATCH_FLAG,
			DS3231_ALARM_MATCH_FLAG, DS3231_ALARM_MATCH_FLAG };

    if (device == NULL)
		return false;
	ds3231_device_t * d = (ds3231_device_t *) device;

	if (NULL != time)
	{
		/*
		 * Set alarm time and enable alarm interrupt
		 */
		data[0] = (rate >= DS3231_ALARM1_MATCH_SEC ?
				dec_to_bcd(time->tm_sec) :
				DS3231_ALARM_MATCH_FLAG);

		data[1] = (rate >= DS3231_ALARM1_MATCH_SECMIN ?
				dec_to_bcd(time->tm_min) :
				DS3231_ALARM_MATCH_FLAG);

		data[2] = (rate >= DS3231_ALARM1_MATCH_SECMINHOUR ?
				dec_to_bcd(time->tm_hour) :
				DS3231_ALARM_MATCH_FLAG);

		data[3] = (rate == DS3231_ALARM1_MATCH_SECMINHOURDAY ?
				(dec_to_bcd(time->tm_wday + 1) & DS3231_ALARM_WEEKDAY_FLAG) :
				(rate == DS3231_ALARM1_MATCH_SECMINHOURDATE ?
						dec_to_bcd(time->tm_mday) :
						DS3231_ALARM_MATCH_FLAG));

		/*
		 * Write alarm time
		 */
		if (ds3231_set_raw(d, DS3231_REG_ALARM1, data, 4))
		{
			/*
			 * Enable alarm ineterrupt
			 */
			if (ds3231_set_bit(d, DS3231_REG_CONTROL, DS3231_CONTROL_A1IE, 1))
				return true;
		}
	}
	else
	{
		/*
		 * Reset alarm time and disable alarm interrupt
		 */
		/*
		 * Write clean value to alarm time
		 */
		if (ds3231_set_raw(d, DS3231_REG_ALARM1, data, 4))
		{
			/*
			 * Disable alarm interrupt
			 */
			if (ds3231_set_bit(d, DS3231_REG_CONTROL, DS3231_CONTROL_A1IE, 0))
				return true;
		}
	}

	return false;
}

bool ds3231_set_alarm2(h_ds3231 device, struct tm * time,
		ds3231_alarm2_rate_t rate)
{
    uint8_t data[3] = {
    		DS3231_ALARM_MATCH_FLAG, DS3231_ALARM_MATCH_FLAG,
			DS3231_ALARM_MATCH_FLAG };

    if (device == NULL)
		return false;
	ds3231_device_t * d = (ds3231_device_t *) device;

	if (NULL != time)
	{
		/*
		 * Set alarm time and enable alarm interrupt
		 */
        data[0] = (rate >= DS3231_ALARM2_MATCH_MIN ?
        		dec_to_bcd(time->tm_min) :
				DS3231_ALARM_MATCH_FLAG);

        data[1] = (rate >= DS3231_ALARM2_MATCH_MINHOUR ?
        		dec_to_bcd(time->tm_hour) :
				DS3231_ALARM_MATCH_FLAG);

        data[2] = (rate == DS3231_ALARM2_MATCH_MINHOURDAY ?
        		(dec_to_bcd(time->tm_wday + 1) & DS3231_ALARM_WEEKDAY_FLAG) :
				(rate == DS3231_ALARM2_MATCH_MINHOURDATE ?
						dec_to_bcd(time->tm_mday) :
						DS3231_ALARM_MATCH_FLAG));

		/*
		 * Write alarm time value
		 */
		if (ds3231_set_raw(d, DS3231_REG_ALARM2, data, 3))
		{
			/*
			 * Set alarm interrupt bit
			 */
			if (ds3231_set_bit(d, DS3231_REG_CONTROL, DS3231_CONTROL_A2IE, 1))
				return true;
		}
	}
	else
	{
		/*
		 * Reset alarm time and disable alarm interrupt
		 */
		/*
		 * Write clean value to alarm time
		 */
		if (ds3231_set_raw(d, DS3231_REG_ALARM2, data, 3))
		{
			/*
			 * Reset alarm interrupt bit
			 */
			if (ds3231_set_bit(d, DS3231_REG_CONTROL, DS3231_CONTROL_A2IE, 0))
				return true;
		}
	}

	return false;
}
