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

/* Read a number of bytes from the RTC over i2c
 * returns true to indicate success
 */
bool ds3231_reg_get(ds3231_device_t * device, uint8_t reg,
		uint8_t *pData, uint32_t length);

/* Send a number of bytes to the RTC over i2c
 * returns true to indicate success
 */
bool ds3231_reg_set(ds3231_device_t * device, uint8_t reg,
		uint8_t * pData, uint32_t length);

/* Get a byte containing just the requested bits
 * pass the register address to read, a mask to apply to the register and
 * an uint* for the output
 * you can test this value directly as true/false for specific bit mask
 * of use a mask of 0xff to just return the whole register byte
 * returns true to indicate success
 */
bool ds3231_flag_get(ds3231_device_t * device, uint8_t reg,
		uint8_t mask, uint8_t *flag);

/* Set/clear bits in a byte register, or replace the byte altogether
 * pass the register address to modify, a byte to replace the existing
 * value with or containing the bits to set/clear and one of
 * DS3231_SET/DS3231_CLEAR/DS3231_REPLACE
 * returns true to indicate success
 */
bool ds3231_flag_set(ds3231_device_t * device, uint8_t reg,
		uint8_t bits, uint8_t mode);


h_ds3231 ICACHE_FLASH_ATTR ds3231_setup(h_brzo_i2c_bus i2c_bus,
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

void ICACHE_FLASH_ATTR ds3231_free(h_ds3231 device)
{
	if (device == NULL)
		return;

	ds3231_device_t *d = (ds3231_device_t *) device;

	os_free(d);
}

bool ds3231_get_time(h_ds3231 device, struct tm *time)
{
    uint8_t data[7];

	if (device == NULL)
		return false;

	ds3231_device_t *d = (ds3231_device_t *) device;

    /*
     * Read time value
     */
    if (!ds3231_reg_get(d, DS3231_REG_TIME, data, 6)) {
        return false;
    }

    /* convert to unix time structure */
    time->tm_sec = bcd_to_dec(data[0]);
    time->tm_min = bcd_to_dec(data[1]);
    if (data[2] & DS3231_TIME_12HOUR_FLAG) {
        /* 12H */
        time->tm_hour = bcd_to_dec(data[2] & DS3231_TIME_12HOUR_MASK) - 1;
        /* AM/PM? */
        if (data[2] & DS3231_TIME_PM_FLAG) time->tm_hour += 12;
    } else {
        /* 24H */
        time->tm_hour = bcd_to_dec(data[2]);
    }
    time->tm_wday = bcd_to_dec(data[3]) - 1;
    time->tm_mday = bcd_to_dec(data[4]);
    time->tm_mon  = bcd_to_dec(data[5] & DS3231_TIME_MONTH_MASK) - 1;
    time->tm_year = bcd_to_dec(data[6]) + 100;
    time->tm_isdst = 0;

    // apply a time zone (if you are not using localtime on the rtc or you want to check/apply DST)
    //applyTZ(time);

    return true;

}

bool ICACHE_FLASH_ATTR ds3231_set_time(h_ds3231 device, struct tm *time, bool clock_12hour)
{
    uint8_t data[7];

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

    return ds3231_reg_set(d, DS3231_REG_TIME, data, 7);
}

bool ds3231_set_alarm(h_ds3231 device, uint8_t alarms, struct tm *time1,
		uint8_t option1, struct tm *time2, uint8_t option2)
{
    uint8_t data[8];
    int i = 0;

    if (device == NULL)
		return false;

	ds3231_device_t *d = (ds3231_device_t *) device;

    /* alarm 1 data */
    if (alarms != DS3231_ALARM_2) {
        data[i++] = (option1 >= DS3231_ALARM1_MATCH_SEC ?
        		dec_to_bcd(time1->tm_sec) :
				DS3231_ALARM_MATCH_FLAG);
        data[i++] = (option1 >= DS3231_ALARM1_MATCH_SECMIN ?
        		dec_to_bcd(time1->tm_min) :
				DS3231_ALARM_MATCH_FLAG);
        data[i++] = (option1 >= DS3231_ALARM1_MATCH_SECMINHOUR ?
        		dec_to_bcd(time1->tm_hour) :
				DS3231_ALARM_MATCH_FLAG);
        data[i++] = (option1 == DS3231_ALARM1_MATCH_SECMINHOURDAY ?
        		(dec_to_bcd(time1->tm_wday + 1) & DS3231_ALARM_WEEKDAY_FLAG) :
				(option1 == DS3231_ALARM1_MATCH_SECMINHOURDATE ?
						dec_to_bcd(time1->tm_mday) :
						DS3231_ALARM_MATCH_FLAG));
    }

    /* alarm 2 data */
    if (alarms != DS3231_ALARM_1) {
        data[i++] = (option2 >= DS3231_ALARM2_MATCH_MIN ?
        		dec_to_bcd(time2->tm_min) :
				DS3231_ALARM_MATCH_FLAG);
        data[i++] = (option2 >= DS3231_ALARM2_MATCH_MINHOUR ?
        		dec_to_bcd(time2->tm_hour) :
				DS3231_ALARM_MATCH_FLAG);
        data[i++] = (option2 == DS3231_ALARM2_MATCH_MINHOURDAY ?
        		(dec_to_bcd(time2->tm_wday + 1) & DS3231_ALARM_WEEKDAY_FLAG) :
				(option2 == DS3231_ALARM2_MATCH_MINHOURDATE ?
						dec_to_bcd(time2->tm_mday) :
						DS3231_ALARM_MATCH_FLAG));
    }

    return ds3231_reg_set(d,
    		(alarms == DS3231_ALARM_2 ? DS3231_REG_ALARM2 : DS3231_REG_ALARM1),
			data, i);
}

bool ds3231_getOscillatorStopFlag(bool *flag)
{
    uint8_t f;

    if (ds3231_flag_get(DS3231_REG_STATUS, DS3231_STATUS_OSF, &f)) {
        *flag = (f ? true : false);
        return true;
    }

    return false;
}

inline bool ds3231_clearOscillatorStopFlag()
{
    return ds3231_flag_set(DS3231_REG_STATUS, DS3231_STATUS_OSF, DS3231_CLEAR);
}

inline bool ds3231_getAlarmFlags(uint8_t *alarms)
{
    return ds3231_flag_get(DS3231_REG_STATUS, DS3231_ALARM_BOTH, alarms);
}

inline bool ds3231_clearAlarmFlags(uint8_t alarms)
{
    return ds3231_flag_set(DS3231_REG_STATUS, alarms, DS3231_CLEAR);
}

inline bool ds3231_enableAlarmInts(uint8_t alarms)
{
    return ds3231_flag_set(DS3231_REG_CONTROL, DS3231_CONTROL_INTCN | alarms, DS3231_SET);
}

inline bool ds3231_disableAlarmInts(uint8_t alarms)
{
    /* Just disable specific alarm(s) requested
     * does not disable alarm interrupts generally (which would enable the squarewave)
     */
    return ds3231_flag_set(DS3231_REG_CONTROL, alarms, DS3231_CLEAR);
}

inline bool ds3231_enable32khz()
{
    return ds3231_flag_set(DS3231_REG_STATUS, DS3231_CONTROL_EOSC, DS3231_SET);
}

inline bool ds3231_disable32khz()
{
    return ds3231_flag_set(DS3231_REG_STATUS, DS3231_CONTROL_EOSC, DS3231_CLEAR);
}

inline bool ds3231_enableSquarewave()
{
    return ds3231_flag_set(DS3231_REG_CONTROL, DS3231_CONTROL_INTCN, DS3231_CLEAR);
}

inline bool ds3231_disableSquarewave()
{
    return ds3231_flag_set(DS3231_REG_CONTROL, DS3231_CONTROL_INTCN, DS3231_SET);
}

bool ds3231_setSquarewaveFreq(uint8_t freq)
{
    uint8_t flag = 0;

    if (ds3231_flag_get(DS3231_REG_CONTROL, 0xff, &flag)) {
        /* clear current rate */
        flag &= ~DS3231_CONTROL_SQW_8192HZ;
        /* set new rate */
        flag |= freq;

        return ds3231_flag_set(DS3231_REG_CONTROL, flag, DS3231_REPLACE);
    }
    return false;
}

bool ds3231_getRawTemp(int16_t *temp)
{
    uint8_t data[2];

    data[0] = DS3231_REG_TEMPERATURE;
    if (ds3231_reg_set(data, 1) && ds3231_reg_get(data, 2)) {
        *temp = (int16_t)(int8_t)data[0] << 2 | data[1] >> 6;
        return true;
    }

    return false;
}

bool ds3231_getTempInteger(int8_t *temp)
{
    int16_t tInt;

    if (ds3231_getRawTemp(&tInt)) {
        *temp = tInt >> 2;
        return true;
    }

    return false;
}

bool ds3231_getTempFloat(float *temp)
{
    int16_t tInt;

    if (ds3231_getRawTemp(&tInt)) {
        *temp = tInt * 0.25;
        return true;
    }

    return false;
}

uint8_t ICACHE_FLASH_ATTR dec_to_bcd(uint8_t dec)
{
    return(((dec / 10) * 16) + (dec % 10));
}

uint8_t ICACHE_FLASH_ATTR bcd_to_dec(uint8_t bcd)
{
    return(((bcd / 16) * 10) + (bcd % 16));
}

bool ICACHE_FLASH_ATTR ds3231_reg_get(ds3231_device_t * device, uint8_t reg,
		uint8_t *pData, uint32_t length)
{
	bool result = false;

	if (device == NULL)
		return false;

	/*
	 * Disable interrupts before transaction
	 */
	taskENTER_CRITICAL();
	/*
	 * Start I2C transaction
	 */
	brzo_i2c_start_transaction(device->i2c_bus, device->i2c_address,
			device->i2c_frequency);
	brzo_i2c_ack_polling(device->i2c_bus, device->i2c_ack_timeout);
	/*
	 * Send register address
	 */
	brzo_i2c_write(device->i2c_bus, &reg, 1, false);
	/*
	 * End transaction
	 */
	if (brzo_i2c_end_transaction(device->i2c_bus) == 0)
	{
		/*
		 * Start I2C transaction
		 */
		brzo_i2c_start_transaction(device->i2c_bus, device->i2c_address,
				device->i2c_frequency);
		brzo_i2c_ack_polling(device->i2c_bus, device->i2c_ack_timeout);
		/*
		 * Receive register data
		 */
		brzo_i2c_read(device->i2c_bus, pData, length, false);
		/*
		 * End transaction
		 */
		if (brzo_i2c_end_transaction(device->i2c_bus) == 0)
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

bool ICACHE_FLASH_ATTR ds3231_reg_set(ds3231_device_t * device, uint8_t reg,
		uint8_t * pData, uint32_t length)
{
	bool result = true;

	if (device == NULL)
		return false;

	/*
	 * Disable interrupts before transaction
	 */
	taskENTER_CRITICAL();
	/*
	 * Start I2C transaction
	 */
	brzo_i2c_start_transaction(device->i2c_bus, device->i2c_address,
			device->i2c_frequency);
	brzo_i2c_ack_polling(device->i2c_bus, device->i2c_ack_timeout);
	/*
	 * Send register address
	 */
	brzo_i2c_write(device->i2c_bus, &reg, 1, true);
	/*
	 * Send data
	 */
	brzo_i2c_write(device->i2c_bus, pData, length, false);
	/*
	 * End transaction
	 */
	if (brzo_i2c_end_transaction(device->i2c_bus) != 0)
		result = false;
	/*
	 * Restore interrupts after transaction
	 */
	taskEXIT_CRITICAL();

	return result;
}

bool ICACHE_FLASH_ATTR ds3231_flag_get(ds3231_device_t * device,
		uint8_t reg, uint8_t mask, uint8_t *flag)
{
    uint8_t data[1];

    /* get register value */
    if (ds3231_reg_get(data, 1)) {
        /* return only requested flag */
        *flag = (data[0] & mask);
        return true;
    }

    return false;
}

bool ICACHE_FLASH_ATTR ds3231_flag_set(ds3231_device_t * device,
		uint8_t reg, uint8_t bits, uint8_t mode)
{
    uint8_t data[2];

    data[0] = reg;
    /* get status register */
    if (ds3231_reg_set(data, 1) && ds3231_reg_get(data+1, 1)) {
        /* clear the flag */
        if (mode == DS3231_REPLACE)
            data[1] = bits;
        else if (mode == DS3231_SET)
            data[1] |= bits;
        else
            data[1] &= ~bits;

        if (ds3231_reg_set(data, 2)) {
            return true;
        }
    }

    return false;
}
