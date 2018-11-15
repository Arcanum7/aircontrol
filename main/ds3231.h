/* Driver for DS3231 high precision RTC module
 *
 * Part of esp-open-rtos
 * Copyright (C) 2015 Richard A Burton <richardaburton@gmail.com>
 * Copyright (C) 2016 Bhuvanchandra DV <bhuvanchandra.dv@gmail.com>
 * MIT Licensed as described in the file LICENSE
 */

/**
 * 2018 Michael Kolomiets (michael dot kolomiets at gmail dot com).
 *
 * Functions to manipulate RTC DS3231
 * Use brzo_i2c driver library to work with I2C bus
 *
 */

#ifndef __DS3231_H__
#define __DS3231_H__

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include "brzo_i2c.h"

#ifdef	__cplusplus
extern "C"
{
#endif

/*
 * [I2C] Default address for DS3231
 */
#define DS3231_I2C_DEF_ADDRESS		0x68

/*
 * [I2C] Bus speed in kbps for HDC1080
 */
#define DS3231_I2C_DEF_SPEED 		100

/*
 * [I2C] Default ACS pooling timeout in uS for HDC1080
 */
#define DS3231_I2C_DEF_ACK_TIMEOUT	500

/*
 * Register address: Time value base
 */
#define DS3231_REG_TIME			0x00
/*
 * Register address: Alarm 1 base
 */
#define DS3231_REG_ALARM1		0x07
/*
 * Register address: Alarm 2 base
 */
#define DS3231_REG_ALARM2		0x0b
/*
 * Register address: Control
 */
#define DS3231_REG_CONTROL		0x0e
/*
 * Register address: Status
 */
#define DS3231_REG_STATUS		0x0f
/*
 * Register address: Aging offset
 */
#define DS3231_ADDR_AGING		0x10
/*
 * Register address: Temperature base
 */
#define DS3231_ADDR_TEMPERATURE	0x11

/*
 * Control Register (0x0E) bits
 */
/*
 * Control: Enable Oscillator (EOSC)
 */
#define DS3231_CONTROL_EOSC			0x80
/*
 * Control: Battery-Backed Square-Wave Enable (BBSQW)
 */
#define DS3231_CONTROL_BBSQW		0x40
/*
 * Control: Battery-Backed Square-Wave Enable (BBSQW)
 */
#define DS3231_CONTROL_CONV			0x20
/*
 * Control: Square-Wave Rate Select (RS2 and RS1)
 */
#define DS3231_CONTROL_SQW_1HZ		0x00
#define DS3231_CONTROL_SQW_1024HZ	0x08
#define DS3231_CONTROL_SQW_4096HZ	0x10
#define DS3231_CONTROL_SQW_8192HZ	0x18
/*
 * Control: Interrupt Control (INTCN)
 */
#define DS3231_CONTROL_INTCN		0x04
/*
 * Control: Alarm 2 Interrupt Enable (A2IE)
 */
#define DS3231_CONTROL_A2IE			0x02
/*
 * Control: Alarm 1 Interrupt Enable (A1IE)
 */
#define DS3231_CONTROL_A1IE			0x01

/*
 * Status Register (0x0F) bits
 */
/*
 * Status: Oscillator Stop Flag
 */
#define DS3231_STATUS_OSF			0x80
/*
 * Status: Enable 32kHz Output
 */
#define DS3231_STATUS_E32KHZ		0x08
/*
 * Status: Busy
 */
#define DS3231_STATUS_BSY			0x04
/*
 * Status: Alarm 2 Flag
 */
#define DS3231_STATUS_A2F       	0x02
/*
 * Status: Alarm 1 Flag
 */
#define DS3231_STATUS_A1F       	0x01

#define DS3231_ALARM_AMPM_MASK		0x20
#define DS3231_ALARM_DAY			0x40
#define DS3231_ALARM_NOTSET			0x80

#define DS3231_12HOUR_FLAG			0x40
#define DS3231_12HOUR_MASK			0x1f
#define DS3231_PM_FLAG				0x20
#define DS3231_MONTH_MASK			0x1f

enum
{
	DS3231_SET = 0,
	DS3231_CLEAR,
	DS3231_REPLACE
};

enum
{
	DS3231_ALARM_NONE = 0,
	DS3231_ALARM_1,
	DS3231_ALARM_2,
	DS3231_ALARM_BOTH
};

enum
{
	DS3231_ALARM1_EVERY_SECOND = 0,
	DS3231_ALARM1_MATCH_SEC,
	DS3231_ALARM1_MATCH_SECMIN,
	DS3231_ALARM1_MATCH_SECMINHOUR,
	DS3231_ALARM1_MATCH_SECMINHOURDAY,
	DS3231_ALARM1_MATCH_SECMINHOURDATE
};

enum
{
	DS3231_ALARM2_EVERY_MIN = 0,
	DS3231_ALARM2_MATCH_MIN,
	DS3231_ALARM2_MATCH_MINHOUR,
	DS3231_ALARM2_MATCH_MINHOURDAY,
	DS3231_ALARM2_MATCH_MINHOURDATE
};

/*
 * Sensor instance handler
 */
typedef void * h_ds3231;

/**
 * @brief      Setup instance of DS3231 sensor I2C slave device
 *
 * @param [in] i2c_bus
 *             I2C bus instance handler
 * @param [in] i2c_address
 *             I2C address of device, 7 bit
 * @param [in] i2c_frequency
 *             I2C clock frequency in kHz
 * @param [in] i2c_ack_timeout
 *             I2C slave ACK wait timeout in usec
 *
 * @return     ds3231_t, NULL:indicates failure,
 *             valid pointer value indicates success.
 */
h_ds3231 ds3231_setup(h_brzo_i2c_bus i2c_bus, uint8_t i2c_address,
		uint16_t i2c_frequency, uint16_t i2c_ack_timeout);

/**
 * @brief      Release instance of DS3231 sensor I2C slave device
 *
 * @param [in] sensor
 *             Handler of DS sensor I2C slave device instance
 *
 * @return     void.
 */
void ds3231_free(h_ds3231 sensor);

/* Set the time on the rtc
 * timezone agnostic, pass whatever you like
 * I suggest using GMT and applying timezone and DST when read back
 * returns true to indicate success
 */
bool ds3231_set_time(struct tm *time);

/* Set alarms
 * alarm1 works with seconds, minutes, hours and day of week/month, or fires every second
 * alarm2 works with minutes, hours and day of week/month, or fires every minute
 * not all combinations are supported, see DS3231_ALARM1_* and DS3231_ALARM2_* defines
 * for valid options you only need to populate the fields you are using in the tm struct,
 * and you can set both alarms at the same time (pass DS3231_ALARM_1/DS3231_ALARM_2/DS3231_ALARM_BOTH)
 * if only setting one alarm just pass 0 for tm struct and option field for the other alarm
 * if using DS3231_ALARM1_EVERY_SECOND/DS3231_ALARM2_EVERY_MIN you can pass 0 for tm stuct
 * if you want to enable interrupts for the alarms you need to do that separately
 * returns true to indicate success
 */
bool ds3231_set_alarm(uint8_t alarms, struct tm *time1, uint8_t option1,
		struct tm *time2, uint8_t option2);

/* Check if oscillator has previously stopped, e.g. no power/battery or disabled
 * sets flag to true if there has been a stop
 * returns true to indicate success
 */
bool ds3231_getOscillatorStopFlag(bool *flag);

/* Clear the oscillator stopped flag
 * returns true to indicate success
 */
bool ds3231_clearOscillatorStopFlag();

/* Check which alarm(s) have past
 * sets alarms to DS3231_ALARM_NONE/DS3231_ALARM_1/DS3231_ALARM_2/DS3231_ALARM_BOTH
 * returns true to indicate success
 */
bool ds3231_getAlarmFlags(uint8_t *alarms);

/* Clear alarm past flag(s)
 * pass DS3231_ALARM_1/DS3231_ALARM_2/DS3231_ALARM_BOTH
 * returns true to indicate success
 */
bool ds3231_clearAlarmFlags(uint8_t alarm);

/* enable alarm interrupts (and disables squarewave)
 * pass DS3231_ALARM_1/DS3231_ALARM_2/DS3231_ALARM_BOTH
 * if you set only one alarm the status of the other is not changed
 * you must also clear any alarm past flag(s) for alarms with
 * interrupt enabled, else it will trigger immediately
 * returns true to indicate success
 */
bool ds3231_enableAlarmInts(uint8_t alarms);

/* Disable alarm interrupts (does not (re-)enable squarewave)
 * pass DS3231_ALARM_1/DS3231_ALARM_2/DS3231_ALARM_BOTH
 * returns true to indicate success
 */
bool ds3231_disableAlarmInts(uint8_t alarms);

/* Enable the output of 32khz signal
 * returns true to indicate success
 */
bool ds3231_enable32khz();

/* Disable the output of 32khz signal
 * returns true to indicate success
 */
bool ds3231_disable32khz();

/* Enable the squarewave output (disables alarm interrupt functionality)
 * returns true to indicate success
 */
bool ds3231_enableSquarewave();

/* Disable the squarewave output (which re-enables alarm interrupts, but individual
 * alarm interrupts also need to be enabled, if not already, before they will trigger)
 * returns true to indicate success
 */
bool ds3231_disableSquarewave();

/* Set the frequency of the squarewave output (but does not enable it)
 * pass DS3231_SQUAREWAVE_RATE_1HZ/DS3231_SQUAREWAVE_RATE_1024HZ/DS3231_SQUAREWAVE_RATE_4096HZ/DS3231_SQUAREWAVE_RATE_8192HZ
 * returns true to indicate success
 */
bool ds3231_setSquarewaveFreq(uint8_t freq);

/* Get the raw value
 * returns true to indicate success
 */
bool ds3231_getRawTemp(int16_t *temp);

/* Get the temperature as an integer
 * returns true to indicate success
 */
bool ds3231_getTempInteger(int8_t *temp);

/* Get the temerapture as a float (in quarter degree increments)
 * returns true to indicate success
 */
bool ds3231_getTempFloat(float *temp);

/* Get the time from the rtc, populates a supplied tm struct
 * returns true to indicate success
 */
bool ds3231_getTime(struct tm *time);
void ds3231_Init(uint8_t scl, uint8_t sda);

#ifdef	__cplusplus
}
#endif

#endif  /* __DS3231_H__ */
