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
#define DS3231_I2C_DEF_ADDRESS              0x68

/*
 * [I2C] Bus speed in kbps for HDC1080
 */
#define DS3231_I2C_DEF_SPEED                100

/*
 * [I2C] Default ACS pooling timeout in uS
 */
#define DS3231_I2C_DEF_ACK_TIMEOUT          500

/*
 * Register address: Time value base
 */
#define DS3231_REG_TIME                     0x00
/*
 * Register address: Alarm 1 base
 */
#define DS3231_REG_ALARM1                   0x07
/*
 * Register address: Alarm 2 base
 */
#define DS3231_REG_ALARM2                   0x0b
/*
 * Register address: Control
 */
#define DS3231_REG_CONTROL                  0x0e
/*
 * Register address: Status
 */
#define DS3231_REG_STATUS                   0x0f
/*
 * Register address: Aging offset
 */
#define DS3231_REG_AGING                    0x10
/*
 * Register address: Temperature base
 */
#define DS3231_REG_TEMPERATURE              0x11

/*
 * Control Register (0x0E) bits
 */
/*
 * Control: Enable Oscillator (EOSC)
 *
 * When set to logic 0, the oscillator is started. When set
 * to logic 1, the oscillator is stopped when the DS3231
 * switches to VBAT. This bit is clear (logic 0) when power
 * is first applied. When the DS3231 is powered by VCC, the
 * oscillator is always on regardless of the status of the
 * EOSC bit. When EOSC is disabled, all register data is static.
 */
#define DS3231_CONTROL_EOSC                 0x80

/*
 * Control: Battery-Backed Square-Wave Enable (BBSQW)
 *
 * When set to logic 1 with INTCN = 0 and VCC < VPF, this
 * bit enables the square wave. When BBSQW is logic 0,
 * the INT/SQW pin goes high impedance when VCC < VPF.
 * This bit is disabled (logic 0) when power is first applied.
 */
#define DS3231_CONTROL_BBSQW                0x40

/*
 * Control: Convert Temperature (CONV)
 *
 * Setting this bit to 1 forces the temperature sensor
 * to convert the temperature into digital code and execute
 * the TCXO algorithm to update the capacitance array to
 * the oscillator. This can only happen when a conversion
 * is not already in progress. The user should check the
 * status bit BSY before forcing the controller to start
 * a new TCXO execution. A user-initiated temperature
 * conversion does not affect the internal 64-second update
 * cycle.
 *
 * A user-initiated temperature conversion does not affect
 * the BSY bit for approximately 2ms. The CONV bit remains
 * at a 1 from the time it is written until the conversion is
 * finished, at which time both CONV and BSY go to 0. The
 * CONV bit should be used when monitoring the status of a
 * user-initiated conversion.
 */
#define DS3231_CONTROL_CONV                 0x20

/*
 * Control: Square-Wave Rate Select (RS2 and RS1)
 *
 * These bits control the frequency of the square-wave output
 * when the square wave has been enabled. The following table
 * shows the square-wave frequencies that can be selected
 * with the RS bits. These bits are both set to logic 1
 * (8.192kHz) when power is first applied.
 */
#define DS3231_CONTROL_SQW_1HZ              0x00
#define DS3231_CONTROL_SQW_1024HZ           0x08
#define DS3231_CONTROL_SQW_4096HZ           0x10
#define DS3231_CONTROL_SQW_8192HZ           0x18
#define DS3231_CONTROL_SQW_MASK             0x18

/*
 * Control: Interrupt Control (INTCN)
 *
 * This bit controls the INT/SQW signal. When the INTCN bit
 * is set to logic 0, a square wave is output on the INT/SQW pin.
 * When the INTCN bit is set to logic 1, then a match between
 * the timekeeping registers and either of the alarm registers
 * activates the INT/SQW output (if the alarm is also enabled).
 * The corresponding alarm flag is always set regardless of
 * the state of the INTCN bit. The INTCN bit is set to logic 1
 * when power is first applied.
 */
#define DS3231_CONTROL_INTCN                0x04

/*
 * Control: Alarm 2 Interrupt Enable (A2IE)
 *
 * When set to logic 1, this bit permits the alarm 2 flag (A2F)
 * bit in the status register to assert INT/SQW (when INTCN = 1).
 * When the A2IE bit is set to logic 0 or INTCN is set to logic
 * 0, the A2F bit does not initiate an interrupt signal. The
 * A2IE bit is disabled (logic 0) when power is first applied.
 */
#define DS3231_CONTROL_A2IE                 0x02

/*
 * Control: Alarm 1 Interrupt Enable (A1IE)
 *
 * When set to logic 1, this bit permits the alarm 1 flag (A1F)
 * bit in the status register to assert INT/SQW (when INTCN = 1).
 * When the A1IE bit is set to logic 0 or INTCN is set to logic
 * 0, the A1F bit does not initiate the INT/SQW signal. The
 * A1IE bit is disabled (logic 0) when power is first applied.
 */
#define DS3231_CONTROL_A1IE                 0x01

/*
 * Status Register (0x0F) bits
 */
/*
 * Status: Oscillator Stop Flag (OSF)
 *
 * A logic 1 in this bit indicates that the oscillator either
 * is stopped or was stopped for some period and may be used
 * to judge the validity of the timekeeping data. This bit
 * is set to logic 1 any time that the oscillator stops. The
 * following are examples of conditions that can cause the OSF
 * bit to be set:
 * 1) The first time power is applied.
 * 2) The voltages present on both VCC and VBAT are insufficient
 *    to support oscillation.
 * 3) The EOSC bit is turned off in battery-backed mode.
 * 4) External influences on the crystal (i.e., noise, leakage,
 *    etc.).
 * This bit remains at logic 1 until written to logic 0.
 */
#define DS3231_STATUS_OSF                   0x80

/*
 * Status: Enable 32kHz Output (EN32kHz)
 *
 * This bit controls the status of the 32kHz pin. When set to
 * logic 1, the 32kHz pin is enabled and outputs a 32.768kHz
 * squarewave signal. When set to logic 0, the 32kHz pin goes
 * to a high-impedance state. The initial power-up state of
 * this bit is logic 1, and a 32.768kHz square-wave signal
 * appears at the 32kHz pin after a power source is applied
 * to the DS3231 (if the oscillator is running).
 */
#define DS3231_STATUS_E32KHZ                0x08

/*
 * Status: Busy (BSY)
 *
 * This bit indicates the device is busy executing TCXO functions.
 * It goes to logic 1 when the conversion signal to the temperature
 * sensor is asserted and then is cleared when the device is in
 * the 1-minute idle state.
 */
#define DS3231_STATUS_BSY                   0x04

/*
 * Status: Alarm 2 Flag (A2F)
 *
 * A logic 1 in the alarm 2 flag bit indicates that the time matched
 * the alarm 2 registers. If the A2IE bit is logic 1 and the INTCN bit
 * is set to logic 1, the INT/SQW pin is also asserted. A2F is cleared
 * when written to logic 0. This bit can only be written to logic 0.
 * Attempting to write to logic 1 leaves the value unchanged.
 */
#define DS3231_STATUS_A2F                   0x02

/*
 * Status:  Alarm 1 Flag (A1F)
 *
 * A logic 1 in the alarm 1 flag bit indicates that the time matched
 * the alarm 1 registers. If the A1IE bit is logic 1 and the INTCN bit
 * is set to logic 1, the INT/SQW pin is also asserted. A1F is cleared
 * when written to logic 0. This bit can only be written to logic 0.
 * Attempting to write to logic 1 leaves the value unchanged.
 */
#define DS3231_STATUS_A1F                   0x01

/*
 * Bit flags and masks for time value registers - time, alarms
 */
/*
 * 12-hour clock mode flag
 */
#define DS3231_TIME_12HOUR_FLAG             0x40
/*
 * Hour value mask for 12-hour mode
 */
#define DS3231_TIME_12HOUR_MASK             0x1f
/*
 * Flag and mask for AM/PM bit
 */
#define DS3231_TIME_PM_FLAG                 0x20
#define DS3231_TIME_AMPM_MASK               DS3231_TIME_PM_FLAG
/*
 * Mask for month value in 0x05 time register
 */
#define DS3231_TIME_MONTH_MASK              0x1f
/*
 * Bit and mask for century value in 0x0h time register
 */
#define DS3231_TIME_CENTURY_FLAG            0x80
#define DS3231_TIME_CENTURY_MASK            DS3231_TIME_CENTURY_FLAG
/*
 * Flag for switch weekday value versus day of month for alarms
 */
#define DS3231_ALARM_WEEKDAY_FLAG           0x40
/*
 * Flag that anables match appropriate alarm row
 *
 * Refer the documentation for DS3231 about allowed combinations
 * https://datasheets.maximintegrated.com/en/ds/DS3231.pdf
 */
#define DS3231_ALARM_MATCH_FLAG             0x80
#define DS3231_ALARM_MATCH_MASK             DS3231_ALARM_MATCH_FLAG

typedef enum
{
	DS3231_ALARM1_EVERY_SEC = 0,
	DS3231_ALARM1_MATCH_SEC,
	DS3231_ALARM1_MATCH_SECMIN,
	DS3231_ALARM1_MATCH_SECMINHOUR,
	DS3231_ALARM1_MATCH_SECMINHOURDAY,
	DS3231_ALARM1_MATCH_SECMINHOURDATE
} ds3231_alarm1_rate_t;

typedef enum
{
	DS3231_ALARM2_EVERY_MIN = 0,
	DS3231_ALARM2_MATCH_MIN,
	DS3231_ALARM2_MATCH_MINHOUR,
	DS3231_ALARM2_MATCH_MINHOURDAY,
	DS3231_ALARM2_MATCH_MINHOURDATE
} ds3231_alarm2_rate_t;

/*
 * Sensor instance handler
 */
typedef void * h_ds3231;

/**
 * @brief      Setup instance of DS3231 device I2C slave device
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
 * @brief      Release instance of DS3231 device I2C slave device
 *
 * @param [in] device
 *             Handler of DS3231 RTC I2C slave device instance
 *
 * @return     void.
 */
void ds3231_free(h_ds3231 device);

/*
 * @brief       Get raw data
 *
 * Read length raw bytes from point registers
 *
 * @param  [in] device
 *              Handler of DS3231 RTC I2C slave device instance
 * @param  [in] reg
 *              Register address to start read data
 * @param [out] p_buff
 *              Buffer to receive data
 * @param  [in] length
 *              Count of bytes to read
 *
 * @return      TRUE if read success, FALSE if read fails.
 */
bool ds3231_get_raw(h_ds3231 device, uint8_t reg, uint8_t * p_buff, uint8_t length);

/*
 * @brief       Set raw data
 *
 * @param  [in] device
 *              Handler of DS3231 RTC I2C slave device instance
 * @param  [in] reg
 *              Register address to start write data
 * @param [out] p_buff
 *              Buffer to data to writen
 * @param  [in] length
 *              Count of bytes to write
 *
 * @return      TRUE if read success, FALSE if read fails.
 */
bool ds3231_set_raw(h_ds3231 device, uint8_t reg, uint8_t * p_buff, uint8_t length);

/**
 * @brief       Get one bit value from desired register
 *
 * @param  [in] device
 *              Handler of DS3231 RTC I2C slave device instance
 * @param  [in] reg
 *              Register address
 * @param  [in] bitmask
 *              Single bit mask
 * @param [out] value
 *              Value to return [0,1]
 *
 * @return      TRUE if read success, FALSE if read fails.
 */
bool ds3231_get_bit(h_ds3231 device, uint8_t reg, uint8_t bitmask,
		uint8_t * value);

/**
 * @brief       Set one bit value to desired register
 *
 * @param  [in] device
 *              Handler of DS3231 RTC I2C slave device instance
 * @param  [in] reg
 *              Register address
 * @param  [in] bitmask
 *              Single bit mask
 * @param [out] value
 *              Value to set [0,1]
 *
 * @return      TRUE if read success, FALSE if read fails.
 */
bool ds3231_set_bit(h_ds3231 device, uint8_t reg, uint8_t bit, uint8_t value);

/**
 * @brief       Get the time from the RTC, populates a supplied tm structure
 *
 * @param  [in] device
 *              Handler of DS3231 RTC I2C slave device instance
 * @param [out] time
 *              Pointer to tm structure
 *
 * @return      TRUE if read success, FALSE if read fails.
 */
bool ds3231_get_time(h_ds3231 device, struct tm *time);

/**
 * @brief       Set the time on the DS3231 RTC
 *
 *              timezone agnostic, pass whatever you like
 *              I suggest using GMT and applying timezone and
 *              DST when read back
 *
 * @param  [in] device
 *              Handler of DS3231 RTC I2C slave device instance
 * @param [out] time
 *              Pointer to tm structure
 *
 * @return      TRUE if read success, FALSE if read fails.
 */
bool ds3231_set_time(h_ds3231 device, struct tm *time);

/**
 * @brief       Set and enable or disable alarm1 timer
 *
 *              timezone agnostic, pass whatever you like
 *              I suggest using GMT and applying timezone and
 *              DST when read back
 *
 * @param  [in] device
 *              Handler of DS3231 RTC I2C slave device instance
 * @param  [in] time
 *              Pointer to tm structure, if NULL alarm will be disabled
 * @param  [in] rate
 *              Pointer to tm structure
 *
 * @return      TRUE if operation success, FALSE if fails.
 */
bool ds3231_set_alarm1(h_ds3231 device, struct tm * time,
		ds3231_alarm1_rate_t rate);

/**
 * @brief       Set and enable or disable alarm2 timer
 *
 * @param  [in] device
 *              Handler of DS3231 RTC I2C slave device instance
 * @param  [in] time
 *              Pointer to tm structure, if NULL alarm will be disabled
 * @param  [in] rate
 *              Pointer to tm structure
 *
 * @return      TRUE if operation success, FALSE if fails.
 */
bool ds3231_set_alarm2(h_ds3231 device, struct tm * time,
		ds3231_alarm2_rate_t rate);

#ifdef	__cplusplus
}
#endif

#endif  /* __DS3231_H__ */
