// Basic support for common SPI controlled thermocouple chips
//
// Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "board/irq.h" // irq_disable
#include "basecmd.h" // oid_alloc
#include "byteorder.h" // be32_to_cpu
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "spicmds.h" // spidev_transfer

enum {
	TS_CHIP_MAX31855, TS_CHIP_MAX31856,
	TS_CHIP_MAX31865, TS_CHIP_MAX6675,
	TS_CHIP_ADS1118, TS_CHIP_ADS1118B
};

DECL_ENUMERATION("thermocouple_type", "MAX31855", TS_CHIP_MAX31855);
DECL_ENUMERATION("thermocouple_type", "MAX31856", TS_CHIP_MAX31856);
DECL_ENUMERATION("thermocouple_type", "MAX31865", TS_CHIP_MAX31865);
DECL_ENUMERATION("thermocouple_type", "MAX6675", TS_CHIP_MAX6675);
DECL_ENUMERATION("thermocouple_type", "ADS1118", TS_CHIP_ADS1118);
DECL_ENUMERATION("thermocouple_type", "ADS1118B", TS_CHIP_ADS1118B);

struct thermocouple_spi {
    struct timer timer;
    uint32_t rest_time;
    uint32_t min_value;           // Min allowed ADC value
    uint32_t max_value;           // Max allowed ADC value
    struct spidev_s *spi;
    uint8_t max_invalid, invalid_count;
    uint8_t chip_type, flags;
};

enum {
    TS_PENDING = 1,
};

static struct task_wake thermocouple_wake;

static uint_fast8_t thermocouple_event(struct timer *timer) {
    struct thermocouple_spi *spi = container_of(
            timer, struct thermocouple_spi, timer);
    // Trigger task to read and send results
    sched_wake_task(&thermocouple_wake);
    spi->flags |= TS_PENDING;
    spi->timer.waketime += spi->rest_time;
    return SF_RESCHEDULE;
}

void
command_config_thermocouple(uint32_t *args)
{
    uint8_t chip_type = args[2];
    if (chip_type > TS_CHIP_ADS1118B)
        shutdown("Invalid thermocouple chip type");
    struct thermocouple_spi *spi = oid_alloc(
        args[0], command_config_thermocouple, sizeof(*spi));
    spi->timer.func = thermocouple_event;
    spi->spi = spidev_oid_lookup(args[1]);
    spi->chip_type = chip_type;
}
DECL_COMMAND(command_config_thermocouple,
             "config_thermocouple oid=%c spi_oid=%c thermocouple_type=%c");

void
command_query_thermocouple(uint32_t *args)
{
    struct thermocouple_spi *spi = oid_lookup(
        args[0], command_config_thermocouple);

    sched_del_timer(&spi->timer);
    spi->timer.waketime = args[1];
    spi->rest_time = args[2];
    if (! spi->rest_time)
        return;
    spi->min_value = args[3];
    spi->max_value = args[4];
    spi->max_invalid = args[5];
    spi->invalid_count = 0;
    sched_add_timer(&spi->timer);
}
DECL_COMMAND(command_query_thermocouple,
             "query_thermocouple oid=%c clock=%u rest_ticks=%u"
             " min_value=%u max_value=%u max_invalid_count=%c");

static void
thermocouple_respond(struct thermocouple_spi *spi, uint32_t next_begin_time
                     , uint32_t value, uint8_t fault, uint8_t oid)
{
    sendf("thermocouple_result oid=%c next_clock=%u value=%u fault=%c",
          oid, next_begin_time, value, fault);
    /* check the result and stop if below or above allowed range */
    if (fault || value < spi->min_value || value > spi->max_value) {
        spi->invalid_count++;
        if (spi->invalid_count < spi->max_invalid)
            return;
        try_shutdown("Thermocouple reader fault");
    }
    spi->invalid_count = 0;
}

static void
thermocouple_handle_max31855(struct thermocouple_spi *spi
                             , uint32_t next_begin_time, uint8_t oid)
{
    uint8_t msg[4] = { 0x00, 0x00, 0x00, 0x00 };
    spidev_transfer(spi->spi, 1, sizeof(msg), msg);
    uint32_t value;
    memcpy(&value, msg, sizeof(value));
    value = be32_to_cpu(value);
    thermocouple_respond(spi, next_begin_time, value, value & 0x07, oid);
}

#define MAX31856_LTCBH_REG 0x0C
#define MAX31856_SR_REG 0x0F

static void
thermocouple_handle_max31856(struct thermocouple_spi *spi
                             , uint32_t next_begin_time, uint8_t oid)
{
    uint8_t msg[4] = { MAX31856_LTCBH_REG, 0x00, 0x00, 0x00 };
    spidev_transfer(spi->spi, 1, sizeof(msg), msg);
    uint32_t value;
    memcpy(&value, msg, sizeof(value));
    value = be32_to_cpu(value) & 0x00ffffff;
    // Read faults
    msg[0] = MAX31856_SR_REG;
    msg[1] = 0x00;
    spidev_transfer(spi->spi, 1, 2, msg);
    thermocouple_respond(spi, next_begin_time, value, msg[1], oid);
}

#define MAX31865_RTDMSB_REG 0x01
#define MAX31865_FAULTSTAT_REG 0x07

static void
thermocouple_handle_max31865(struct thermocouple_spi *spi
                             , uint32_t next_begin_time, uint8_t oid)
{
    uint8_t msg[4] = { MAX31865_RTDMSB_REG, 0x00, 0x00, 0x00 };
    spidev_transfer(spi->spi, 1, 3, msg);
    uint32_t value;
    memcpy(&value, msg, sizeof(value));
    value = (be32_to_cpu(value) >> 8) & 0xffff;
    // Read faults
    msg[0] = MAX31865_FAULTSTAT_REG;
    msg[1] = 0x00;
    spidev_transfer(spi->spi, 1, 2, msg);
    uint8_t fault = (msg[1] & ~0x03) | (value & 0x0001);
    thermocouple_respond(spi, next_begin_time, value, fault, oid);
}

static void
thermocouple_handle_max6675(struct thermocouple_spi *spi
        , uint32_t next_begin_time, uint8_t oid)
{
    uint8_t msg[2] = { 0x00, 0x00};
    spidev_transfer(spi->spi, 1, sizeof(msg), msg);
    uint16_t value;
    memcpy(&value, msg, sizeof(msg));
    value = be16_to_cpu(value);
    thermocouple_respond(spi, next_begin_time, value, value & 0x06, oid);
}
uint16_t ads_cold = 0;
uint16_t ads_t1 = 0;
static void
thermocouple_handle_ads1118(struct thermocouple_spi *spi
        , uint32_t next_begin_time, uint8_t oid)
{
    static uint16_t ads_t0 = 0;
    static uint8_t ads_mux = 0;
    static uint8_t ads_loop_cnt = 0;
    static uint8_t ads_cold_cnt = 0;

    uint8_t msg[3] [4] =
    {
	    // 0x0c9a = ch Cold pullup 128SPS Continuous
	    {0x00 , 0x00 , 0x0c , 0x9a },
	    // 0x0c8a = ch 0 and 1 pullup 128SPS Continuous ±0.256 V
	    {0x0c , 0x8a , 0x0c , 0x8a },
	    // 2 and 3 pullup 128SPS Continuous ±0.256 V
	    {0x00 , 0x00 , 0x3c , 0x8a }
    };

    spidev_transfer(spi->spi, 1, sizeof(msg[ads_mux]), msg[ads_mux]);
    uint32_t value;
    memcpy(&value, msg[ads_mux], sizeof(value));
    value = be32_to_cpu(value) ;

    if (ads_loop_cnt < 1 )
    {
        ads_loop_cnt++;
    }
    else
    {
        ads_loop_cnt = 0;
        //sendf("debug#######  value=%hu" , ads_mux   );
        value = (value >> 18) & 0x3fff;

        if (ads_mux == 0)
	{
			//update cold about every min
			if (ads_cold_cnt <1 )
            {
                ads_cold = (uint16_t)value * 0.05;
                //sendf("debug_ads_cold  value=%hu" , ads_cold   );
                ads_mux = 1;
                ads_cold_cnt = 31;
            }
            else
                ads_mux = 1;
            ads_cold_cnt--;
        }
        else if (ads_mux == 1)
        {
            if (0x2000 & value)
            {
		    // Negitive temp
                ads_t0 =(uint16_t)( ads_cold - (0x1fff & (~(value - 1))));
            } else
                ads_t0 = (uint16_t)(value + ads_cold);
            ads_mux = 2;
	}
        else if (ads_mux == 2)
        {
            if (0x2000 & value)
            {
		    // Negitive temp
                ads_t1 =(uint16_t)( ads_cold - (0x1fff & (~(value - 1))));
            } else
                ads_t1 = (uint16_t)(value + ads_cold);
            ads_mux = 0;
        }
        thermocouple_respond(spi, next_begin_time, ads_t0, 0, oid);
    }
}
static void
thermocouple_handle_ads1118B(struct thermocouple_spi *spi
        , uint32_t next_begin_time, uint8_t oid)
{
    thermocouple_respond(spi, next_begin_time, ads_t1, 0, oid);
    }


// task to read thermocouple and send response
void
thermocouple_task(void)
{
    if (!sched_check_wake(&thermocouple_wake))
        return;
    uint8_t oid;
    struct thermocouple_spi *spi;
    foreach_oid(oid, spi, command_config_thermocouple) {
        if (!(spi->flags & TS_PENDING))
            continue;
        irq_disable();
        uint32_t next_begin_time = spi->timer.waketime;
        spi->flags &= ~TS_PENDING;
        irq_enable();
        switch (spi->chip_type) {
        case TS_CHIP_MAX31855:
            thermocouple_handle_max31855(spi, next_begin_time, oid);
            break;
        case TS_CHIP_MAX31856:
            thermocouple_handle_max31856(spi, next_begin_time, oid);
            break;
        case TS_CHIP_MAX31865:
            thermocouple_handle_max31865(spi, next_begin_time, oid);
            break;
        case TS_CHIP_MAX6675:
            thermocouple_handle_max6675(spi, next_begin_time, oid);
            break;
        case TS_CHIP_ADS1118:
            thermocouple_handle_ads1118(spi, next_begin_time, oid);
            break;
        case TS_CHIP_ADS1118B:
            thermocouple_handle_ads1118B(spi, next_begin_time, oid);
            break;
        }
    }
}
DECL_TASK(thermocouple_task);
