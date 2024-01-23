// Support for the ADS1263 ADC sensor chip
//
// Copyright (C) 2023  Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "board/misc.h" // timer_read_time
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "spicmds.h" // spidev_transfer
#include "sensor_ads1263.h" // ads1263_query

#define MAX_SPI_READ_TIME timer_from_us(50)

uint8_t
ads1263_is_ready(struct ads1263_sensor *ads) {
    // if the pin is low the sample is ready
    return !gpio_in_read(ads->drdy);
}

// ads1263 sensor query
void
ads1263_query(struct ads1263_sensor *ads, struct mux_adc_sample *sample) {
    // if the pin is high the sample is not ready
    if (gpio_in_read(ads->drdy)) {
        sample->sample_not_ready = 1;
        return;
    }

    uint8_t msg[7] = { 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint32_t measurement_time = timer_read_time();
    spidev_transfer(ads->spi, 1, sizeof(msg), msg);
    uint32_t read_end_time = timer_read_time();

    // check the status byte to see if the chip reset
    if ((msg[1] & 0x1) == 1) {
        shutdown("ADS1263 Reported an unexpected reset while sampling!");
    }
    // check the status byte to see if the data is fresh, if not ignore
    else if ((msg[1] & 0x40) == 0) {
        sample->sample_not_ready = 1;
        return;
    }
    // check for a timing error
    else if ((read_end_time - measurement_time) > MAX_SPI_READ_TIME) {
        // if this happens mcu is probably overloaded
        shutdown("ADS1263 read timing error, read took too long");
    }

    // TODO: perform CRC check
    // uint8_t crc = msg[6]
    // else if (!test_crc(msg, crc)) {
    //   sample->crc_error = 1;
    //   return;
    // }

    sample->counts = (msg[2] << 24) | (msg[3] << 16) | (msg[4] << 8) | msg[5];
    sample->measurement_time = measurement_time;
}

void
command_config_ads1263(uint32_t *args)
{
    struct ads1263_sensor *ads = oid_alloc(args[0]
                                    , command_config_ads1263, sizeof(*ads));
    
    ads->spi = spidev_oid_lookup(args[1]);
    if (!spidev_have_cs_pin(ads->spi))
        shutdown("ADS 1263 sensor requires cs pin");
    ads->drdy = gpio_in_setup(args[2], -1);
}
DECL_COMMAND(command_config_ads1263, "config_ads1263 oid=%c spi_oid=%c" \
                                    " drdy_pin=%u");

// Lookup a look up an ads1263 sensor instance by oid
struct ads1263_sensor *
ads1263_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_ads1263);
}