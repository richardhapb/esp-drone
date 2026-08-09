/**
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (c) 2014, Bitcraze AB, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 *
 * i2c_drv.c - i2c driver implementation
 *
 * @note
 * For some reason setting CR1 reg in sequence with
 * I2C_AcknowledgeConfig(I2C_SENSORS, ENABLE) and after
 * I2C_GenerateSTART(I2C_SENSORS, ENABLE) sometimes creates an
 * instant start->stop condition (3.9us long) which I found out with an I2C
 * analyzer. This fast start->stop is only possible to generate if both
 * start and stop flag is set in CR1 at the same time. So i tried setting the CR1
 * at once with I2C_SENSORS->CR1 = (I2C_CR1_START | I2C_CR1_ACK | I2C_CR1_PE) and the
 * problem is gone. Go figure...
 */


#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"

#include "stm32_legacy.h"
#include "i2c_drv.h"
#include "config.h"
#define DEBUG_MODULE "I2CDRV"
#include "debug_cf.h"

// Definitions of sensors I2C bus
#define I2C_DEFAULT_SENSORS_CLOCK_SPEED             400000

// Definition of eeprom and deck I2C buss,use two i2c with 400Khz clock simultaneously could trigger the watchdog
#define I2C_DEFAULT_DECK_CLOCK_SPEED                100000

static bool isinit_i2cPort[2] = {0, 0};

// Cost definitions of busses
static const I2cDef sensorBusDef = {
    .i2cPort            = I2C_NUM_0,
    .i2cClockSpeed      = I2C_DEFAULT_SENSORS_CLOCK_SPEED,
    .gpioSCLPin         = CONFIG_I2C0_PIN_SCL,
    .gpioSDAPin         = CONFIG_I2C0_PIN_SDA,
    .gpioPullup         = GPIO_PULLUP_DISABLE,
};

I2cDrv sensorsBus = {
    .def                = &sensorBusDef,
};

static const I2cDef deckBusDef = {
    .i2cPort            = I2C_NUM_1,
    .i2cClockSpeed      = I2C_DEFAULT_DECK_CLOCK_SPEED,
    .gpioSCLPin         = CONFIG_I2C1_PIN_SCL,
    .gpioSDAPin         = CONFIG_I2C1_PIN_SDA,
    .gpioPullup         = GPIO_PULLUP_ENABLE,
};

I2cDrv deckBus = {
    .def                = &deckBusDef,
};

/**
 * Free a bus wedged by a slave holding SDA low.
 *
 * A slave that was mid-transfer when the master reset keeps driving SDA until
 * it has clocked out the rest of its byte. The MPU6050 has no reset pin, so a
 * CPU-only reset cannot clear this and the bus stays stuck until the board is
 * physically unpowered - which is what the sensor driver's "power off and
 * power on the device" message is really asking for.
 *
 * Recovery is the standard sequence: pulse SCL until the slave lets SDA go,
 * then issue a STOP so it returns to idle. Runs before the I2C peripheral
 * claims the pins.
 */
static void i2cdrvBusRecover(const I2cDef *def)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << def->gpioSCLPin) | (1ULL << def->gpioSDAPin),
        .mode         = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    gpio_set_level(def->gpioSDAPin, 1);
    gpio_set_level(def->gpioSCLPin, 1);
    esp_rom_delay_us(10);

    if (gpio_get_level(def->gpioSDAPin) == 1) {
        return;                                 /* bus already idle */
    }

    DEBUG_PRINTW("i2c %d: SDA held low, clocking the bus free", def->i2cPort);

    /* Nine pulses is one byte plus the ACK - enough for any slave to finish. */
    for (int i = 0; i < 9 && gpio_get_level(def->gpioSDAPin) == 0; i++) {
        gpio_set_level(def->gpioSCLPin, 0);
        esp_rom_delay_us(5);
        gpio_set_level(def->gpioSCLPin, 1);
        esp_rom_delay_us(5);
    }

    /* STOP: SDA low->high while SCL is high. */
    gpio_set_level(def->gpioSDAPin, 0);
    esp_rom_delay_us(5);
    gpio_set_level(def->gpioSCLPin, 1);
    esp_rom_delay_us(5);
    gpio_set_level(def->gpioSDAPin, 1);
    esp_rom_delay_us(5);

    DEBUG_PRINTI("i2c %d: after recovery SCL=%d SDA=%d", def->i2cPort,
                 gpio_get_level(def->gpioSCLPin),
                 gpio_get_level(def->gpioSDAPin));
}

static void i2cdrvInitBus(I2cDrv *i2c)
{
    if (isinit_i2cPort[i2c->def->i2cPort]) {
        return;
    }

    i2cdrvBusRecover(i2c->def);

    i2c_config_t conf = {0};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = i2c->def->gpioSDAPin;
    conf.sda_pullup_en = i2c->def->gpioPullup;
    conf.scl_io_num = i2c->def->gpioSCLPin;
    conf.scl_pullup_en = i2c->def->gpioPullup;
    conf.master.clk_speed = i2c->def->i2cClockSpeed;
    esp_err_t err = i2c_param_config(i2c->def->i2cPort, &conf);

    if (!err) {
        err = i2c_driver_install(i2c->def->i2cPort, conf.mode, 0, 0, 0);
    }

    DEBUG_PRINTI(" i2c %d driver install return = %d", i2c->def->i2cPort, err);
    i2c->isBusFreeMutex = xSemaphoreCreateMutex();
    isinit_i2cPort[i2c->def->i2cPort] = true;
}


//-----------------------------------------------------------

void i2cdrvInit(I2cDrv *i2c)
{
    i2cdrvInitBus(i2c);
}

void i2cdrvTryToRestartBus(I2cDrv *i2c)
{
    i2cdrvInitBus(i2c);
}

