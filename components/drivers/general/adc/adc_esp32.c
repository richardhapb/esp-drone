/**
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * adc.c - Analog Digital Conversion
 *
 *
 */

#include "esp_idf_version.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "adc_esp32.h"
#include "config.h"
#include "pm_esplane.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "ADC"
#include "debug_cf.h"

static bool isInit;

static adc_oneshot_unit_handle_t adcHandle;
static adc_cali_handle_t caliHandle;
static bool isCalibrated;

#ifdef CONFIG_IDF_TARGET_ESP32
static const adc_channel_t channel = ADC_CHANNEL_7; //GPIO35 if ADC1
#elif defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
static const adc_channel_t channel = ADC_CHANNEL_1;     // GPIO2 if ADC1
#endif

static const adc_bitwidth_t width = ADC_BITWIDTH_DEFAULT;
static const adc_atten_t atten = ADC_ATTEN_DB_12;
static const adc_unit_t unit = ADC_UNIT_1;
#define DEFAULT_VREF 1100 //Used only when the chip carries no calibration eFuse
#define NO_OF_SAMPLES   30          //Multisampling

/**
 * Set up a calibration scheme for the configured unit/channel/attenuation.
 * Line fitting is what the ESP32 supports; the S2/S3 use curve fitting.
 * Calibration is optional: without it we fall back to a raw approximation.
 */
static bool adcCalibrationInit(void)
{
    esp_err_t err;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t caliConfig = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = width,
    };
    err = adc_cali_create_scheme_curve_fitting(&caliConfig, &caliHandle);
    if (err == ESP_OK) {
        DEBUG_PRINTI("ADC calibrated using curve fitting");
        return true;
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t caliConfig = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = width,
        .default_vref = DEFAULT_VREF,
    };
    err = adc_cali_create_scheme_line_fitting(&caliConfig, &caliHandle);
    if (err == ESP_OK) {
        DEBUG_PRINTI("ADC calibrated using line fitting");
        return true;
    }
#else
    err = ESP_ERR_NOT_SUPPORTED;
#endif

    DEBUG_PRINTW("ADC calibration unavailable (err = %d), using raw approximation", err);
    return false;
}

float analogReadVoltage(uint32_t pin)
{
    uint32_t adc_reading = 0;

    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        int raw = 0;

        if (adc_oneshot_read(adcHandle, channel, &raw) == ESP_OK) {
            adc_reading += raw;
        }
    }

    adc_reading /= NO_OF_SAMPLES;

    //Convert adc_reading to voltage in mV
    int voltage = 0;

    if (isCalibrated && adc_cali_raw_to_voltage(caliHandle, (int)adc_reading, &voltage) == ESP_OK) {
        return voltage / 1000.0f;
    }

    // Uncalibrated fallback: assume a full-scale 12 bit conversion at ~2.5V.
    return (adc_reading * 2500.0f / 4095.0f) / 1000.0f;
}

void adcInit(void)
{
    if (isInit) {
        return;
    }

    adc_oneshot_unit_init_cfg_t unitConfig = {
        .unit_id = unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unitConfig, &adcHandle));

    adc_oneshot_chan_cfg_t channelConfig = {
        .atten = atten,
        .bitwidth = width,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adcHandle, channel, &channelConfig));

    isCalibrated = adcCalibrationInit();

    isInit = true;
}

bool adcTest(void)
{
    return isInit;
}
