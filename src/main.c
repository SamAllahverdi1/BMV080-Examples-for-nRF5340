#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h> // For memset
#include <zephyr/drivers/gpio.h> // LED config

// Bosch BMV080 C driver API
#include "bmv080.h"

#include "i2c_header.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* --- Devicetree and Global Variables --- */

// The devicetree node identifier for the BMV080 sensor.
#define BMV080_NODE DT_ALIAS(bmv080)
static const struct i2c_dt_spec bmv080_dev = I2C_DT_SPEC_GET(BMV080_NODE);

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// Global handle for the BMV080 sensor instance
static bmv080_handle_t bmv080_handle = NULL;
static volatile uint32_t data_ready_callback_count = 0;

/* --- Callback Functions for Bosch Driver --- */

// Bridge between the Bosch driver and the Zephyr I2C API.
static int8_t bmv080_i2c_read(bmv080_sercom_handle_t sercom_handle, uint16_t reg_addr, uint16_t *data, uint16_t len)
{
    const struct i2c_dt_spec *dev = (const struct i2c_dt_spec *)sercom_handle;
    uint8_t reg_addr_swapped[2] = {(uint8_t)(reg_addr >> 8), (uint8_t)reg_addr};

    if (i2c_write_read_dt(dev, reg_addr_swapped, sizeof(reg_addr_swapped), (uint8_t *)data, len * 2) != 0) {
        return E_BMV080_ERROR_HW_READ;
    }
    return E_BMV080_OK;
}

static int8_t bmv080_i2c_write(bmv080_sercom_handle_t sercom_handle, uint16_t reg_addr, const uint16_t *data, uint16_t len)
{
    const struct i2c_dt_spec *dev = (const struct i2c_dt_spec *)sercom_handle;
    uint8_t buf[2 + (len * 2)];
    buf[0] = (uint8_t)(reg_addr >> 8);
    buf[1] = (uint8_t)reg_addr;
    memcpy(&buf[2], data, len * 2);

    if (i2c_write_dt(dev, buf, sizeof(buf)) != 0) {
        return E_BMV080_ERROR_HW_WRITE;
    }
    return E_BMV080_OK;
}

static int8_t bmv080_delay_ms(uint32_t period_ms)
{
    k_msleep(period_ms);
    return E_BMV080_OK;
}

static uint32_t bmv080_get_tick_ms(void)
{
    return k_uptime_get_32();
}

// Callback that receives and prints the sensor data.
static void use_sensor_output(bmv080_output_t bmv080_output, void *callback_parameters)
{
    data_ready_callback_count++;
    printk("Runtime: %.2fs, PM1: %.0f ug/m3, PM2.5: %.0f ug/m3, PM10: %.0f ug/m3, Obstructed: %s\n",
           bmv080_output.runtime_in_sec,
           bmv080_output.pm1_mass_concentration,
           bmv080_output.pm2_5_mass_concentration,
           bmv080_output.pm10_mass_concentration,
           (bmv080_output.is_obstructed ? "yes" : "no"));
}

/* --- Main Application --- */

int main(void)
{

    if (!gpio_is_ready_dt(&led)) {
        // If this fails, something is very wrong with the board config
        return 0;
    }
    if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) < 0) {
        return 0;
    }

    bmv080_status_code_t status = E_BMV080_OK;
    bmv080_status_code_t final_status = E_BMV080_OK;

    printk("\n--- BMV080 Zephyr Example ---\n");

    // Check if the I2C device from the devicetree is ready
    if (!device_is_ready(bmv080_dev.bus)) {
        printk("I2C bus is not ready!\n");
        return 0;
    }

    // 1. Get Driver Version
    uint16_t major = 0, minor = 0, patch = 0;
    status = bmv080_get_driver_version(&major, &minor, &patch, NULL, NULL);
    if (status != E_BMV080_OK) {
        printk("Failed to get driver version. Error: %d\n", status);
        final_status = status;
        goto exit_main;
    }
    printk("Bosch Driver Version: %d.%d.%d\n", major, minor, patch);

    // 2. Open Sensor
    status = bmv080_open(&bmv080_handle, (bmv080_sercom_handle_t)&bmv080_dev, bmv080_i2c_read, bmv080_i2c_write, bmv080_delay_ms);
    if (status != E_BMV080_OK) {
        printk("Failed to open BMV080 sensor. Error: %d\n", status);
        final_status = status;
        goto exit_main;
    }
    printk("Sensor opened successfully.\n");

    // 3. Reset Sensor
    status = bmv080_reset(bmv080_handle);
    if (status != E_BMV080_OK) {
        printk("Failed to reset BMV080 sensor. Error: %d\n", status);
        final_status = status;
        goto close_sensor;
    }
    printk("Sensor reset successfully.\n");

    // 4. Get Sensor ID
    char id[13];
    memset(id, 0, sizeof(id));
    status = bmv080_get_sensor_id(bmv080_handle, id);
    if (status != E_BMV080_OK) {
        printk("Failed to get sensor ID. Error: %d\n", status);
        final_status = status;
        goto close_sensor;
    }
    printk("Sensor ID: %s\n\n", id);

    // 5. Continuous Measurement Example (30 seconds)
    printk("--- Starting Continuous Measurement (30 seconds) ---\n");
    status = bmv080_start_continuous_measurement(bmv080_handle);
    if (status != E_BMV080_OK) {
        printk("Failed to start continuous measurement. Error: %d\n", status);
        final_status = status;
        goto close_sensor;
    }

    data_ready_callback_count = 0;
    while (data_ready_callback_count < 30) {
        bmv080_serve_interrupt(bmv080_handle, use_sensor_output, NULL);
        gpio_pin_toggle_dt(&led);
        k_msleep(100); // Poll every 100ms
    }

    status = bmv080_stop_measurement(bmv080_handle);
    printk("--- Continuous Measurement Stopped ---\n\n");

    // 6. Duty-Cycled Measurement Example (60 seconds)
    printk("--- Starting Duty-Cycled Measurement (60 seconds) ---\n");
    uint16_t duty_cycle_period = 20; // 20 seconds between measurements
    status = bmv080_set_parameter(bmv080_handle, "duty_cycling_period", &duty_cycle_period);
    printk("Set duty cycle period to %u seconds\n", duty_cycle_period);

    status = bmv080_start_duty_cycling_measurement(bmv080_handle, bmv080_get_tick_ms, E_BMV080_DUTY_CYCLING_MODE_0);
     if (status != E_BMV080_OK) {
        printk("Failed to start duty-cycled measurement. Error: %d\n", status);
        final_status = status;
        goto close_sensor;
    }

    data_ready_callback_count = 0;
    while ((data_ready_callback_count * duty_cycle_period) < 60) {
         bmv080_serve_interrupt(bmv080_handle, use_sensor_output, NULL);
         k_msleep(100); // Poll every 100ms
    }

    status = bmv080_stop_measurement(bmv080_handle);
    printk("--- Duty-Cycled Measurement Stopped ---\n\n");


close_sensor:
    // 7. Close Sensor
    status = bmv080_close(&bmv080_handle);
    if (status != E_BMV080_OK) {
        printk("Failed to close BMV080 sensor. Error: %d\n", status);
        if (final_status == E_BMV080_OK) final_status = status;
    }
    printk("Sensor closed.\n");

exit_main:
    printk("Example finished with final status code: %d\n", final_status);
    return 0;
}