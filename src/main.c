#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>

/* --- Our Header Files --- */
#include "bmv080.h"
#include "i2c_header.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* --- Devicetree and Global Variables --- */

#define DEBUG 1

#define BMV080_NODE DT_ALIAS(bmv080)
static const struct i2c_dt_spec bmv080_dev = I2C_DT_SPEC_GET(BMV080_NODE);

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static bmv080_handle_t bmv080_handle = NULL;
static volatile uint32_t data_ready_callback_count = 0;

/* --- Callback Functions for Bosch Driver --- */

static int8_t bmv080_i2c_read(bmv080_sercom_handle_t sercom_handle, uint8_t *data, uint32_t len)
{
    const struct i2c_dt_spec *dev = (const struct i2c_dt_spec *)sercom_handle;
    int32_t rc = i2c_driver_read(dev, data, len);
    return (rc == 0) ? E_BMV080_OK : E_BMV080_ERROR_HW_READ;
}

static int8_t bmv080_i2c_write(bmv080_sercom_handle_t sercom_handle, const uint8_t *data, uint32_t len)
{
    const struct i2c_dt_spec *dev = (const struct i2c_dt_spec *)sercom_handle;
    int32_t rc = i2c_driver_write(dev, data, len);
    return (rc == 0) ? E_BMV080_OK : E_BMV080_ERROR_HW_WRITE;
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

/* Print Sensor Data to Serial Terminal*/
static void print_sensor_output(bmv080_output_t bmv080_output, void *callback_parameters)
{
    data_ready_callback_count++;
    printk("Runtime: %.2fs, PM1: %.0f ug/m3, PM2.5: %.0f ug/m3, PM10: %.0f ug/m3, Obstructed: %s\n",
           bmv080_output.runtime_in_sec,
           bmv080_output.pm1_mass_concentration,
           bmv080_output.pm2_5_mass_concentration,
           bmv080_output.pm10_mass_concentration,
           (bmv080_output.is_obstructed ? "yes" : "no"));
}

int main(void)
{
    if (DEBUG) {
        printk("Attempting to configure LED.");
    }

    if (!gpio_is_ready_dt(&led) || gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) < 0) {
        return 0;
    }

    if (DEBUG) {
        printk("Configured LED.");
    }

    bmv080_status_code_t status = E_BMV080_OK;
    bmv080_status_code_t final_status = E_BMV080_OK;

    printk("\n--- BMV080 Concentration Measurement Example ---\n");

    if (!device_is_ready(bmv080_dev.bus)) {
        printk("I2C bus is not ready!\n");
        return 0;
    }

    /* Get Driver Version */
    uint16_t major = 0;
    uint16_t minor = 0;
    uint16_t patch = 0;
    status = bmv080_get_driver_version(&major, &minor, &patch, NULL, NULL);
    if (status != E_BMV080_OK) {
        printk("Failed to get driver version. Error: %d\n", status);
        final_status = status;
        goto exit_main;
    }
    printk("Bosch Driver Version: %d.%d.%d\n", major, minor, patch);

    /* Open Sensor */
    status = bmv080_open(&bmv080_handle, (bmv080_sercom_handle_t)&bmv080_dev, bmv080_i2c_read, bmv080_i2c_write, bmv080_delay_ms);
    if (status != E_BMV080_OK) {
        printk("Failed to open BMV080 sensor. Error: %d\n", status);
        final_status = status;
        goto exit_main;
    }
    printk("Sensor opened successfully.\n");

    /* Reset Sensor */
    status = bmv080_reset(bmv080_handle);
    if (status != E_BMV080_OK) {
        printk("Failed to reset BMV080 sensor. Error: %d\n", status);
        final_status = status;
        goto close_sensor;
    }
    printk("Sensor reset successfully.\n");

    /* Get Sensor ID */
    char id[13];
    memset(id, 0, sizeof(id));
    status = bmv080_get_sensor_id(bmv080_handle, id);
    if (status != E_BMV080_OK) {
        printk("Failed to get sensor ID. Error: %d\n", status);
        final_status = status;
        goto close_sensor;
    }
    printk("Sensor ID: %s\n\n", id);

    /* Set Measurement Algorithm
        1 = Fast Response
        2 = Balanced
        3 = High Precision
    */
    bmv080_measurement_algorithm_t measurement_algorithm = E_BMV080_MEASUREMENT_ALGORITHM_BALANCED;
    status = bmv080_set_parameter(bmv080_handle,"measurement_algorithm", (void*)&measurement_algorithm);
    if (status != E_BMV080_OK) {
        printk("Failed to set measurement algorithm. Error: %d\n", status);
        final_status = status;
        goto close_sensor;
    }
    printk("Measurement Algorithm: %d\n\n", measurement_algorithm);

    /* ================== Continuous Measurement ==================*/
    printk("=== Starting Continuous Measurement ===\n");
    status = bmv080_start_continuous_measurement(bmv080_handle);
    if (status != E_BMV080_OK) {
        printk("Failed to start continuous measurement. Error: %d\n", status);
        final_status = status;
        goto close_sensor;
    }

    /* For continuous_measurement_duration seconds, 
        -print output data
        -toggle the LED to indicate measuremeant
        -delay by 100 ms to increase accuracy
    */
    data_ready_callback_count = 0;
    uint32_t continuous_measurement_duration = 60;
    while (data_ready_callback_count < continuous_measurement_duration) {
        bmv080_serve_interrupt(bmv080_handle, print_sensor_output, NULL);
        gpio_pin_toggle_dt(&led);
        bmv080_delay_ms(100);
    }

    status = bmv080_stop_measurement(bmv080_handle);
    printk("=== Continuous Measurement Stopped ===\n\n");

    /* ================== Duty Cycling ==================*/
    printk("=== Starting Duty-Cycled Measurement ===\n");

    uint16_t duty_cycle_period = 20;
    status = bmv080_set_parameter(bmv080_handle, "duty_cycling_period", &duty_cycle_period);
    printk("Set duty cycle period to %u seconds\n", duty_cycle_period);

    bmv080_duty_cycling_mode_t duty_cycling_mode = E_BMV080_DUTY_CYCLING_MODE_0;
    status = bmv080_start_duty_cycling_measurement(bmv080_handle, bmv080_get_tick_ms, duty_cycling_mode);
     if (status != E_BMV080_OK) {
        printk("Failed to start duty-cycled measurement. Error: %d\n", status);
        final_status = status;
        goto close_sensor;
    }

    /* For duty_cycling_duration seconds, 
        -print output data
        -toggle the LED to indicate measuremeant
        -delay by 100 ms to increase accuracy
    */
    data_ready_callback_count = 0;
    uint32_t duty_cycling_duration = 60;
    while ((data_ready_callback_count * duty_cycle_period) < duty_cycling_duration) {
        bmv080_serve_interrupt(bmv080_handle, print_sensor_output, NULL);
        gpio_pin_toggle_dt(&led);
        bmv080_delay_ms(100);
    }

    status = bmv080_stop_measurement(bmv080_handle);
    printk("=== Duty-Cycled Measurement Stopped ===\n\n");

close_sensor:
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