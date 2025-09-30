# Zephyr and Bosch BMV080 Particulate Matter Sensor Example

### An example application for interfacing with the Bosch BMV080 particulate matter sensor on the nRF5340 DK.

![Zephyr](https://img.shields.io/badge/Zephyr-RTOS-blue) ![Hardware](https://img.shields.io/badge/Hardware-nRF5340_DK-lightgrey)

## About The Project

This repository contains a standalone Zephyr application for the nRF5340 DK that demonstrates how to interface with the Bosch BMV080 particulate matter sensor. It uses the official pre-compiled C libraries from Bosch and showcases a structured, real-world approach to initializing, configuring, and reading data from the sensor.

The application's core logic is contained within `main.c`, which follows the recommended workflow from the official sensor documentation. It initializes the sensor, performs a reset, reads the unique device ID, and then runs two measurement cycles: one in continuous mode and one in duty-cycled mode. All status updates and sensor readings are printed to a serial terminal for monitoring.

## Features

* **I²C Communication**: Interfaces with the BMV080 sensor over the I²C protocol.
* **Official Library Integration**: Links and utilizes the official Bosch pre-compiled static libraries (`lib_bmv080.a`, `lib_postProcessor.a`).
* **Hardware Abstraction**: Implements a C-based hardware abstraction layer (HAL) to connect Zephyr's I²C and kernel APIs with the Bosch driver's function pointer callbacks.
* **Complete Sensor Workflow**: Follows the complete, recommended workflow for the sensor, including initialization, reset, ID check, and data acquisition.
* **Multiple Measurement Modes**: Runs demonstrations in both **continuous mode** and **duty-cycled mode**.
* **Serial Output**: Outputs formatted sensor data and status updates to a serial console.

## Built With

* C Programming Language
* Zephyr RTOS
* nRF Connect SDK
* Bosch BMV080 Official C Libraries

## Getting Started

To get a local copy up and running, follow these simple steps.

### Prerequisites

**Hardware:**
* Nordic nRF5340 DK
* A breakout board for the Bosch BMV080 Particulate Matter Sensor
* Jumper wires and a USB cable

**Software:**
* nRF Connect SDK (v3.1.0 or later)
* Visual Studio Code with the **nRF Connect for VS Code** extension pack
* A serial terminal program (e.g., Tera Term, PuTTY)

### Installation & Setup

1.  **Clone the repository:**
    ```bash
    git clone <your-repo-url>
    ```

2.  **Hardware Connection:**
    Connect the BMV080 sensor to your nRF5340 DK. The project is configured to use the **I2C2** peripheral.

    | nRF5340 DK Pin | BMV080 Pin | Function  |
    | :------------- | :--------- | :-------- |
    | **P1.02**      | `SDA`      | I2C Data  |
    | **P1.03**      | `SCL`      | I2C Clock |
    | `VDD`          | `VCC`      | Power     |
    | `GND`          | `GND`      | Ground    |

    > **Note:** Ensure your sensor's hardware pins (CSB and MISO) are configured to select the I²C address **`0x57`**.

3.  **Build the Firmware:**
    * Open the project folder in VS Code.
    * In the **nRF Connect** extension sidebar, click on **"Click to create a new build"**.
    * Select the board **`nrf5340dk/nrf5340/cpuapp`**.
    * After the initial configuration is complete, click the **Build** button.

4.  **Flash the Firmware:**
    * Connect your nRF5340 DK to your computer.
    * Click the **Flash** button in the nRF Connect extension view to program the board.

## Viewing the Output

Connect to the board's serial port using a terminal program with the following settings:
* **Baud rate**:    `115200`
* **Data bits**:    `8`
* **Parity**:       `none`
* **Stop bits**:    `1`
* **Flow control**: `none`

After resetting the board, you will see the following output:

```bash
--- BMV080 Zephyr Example ---
Bosch Driver Version: 1.0.16
Sensor opened successfully.
Sensor reset successfully.
Sensor ID: [Your Sensor's ID Will Appear Here]

--- Starting Continuous Measurement (30 seconds) ---
Runtime: 1.00s, PM2.5: 12 ug/m3, PM10: 15 ug/m3, Obstructed: no
Runtime: 2.00s, PM2.5: 12 ug/m3, PM10: 16 ug/m3, Obstructed: no
...
--- Continuous Measurement Stopped ---

--- Starting Duty-Cycled Measurement (60 seconds) ---
Set duty cycle period to 20 seconds
Runtime: 10.00s, PM2.5: 11 ug/m3, PM10: 14 ug/m3, Obstructed: no
Runtime: 30.00s, PM2.5: 11 ug/m3, PM10: 14 ug/m3, Obstructed: no
Runtime: 50.00s, PM2.5: 12 ug/m3, PM10: 15 ug/m3, Obstructed: no
--- Duty-Cycled Measurement Stopped ---

Sensor closed.
Example finished with final status code: 0
```

## Contact

Sam Allahverdi - <sam.allahverdi1@gmail.com>

Project Link: [https://github.com/SamAllahverdi1/simple-PM-sensor](https://https://github.com/SamAllahverdi1/simple-PM-sensor)

## License

Distributed under no license.