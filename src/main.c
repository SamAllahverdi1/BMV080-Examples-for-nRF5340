#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <stdint.h>

// SH2 driver
#include <sh2.h>
#include <euler.h>
#include <sh2_err.h>
#include <sh2_hal.h>
#include <sh2_SensorValue.h>
#include <sh2_util.h>
#include <shtp.h>

// I2C
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
//#include <sh2_hal.c>

LOG_MODULE_REGISTER(bno086main, LOG_LEVEL_INF); // for debugging

// Node identifiers
#define BOOTN_NODE 	DT_NODELABEL(bootn) // P1.10
#define RESETN_NODE DT_NODELABEL(resetn) // P1.12
#define SA0_NODE DT_NODELABEL(sa0) // P1.13
#define INTN_NODE 	DT_NODELABEL(intn) //P1.14
#define WAKE_NODE DT_NODELABEL(wake) //P0.27
#define PS0_NODE 	DT_NODELABEL(ps0) //P0.26
#define PS1_NODE DT_NODELABEL(ps1) //P0.25

#define I2C_MAX_BUFFER_SIZE 256
// The following returns the structure i2c_dt_spec, which contains the device pointer for the I2C bus as well as the target bus. 
//SDA P1.02
//SCK P1.03
static const struct i2c_dt_spec i2c_dev = I2C_DT_SPEC_GET(DT_NODELABEL(bno086)); //

// The following pins might be needed to work with the IMU.
static const struct gpio_dt_spec bootn = GPIO_DT_SPEC_GET(BOOTN_NODE, gpios);
static const struct gpio_dt_spec resetn = GPIO_DT_SPEC_GET(RESETN_NODE, gpios);
static const struct gpio_dt_spec intn = GPIO_DT_SPEC_GET(INTN_NODE, gpios);
static const struct gpio_dt_spec wake = GPIO_DT_SPEC_GET(WAKE_NODE, gpios);
static const struct gpio_dt_spec ps0 = GPIO_DT_SPEC_GET(PS0_NODE, gpios);
static const struct gpio_dt_spec ps1 = GPIO_DT_SPEC_GET(PS1_NODE, gpios);
static const struct gpio_dt_spec sa0 = GPIO_DT_SPEC_GET(SA0_NODE, gpios);

// Function prototypes
static int i2chal_open(sh2_Hal_t *self);
static void i2chal_close(sh2_Hal_t *self);
static int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);

float q_to_yaw(float r, float i, float j, float k);
float q_to_pitch(float r, float i, float j, float k);
float q_to_roll(float r, float i, float j, float k);
void q_to_ypr(float r, float i, float j, float k, float *pYaw, float *pPitch, float *pRoll);
/****** I2C INTERFACE ******/
// This function initializes communications with the device.  It
// can initialize any GPIO pins and peripheral devices used to
// interface with the sensor hub.
// It should also perform a reset cycle on the sensor hub to
// ensure communications start from a known state.

static int i2chal_open(sh2_Hal_t *self) {
	LOG_DBG("i2chal_open enter.");
	static uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};

	//uint8_t read_buf[5];
	bool success = false;
	if (i2c_is_ready_dt(&i2c_dev)){
		printk("I2C bus is  ready.\n");
	}
	LOG_DBG("i2chal_open device available.");
	LOG_DBG("Using I2C address: 0x%X", i2c_dev.addr);

for (uint8_t attempts = 0; attempts < 5; attempts++) {
    LOG_DBG("i2chal_open write attempt %d.", attempts);
    int ret = i2c_write_dt(&i2c_dev, softreset_pkt, 5);
    //int ret = i2c_read_dt(&i2c_dev, read_buf, sizeof(read_buf));
    
    if (!ret) {
        success = true;
        break;
    } else {
        LOG_ERR("i2c_write_dt failed with error code %d on attempt %d", ret, attempts);
    }
}

	if (!success) return -1;
	k_sleep(K_MSEC(300));
	
	LOG_DBG("i2chal_open exit success.");
	return 0;
}
// This function completes communications with the sensor hub.
// It should put the device in reset then de-initialize any
// peripherals or hardware resources that were used.
static void i2chal_close(sh2_Hal_t *self) { 
    // Nothing to do, as Zephyr handles I2C cleanup automatically
}

// This function supports reading data from the sensor hub.
// It will be called frequently to service the device.
//
// If the HAL has received a full SHTP transfer, this function
// should load the data into pBuffer, set the timestamp to the
// time the interrupt was detected, and return the non-zero length
// of data in this transfer.
//
// If the HAL has not recevied a full SHTP transfer, this function
// should return 0.
//
// Because this function is called regularly, it can be used to
// perform other housekeeping operations.  (In the case of UART
// interfacing, bytes transmitted are staggered in time and this
// function can be used to keep the transmission flowing.)
static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
  LOG_DBG("i2chal_read enter.");

  uint8_t header[4];
  if (i2c_read_dt(&i2c_dev, header, 4)) {
    return 0;
  }

  // Determine amount to read
  uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
  // Unset the "continue" bit
  packet_size &= ~0x8000;

  //  LOG_DBG("Read SHTP header. Packet size: %d, buffer size: %d.", packet_size, len);

  size_t i2c_buffer_max = I2C_MAX_BUFFER_SIZE;

  if (packet_size > len) {
    // packet wouldn't fit in our buffer
    return 0;
  }
  // the number of non-header bytes to read
  uint16_t cargo_remaining = packet_size;
  static uint8_t i2c_buffer[I2C_MAX_BUFFER_SIZE];
  uint16_t read_size;
  uint16_t cargo_read_amount = 0;
  bool first_read = true;

  while (cargo_remaining > 0) {
    if (first_read) {
      read_size = MIN(i2c_buffer_max, (size_t)cargo_remaining);
    } else {
      read_size = MIN(i2c_buffer_max, (size_t)cargo_remaining + 4);
    }

        LOG_DBG("Reading from I2C: %d. Remaining to read: %d", read_size, cargo_remaining);

    if (i2c_read_dt(&i2c_dev, i2c_buffer, read_size)) {
      return 0;
    }

    if (first_read) {
      // The first time we're saving the "original" header, so include it in the
      // cargo count
      cargo_read_amount = read_size;
      memcpy(pBuffer, i2c_buffer, cargo_read_amount);
      first_read = false;
    } else {
      // this is not the first read, so copy from 4 bytes after the beginning of
      // the i2c buffer to skip the header included with every new i2c read and
      // don't include the header in the amount of cargo read
      cargo_read_amount = read_size - 4;
      memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
    }
    // advance our pointer by the amount of cargo read
    pBuffer += cargo_read_amount;
    // mark the cargo as received
    cargo_remaining -= cargo_read_amount;
  }
  LOG_DBG("i2chal_read exit.");
  return packet_size;
}
static int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  LOG_DBG("i2chal_write enter.");
  size_t i2c_buffer_max = I2C_MAX_BUFFER_SIZE;

  LOG_DBG("Write packet size I2C: %d. max buffer size: %d", len, i2c_buffer_max);

  uint16_t write_size = MIN(i2c_buffer_max, len);
  if (i2c_write_dt(&i2c_dev, pBuffer, write_size)) {
    return 0;
  }

  return write_size;
  LOG_DBG("i2chal_write exit.");
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
  int64_t t = k_uptime_get() * 1000;
  LOG_DBG("hal_getTimeUs %llu", t);
  return (uint32_t)t;

}
/************ END I2C INTERFACE ************/


/*** I referenced the following code from this source:
https://github.com/jens-ajisai/gently-communicating-home-using-Matter/blob/main/postureChecker/src/sensors/driver/bno08x.c
***/

//Initializes the sensor with basic settings using I2C
//Returns false if sensor is not detected
sh2_Hal_t _HAL = {.open = i2chal_open,
                  .close = i2chal_close,
                  .read = i2chal_read,
                  .write = i2chal_write,
                  .getTimeUs = hal_getTimeUs};
                  
// I think this resets tracking and sensor data
static sh2_SensorValue_t *_sensor_value = NULL;
static bool _reset_occurred = false;

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
  if (pEvent->eventId == SH2_RESET) {
    LOG_DBG("BNO08X Reset");
    _reset_occurred = true;
  }
}

static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
  int rc;

  
  rc = sh2_decodeSensorEvent(_sensor_value, event);
  if (rc != SH2_OK) {
    LOG_ERR("BNO08x - Error decoding sensor event");
    _sensor_value->timestamp = 0;
    return;
  }
}

// Note: There is a function called BNO08x::begin from the SparkFun library, but it really just performs some setup. This init() function is actually where things happen. 

bool bno08x_init(sh2_ProductIds_t *prodIds) {
  int status;

  LOG_DBG("bno08x_init enter.");

  status = sh2_open(&_HAL, hal_callback, NULL);
  if (status != SH2_OK) {
    return false;
  }

  if (prodIds != NULL) {
    memset(prodIds, 0, sizeof(sh2_ProductIds_t));
    status = sh2_getProdIds(prodIds);
    if (status != SH2_OK) {
      return false;
    }
  }
  sh2_setSensorCallback(sensorHandler, NULL);

  LOG_DBG("bno08x_init exit.");
  return true;
}

bool bno08x_enableReport(sh2_SensorId_t sensorId, uint32_t interval_us) {
  static sh2_SensorConfig_t config;

  // These sensor options are disabled or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = 0;

  config.reportInterval_us = interval_us;
  int status = sh2_setSensorConfig(sensorId, &config);

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

bool bno08x_wasReset(void) {
  bool x = _reset_occurred;
  _reset_occurred = false;

  return x;
}

bool bno08x_getSensorEvent(sh2_SensorValue_t *value) {
  _sensor_value = value;

  value->timestamp = 0;

  sh2_service();

  if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV) {
    // no new events
    return false;
  }

  return true;
}
/*** END of bno08x.c code ***/



void set_bootloader_mode() {
    int ret;
	printf("Checking if GPIOs are ready...\n");
 	if (!gpio_is_ready_dt(&bootn)) {
		return;
	}

 	if (!gpio_is_ready_dt(&resetn)) {
		return;
	}
	if (!gpio_is_ready_dt(&ps0)) {
		return;
	}

 	if (!gpio_is_ready_dt(&ps1)) {
		return;
	}
    printf("GPIOs are ready. Configuring pins...\n");

    // Configure BOOTN as output
    ret = gpio_pin_configure_dt(&bootn, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printf("Failed to configure BOOTN\n");
        return;
    }

    // Configure RESET as output
    ret = gpio_pin_configure_dt(&resetn, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printf("Failed to configure RESETN\n");
        return;
    }
    ret = gpio_pin_configure_dt(&ps0, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printf("Failed to configure BOOTN\n");
        return;
    }

    // Configure RESET as output
    ret = gpio_pin_configure_dt(&ps1, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printf("Failed to configure RESETN\n");
        return;
    }
    printf("GPIOs are ready. Initiating bootloader to initialize SPI...\n");
    // Step 1: Set BOOTN LOW (Enter Bootloader Mode)
    gpio_pin_set_dt(&bootn, 1);
    
    // Step 2: Reset the BNO08X
    // I have duplicates because I wasn't sure when it is in reset mode. Will fix. 
    gpio_pin_set_dt(&ps0, 0);
    gpio_pin_set_dt(&ps1, 0);
    gpio_pin_set_dt(&resetn, 1);
    k_sleep(K_MSEC(10));  // Wait 10ms
    gpio_pin_set_dt(&ps0, 0);
    gpio_pin_set_dt(&ps1, 0);
    gpio_pin_set_dt(&resetn, 0);
    k_sleep(K_MSEC(100)); // Wait 100ms
    gpio_pin_set_dt(&ps0, 0);
    gpio_pin_set_dt(&ps1, 0);
    printf("BNO08X should now be in bootloader mode\n");
    return;
}

int main(void) {
	int ret;
 	if (!gpio_is_ready_dt(&sa0)) {
		return;
	}
    printf("GPIOs are ready. Configuring pins...\n");

    // Configure BOOTN as output
    ret = gpio_pin_configure_dt(&sa0, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printf("Failed to configure BOOTN\n");
        return;
    }

 
/*
	set_bootloader_mode();
	gpio_pin_set_dt(&bootn, 0);
    printf("Bootloader finished.\n");
	k_sleep(K_MSEC(100));
	printk("Starting tests...\n");
 
	printk("Tests passed. Reading from IMU...\n");
*/	
    // Initialize BNO086 IMU
    // maybe this will work 
    printk("here1\n");
    sh2_ProductIds_t prodIds;
    printk("here1.5\n");
    if (bno08x_init(&prodIds)) {
        printf("BNO08x_init function returns true\n");
    }

    	
    	
    
    // Enable Rotation Vector Report
    printk("here2\n");
    if (bno08x_enableReport(SH2_ARVR_STABILIZED_RV, 500*1000)) {
        printf("Enabled rotation vector report\n");
    }
    
    /*
    if (bno08x_enableReport(SH2_ACCELEROMETER, 500*1000)) {
        printf("Enabled  accelerometer report\n");
    }*/
    printk("here2.5\n");
    // Retrieve Sensor Event
    sh2_SensorValue_t sensorValue;
    printk("here3\n");
    

while (1) {
		if (bno08x_getSensorEvent(&sensorValue)) {
		printk("Sensor ID: %d\n", sensorValue.sensorId);

    float roll, pitch, yaw;
    q_to_ypr(sensorValue.un.arvrStabilizedRV.real, sensorValue.un.arvrStabilizedRV.i,
             sensorValue.un.arvrStabilizedRV.j, sensorValue.un.arvrStabilizedRV.k,
             &yaw, &pitch, &roll);
		//printk("llinear accelerometer - x: %f y: %f z: %f\n",sensorValue.un.linearAcceleration.x,  sensorValue.un.linearAcceleration.y, sensorValue.un.linearAcceleration.z); 
    // Print Euler Angles
    printk("Roll: %f, Pitch: %f, Yaw: %f, Accuracy: %f\n", (roll*180/3.14), pitch*180/3.14, yaw*180/3.14, sensorValue.un.arvrStabilizedRV.accuracy);

	}

}
    return 0;
}


