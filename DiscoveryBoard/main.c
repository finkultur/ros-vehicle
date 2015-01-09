/*
   ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
   2011,2012 Giovanni Di Sirio.

   This file is part of ChibiOS/RT.

   ChibiOS/RT is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   ChibiOS/RT is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

	Edited by: ...
	
   */

#include "ch.h"
#include "hal.h"
#include "test.h"
#include "spi.h"
#include "i2c.h"
#include "usbcfg.h"
#include "chprintf.h"

#define usb_lld_connect_bus(usbp)
#define usb_lld_disconnect_bus(usbp)

#define number_of_samples 2048
#define sample_freq 40 // hz

/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;

static float mdps_per_digit = 8.75;


typedef enum 
{
  LED3 = GPIOE_LED3_RED,
  LED4 = GPIOE_LED4_BLUE,
  LED5 = GPIOE_LED5_ORANGE,
  LED6 = GPIOE_LED6_GREEN,
  LED7 = GPIOE_LED7_GREEN,
  LED8 = GPIOE_LED8_ORANGE,
  LED9 = GPIOE_LED9_BLUE,
  LED10 = GPIOE_LED10_RED
} LedIO;

uint8_t led_counter = 0;
static uint8_t LED[] = {LED3, LED4, LED5, LED6, LED7, LED8, LED9, LED10};


static const SPIConfig spi1cfg = {
  NULL,
  /* HW dependent part.*/
  GPIOE,
  GPIOE_SPI1_CS,
  SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA,
  0
};

static const I2CConfig i2cconfig = {
  0x00902025, //voodoo magic
  0,
  0
};
 

/*static uint8_t readByteSPI(uint8_t reg)
{
	char txbuf[2] = {0x80 | reg, 0xFF};
	char rxbuf[2];
	spiSelect(&SPID1);
	spiExchange(&SPID1, 2, txbuf, rxbuf);
	spiUnselect(&SPID1);
	return rxbuf[1];
}*/
static uint8_t writeByteSPI(uint8_t reg, uint8_t val)
{
	char txbuf[2] = {reg, val};
	char rxbuf[2];
	spiSelect(&SPID1);
	spiExchange(&SPID1, 2, txbuf, rxbuf);
	spiUnselect(&SPID1);
	return rxbuf[1];
}

/*static uint8_t readByteI2C(uint8_t addr, uint8_t reg)
{
    uint8_t data;
    i2cAcquireBus(&I2CD1);
    (void)i2cMasterTransmitTimeout(&I2CD1, addr, &reg, 1, &data, 1, TIME_INFINITE);
    i2cReleaseBus(&I2CD1);
    return data;
}*/
static void writeByteI2C(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t cmd[] = {reg, val};
    i2cAcquireBus(&I2CD1);
    (void)i2cMasterTransmitTimeout(&I2CD1, addr, cmd, 2, NULL, 0, TIME_INFINITE);
    i2cReleaseBus(&I2CD1);
}

static void initGyro(void)
{
    /* see the L3GD20 Datasheet */
    writeByteSPI(0x20, 0xcF);
}
static void initAccel(void)
{
    // Highest speed, enable all axes
    writeByteI2C(0x19, 0x20, 0x97);
}
static void initMag(void)
{
    // Highest speed
    writeByteI2C(0x1E, 0x00, 0x1C);
    writeByteI2C(0x1E, 0x02, 0x00);
}
static uint8_t readGyro(float* data)
{
    /* read from L3GD20 registers and assemble data */
    /* 0xc0 sets read and address increment */
    char txbuf[8] = {0xc0 | 0x27, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    char rxbuf[8];
    spiSelect(&SPID1);
    spiExchange(&SPID1, 8, txbuf, rxbuf);
    spiUnselect(&SPID1);
    if (rxbuf[1] & 0x7) {
        int16_t val_x = (rxbuf[3] << 8) | rxbuf[2];
        int16_t val_y = (rxbuf[5] << 8) | rxbuf[4];
        int16_t val_z = (rxbuf[7] << 8) | rxbuf[6];
        data[0] = (((float)val_x) * mdps_per_digit)/1000.0;
        data[1] = (((float)val_y) * mdps_per_digit)/1000.0;
        data[2] = (((float)val_z) * mdps_per_digit)/1000.0;
        return 1;
    }
    return 0;
}
static uint8_t readAccel(float* data)
{
    // setting MSB makes it increment the address for a multiple byte read
    uint8_t start_reg = 0x27 | 0x80;
    uint8_t out[7];
    i2cAcquireBus(&I2CD1);
    msg_t f = i2cMasterTransmitTimeout(
      &I2CD1, 0x19, &start_reg, 1, out, 7, TIME_INFINITE);
    (void)f;
    i2cReleaseBus(&I2CD1);
    if (out[0] & 0x8) {
        int16_t val_x = (out[2] << 8) | out[1];
        int16_t val_y = (out[4] << 8) | out[3];
        int16_t val_z = (out[6] << 8) | out[5];
        // Accel scale is +- 2.0g
        data[0] = ((float)val_x)*(4.0/(65535.0))*9.81;
        data[1] = ((float)val_y)*(4.0/(65535.0))*9.81;
        data[2] = ((float)val_z)*(4.0/(65535.0))*9.81;
        return 1;
    }
    return 0;
}
static uint8_t readMag(float* data)
{
    uint8_t start_reg = 0x03;
    uint8_t out[7];
    i2cAcquireBus(&I2CD1);
    msg_t f = i2cMasterTransmitTimeout(
        &I2CD1, 0x1E, &start_reg, 1, out, 7, TIME_INFINITE);
    (void)f;
    i2cReleaseBus(&I2CD1);
    // out[6] doesn't seem to reflect actual new data, so just push out every time
    int16_t val_x = (out[0] << 8) | out[1];
    int16_t val_z = (out[2] << 8) | out[3];
    int16_t val_y = (out[4] << 8) | out[5];
    data[0] = ((float)val_x)*1.22;
    data[1] = ((float)val_y)*1.22;
    data[2] = ((float)val_z)*1.22;
    return 1;
}

/*
  Lights the compass, one led at every call, if 
  all leds are turned on, it turns all leds off
  and starts over
*/
static void light_compass() {
  if(led_counter > 7) {
    int i;
    for(i = 0; i < 8; i++) {
      palClearPad(GPIOE, LED[i]);
    }
    led_counter = 0;
  } 

  palSetPad(GPIOE, LED[led_counter]);
  led_counter = led_counter + 1;
}

/*
  returns a float_t array[3] containing the drift value for x, y and z axies on the gyro
*/
static void calibrate_gyro(float *gyro_drift) {
  float tot_x = 0;
  float tot_y = 0;
  float tot_z = 0;

  float gyroData[3];

  int counter = 0;
  while(true) {
    // read from the gyro
    if(readGyro(gyroData)) {
      tot_x = tot_x + gyroData[0];
      tot_y = tot_y + gyroData[1];
      tot_z = tot_z + gyroData[2];

      counter++;

      light_compass();

      // if we have collected 'number_of_samples' samples, exit while loop
      if(counter >= number_of_samples) {
        break;
      }
    }

    // sleeps according to given freq    
    chThdSleepMilliseconds(1000/sample_freq);
  }
  chThdSleepMilliseconds(500);

  // calculate mean of drift
  gyro_drift[0] = tot_x / number_of_samples;
  gyro_drift[1] = tot_y / number_of_samples;
  gyro_drift[2] = tot_z / number_of_samples;

  // turn of leds
  int j;
  for(j = 0; j < 8; j++) {
    palClearPad(GPIOE, LED[j]);
  }

  // turn on the green leds
  palSetPad(GPIOE, LED[4]);
  palSetPad(GPIOE, LED[3]);
} 



int main(void) {
    halInit();
    chSysInit();

    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1000);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    spiStart(&SPID1, &spi1cfg);
    i2cStart(&I2CD1, &i2cconfig);
    initGyro();
    initAccel();
    initMag();
    chThdSleepMilliseconds(500);


    float gyro_drift[3];
    calibrate_gyro(gyro_drift);
			
	while(TRUE) {
		int receive = chnGetTimeout(&SDU1, TIME_IMMEDIATE);
		chThdSleepMilliseconds(10);

		if (((char) receive) == 'r') {
			while (TRUE) {
        float gyroData[3];
    		float accelData[3];
    		float magData[3];

				if (readGyro(gyroData) && readAccel(accelData) && readMag(magData)) {
          chprintf((BaseSequentialStream *)&SDU1, 
            "%f,%f,%f:%f,%f,%f:%f,%f,%f\n",
            gyroData[0]-gyro_drift[0], gyroData[1]-gyro_drift[1], gyroData[2]-gyro_drift[2],
            accelData[0], accelData[1], accelData[2],
            magData[0], magData[1], magData[2]);
				  /*
          chprintf((BaseSequentialStream *)&SDU1, 
            "gyro:%f:%f:%f\n", gyroData[0], gyroData[1], gyroData[2]);
				  chprintf((BaseSequentialStream *)&SDU1, 
            "accl:%f:%f:%f\n", accelData[0], accelData[1], accelData[2]);
				  chprintf((BaseSequentialStream *)&SDU1, 
            "mag:%f:%f:%f\n", magData[0], magData[1], magData[2]);
          */
					break;
				}
			}
		}
	}
}
