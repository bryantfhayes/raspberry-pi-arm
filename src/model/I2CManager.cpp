/******************************************************************************
i2ctest.cpp
Raspberry Pi I2C interface demo
Byron Jacquot @ SparkFun Electronics>
4/2/2014
https://github.com/sparkfun/Pi_Wedge

A brief demonstration of the Raspberry Pi I2C interface, using the SparkFun
Pi Wedge breakout board.

Resources:

This example makes use of the Wiring Pi library, which streamlines the interface
the the I/O pins on the Raspberry Pi, providing an API that is similar to the
Arduino.  You can learn about installing Wiring Pi here:
http://wiringpi.com/download-and-install/

The I2C API is documented here:
https://projects.drogon.net/raspberry-pi/wiringpi/i2c-library/

The init call returns a standard file descriptor.  More detailed configuration
of the interface can be performed using ioctl calls on that descriptor.
See the wiringPi I2C implementation (wiringPi/wiringPiI2C.c) for some examples.
Parameters configurable with ioctl are documented here:
http://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/i2c/dev-interface

Hardware connections:

This file interfaces with the SparkFun MCP4725 breakout board:
https://www.sparkfun.com/products/8736

The board was connected as follows:
(Raspberry Pi)(MCP4725)
GND  -> GND
3.3V -> Vcc
SCL  -> SCL
SDA  -> SDA

An oscilloscope probe was connected to the analog output pin of the MCP4725.

To build this file, I use the command:
>  g++ i2ctest.cpp -lwiringPi

Then to run it, first the I2C kernel module needs to be loaded.  This can be 
done using the GPIO utility.
> gpio load i2c 400
> ./a.out

This will run the MCP through its output range several times.  A rising 
sawtooth will be seen on the analog output.

Development environment specifics:
Tested on Raspberry Pi V2 hardware, running Raspbian.
Building with GCC 4.6.3 (Debian 4.6.3-14+rpi1)

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include <iostream>
#include <string>
#include <errno.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <cmath>

#define PCA9685_DEFAULT_I2C_ADDR 0x40
#define PCA9685_INTERNAL_OSC  25000000.0
#define PCA9685_ALL_LED 0xff
#define DEFAULT_PWM 1245
#define PWM_HZ 200

using namespace std;

class PCA9685
{
    int mAddr;
    int mfd;
    bool mRestartEnabled;
    static PCA9685 *sInstance;
    PCA9685(){}

    bool setPrescaleFromHz(float hz, float oscFreq);

  public:
    /**
     * PCA9685 registers
     */
    typedef enum { REG_MODE1       = 0x00,
                   REG_MODE2       = 0x01,
                   REG_I2C_SA1     = 0x02, // I2C subaddress 1
                   REG_I2C_SA2     = 0x03,
                   REG_I2C_SA3     = 0x04,
                   REG_ALLCALL     = 0x05, // I2C all call address

                   // LED output PWM control
                   REG_LED0_ON_L   = 0x06, // LED0 ON low byte
                   REG_LED0_ON_H   = 0x07, // LED0 ON high byte
                   REG_LED0_OFF_L  = 0x08, // LED0 OFF low byte
                   REG_LED0_OFF_H  = 0x09, // LED0 OFF high byte
                   REG_LED1_ON_L   = 0x0a,
                   REG_LED1_ON_H   = 0x0b,
                   REG_LED1_OFF_L  = 0x0c,
                   REG_LED1_OFF_H  = 0x0d,
                   REG_LED2_ON_L   = 0x0e,
                   REG_LED2_ON_H   = 0x0f,
                   REG_LED2_OFF_L  = 0x10,
                   REG_LED2_OFF_H  = 0x11,
                   REG_LED3_ON_L   = 0x12,
                   REG_LED3_ON_H   = 0x13,
                   REG_LED3_OFF_L  = 0x14,
                   REG_LED3_OFF_H  = 0x15,
                   REG_LED4_ON_L   = 0x16,
                   REG_LED4_ON_H   = 0x17,
                   REG_LED4_OFF_L  = 0x18,
                   REG_LED4_OFF_H  = 0x19,
                   REG_LED5_ON_L   = 0x1a,
                   REG_LED5_ON_H   = 0x1b,
                   REG_LED5_OFF_L  = 0x1c,
                   REG_LED5_OFF_H  = 0x1d,
                   REG_LED6_ON_L   = 0x1e,
                   REG_LED6_ON_H   = 0x1f,
                   REG_LED6_OFF_L  = 0x20,
                   REG_LED6_OFF_H  = 0x21,
                   REG_LED7_ON_L   = 0x22,
                   REG_LED7_ON_H   = 0x23,
                   REG_LED7_OFF_L  = 0x24,
                   REG_LED7_OFF_H  = 0x25,
                   REG_LED8_ON_L   = 0x26,
                   REG_LED8_ON_H   = 0x27,
                   REG_LED8_OFF_L  = 0x28,
                   REG_LED8_OFF_H  = 0x29,
                   REG_LED9_ON_L   = 0x2a,
                   REG_LED9_ON_H   = 0x2b,
                   REG_LED9_OFF_L  = 0x2c,
                   REG_LED9_OFF_H  = 0x2d,
                   REG_LED10_ON_L  = 0x2e,
                   REG_LED10_ON_H  = 0x2f,
                   REG_LED10_OFF_L = 0x30,
                   REG_LED10_OFF_H = 0x31,
                   REG_LED11_ON_L  = 0x32,
                   REG_LED11_ON_H  = 0x33,
                   REG_LED11_OFF_L = 0x34,
                   REG_LED11_OFF_H = 0x35,
                   REG_LED12_ON_L  = 0x36,
                   REG_LED12_ON_H  = 0x37,
                   REG_LED12_OFF_L = 0x38,
                   REG_LED12_OFF_H = 0x39,
                   REG_LED13_ON_L  = 0x3a,
                   REG_LED13_ON_H  = 0x3b,
                   REG_LED13_OFF_L = 0x3c,
                   REG_LED13_OFF_H = 0x3d,
                   REG_LED14_ON_L  = 0x3e,
                   REG_LED14_ON_H  = 0x3f,
                   REG_LED14_OFF_L = 0x40,
                   REG_LED14_OFF_H = 0x41,
                   REG_LED15_ON_L  = 0x42,
                   REG_LED15_ON_H  = 0x43,
                   REG_LED15_OFF_L = 0x44,
                   REG_LED15_OFF_H = 0x45,
                   // 0x46-0xf9 reserved

                   REG_ALL_LED_ON_L  = 0xfa, // write all LED ON L
                   REG_ALL_LED_ON_H  = 0xfb, // write all LED ON H
                   REG_ALL_LED_OFF_L = 0xfc, // write all LED OFF L
                   REG_ALL_LED_OFF_H = 0xfd, // write all LED OFF H
                   REG_PRESCALE      = 0xfe,
                   REG_TESTMODE      = 0xff  // don't use
    } PCA9685_REG_T;

    /**
     * MODE1 bits
     */
    typedef enum { MODE1_ALL_CALL    = 0x01, // all call status
                   MODE1_SUB3        = 0x02, // subcall 3 status
                   MODE1_SUB2        = 0x04, // subcall 2 status
                   MODE1_SUB1        = 0x08, // subcall 1 status
                   MODE1_SLEEP       = 0x10, // sleep/normal mode
                   MODE1_AI          = 0x20, // auto-increment enable
                   MODE1_EXTCLK      = 0x40, // external clock enable
                   MODE1_RESTART     = 0x80  // restart status
    } PCA9685_MODE1_T;
    
    /**
     * MODE2 bits
     */
    typedef enum { MODE2_OUTNE0      = 0x01, // output driver enable bit 0
                   MODE2_OUTNE       = 0x02, // output driver enable bit 1
                   MODE2_OUTDRV      = 0x04, // output open-drain/totem pole
                   MODE2_OCH         = 0x08, // output change on STOP or ACK
                   MODE2_INVRT       = 0x10, // output logic state invert
                   MODE2_RESERVE0    = 0x20, // reserved
                   MODE2_RESERVE1    = 0x40, // reserved
                   MODE2_RESERVE2    = 0x80  // reserved
    } PCA9685_MODE2_T;

    int init()
    {
      int status = 0;

      // I2C setup
      mAddr = PCA9685_DEFAULT_I2C_ADDR;
      mfd = wiringPiI2CSetup(mAddr);
      if (mfd == -1)
      {
        return 1;
      }

      usleep(1000000);

      // PCA9685 initial setup
      enableAutoIncrement(true);
      enableRestart(true);
      setModeSleep(true);
      setPrescaleFromHz(PWM_HZ - 10);
      setModeSleep(false);

      usleep(1000000);

      ledOnTime(PCA9685_ALL_LED, 0);
      ledOffTime(PCA9685_ALL_LED, DEFAULT_PWM);

      return 0;
    }

    void destroy()
    {
      if (sInstance != NULL)
      {
        delete sInstance;
        sInstance = NULL;
      }
    }

    bool write16(int reg, int data)
    {
      int result = wiringPiI2CWriteReg16(mfd, reg, data);
      if (result == -1)
      {
        return false;
      }
      return true;
    }

    bool write8(int reg, int data)
    {
      int result = wiringPiI2CWriteReg8(mfd, reg, data);
      if (result == -1)
      {
        return false;
      }
      return true;
    }

    int read16(int reg)
    {
      int result = wiringPiI2CReadReg16(mfd, reg);
      return result;
    }

    int read8(int reg)
    {
      int result = wiringPiI2CReadReg8(mfd, reg);
      return result;
    }

    void enableRestart(bool enabled) 
    {
      mRestartEnabled = enabled; 
    }

    bool setModeSleep(bool sleep)
    {
      uint8_t mode1 = read8(REG_MODE1);
      uint8_t restartBit = mode1 & MODE1_RESTART;

      if (sleep)
        mode1 |= MODE1_SLEEP;
      else
        mode1 &= ~MODE1_SLEEP;

      // if we are waking up, then preserve but don't write restart bit if set
      if (!sleep && restartBit)
        mode1 &= ~MODE1_RESTART;

      write8(REG_MODE1, mode1);

      // Need a delay of 500us after turning sleep mode off for the oscillator 
      // to stabilize
      if (!sleep)
        usleep(500);

      // now check to see if we want to (and can) restart when waking up
      if (restartBit && mRestartEnabled && !sleep)
        {
          mode1 |= restartBit;
          write8(REG_MODE1, mode1);
        }

      return true;
    }

    bool enableAutoIncrement(bool ai)
    {
      uint8_t mode1 = read8(REG_MODE1);

      if (ai)
      {
        mode1 |= MODE1_AI;
      }
      else
      {
        mode1 &= ~MODE1_AI;
      }

      return write8(REG_MODE1, mode1);
    }

    bool ledFullOn(uint8_t led, bool val)
    {
      if (led > 15 && (led != PCA9685_ALL_LED))
      {
        return false;
      }

      // figure out the register offset (*_ON_H)
      uint8_t regoff;

      if (led == PCA9685_ALL_LED)
      {
        regoff = REG_ALL_LED_ON_H;
      }
      else
      {
        regoff = REG_LED0_ON_L + (led * 4) + 1;
      }

      uint8_t bits = read8(regoff);

      if (val)
      {
        bits |= 0x10;
      }
      else
      {
        bits &= ~0x10;
      }

      return write8(regoff, bits);
    }

    bool ledOnTime(uint8_t led, uint16_t time)
    {
      if (led > 15 && (led != PCA9685_ALL_LED))
      {
        return false;
      }

      if (time > 4095)
      {
        return false;
      }

      // figure out the register offset (*_ON_L)
      uint8_t regoff;

      if (led == PCA9685_ALL_LED)
      {
        regoff = REG_ALL_LED_ON_L;
      }
      else
      {
        regoff = REG_LED0_ON_L + (led * 4);
      }

      // we need to preserve the full ON bit in *_ON_H
      uint8_t onbit = (read8(regoff + 1) & 0x10);

      time = (time & 0x0fff) | (onbit << 8);

      return write16(regoff, time);
    }

    bool ledFullOff(uint8_t led, bool val)
    {
      if (led > 15 && (led != PCA9685_ALL_LED))
      {
        return false;
      }

      // figure out the register offset (*_OFF_H)
      uint8_t regoff;

      if (led == PCA9685_ALL_LED)
      {
        regoff = REG_ALL_LED_OFF_H;
      }
      else
      {
        regoff = REG_LED0_ON_L + (led * 4) + 3;
      }

      uint8_t bits = read8(regoff);

      if (val)
      {
        bits |= 0x10;
      }
      else
      {
        bits &= ~0x10;
      }

      return write8(regoff, bits);
    }

    bool ledOffTime(uint8_t led, uint16_t time)
    {
      if (led > 15 && (led != PCA9685_ALL_LED))
      {
        return false;
      }

      if (time > 4095)
      {
        return false;
      }

      // figure out the register offset (*_OFF_L)
      uint8_t regoff;

      if (led == PCA9685_ALL_LED)
      {
        regoff = REG_ALL_LED_OFF_L;
      }
      else
      {
        regoff = REG_LED0_ON_L + (led * 4) + 2;
      }

      // we need to preserve the full OFF bit in *_OFF_H
      uint8_t offbit = (read8(regoff + 1) & 0x10);

      time = (time & 0x0fff) | (offbit << 8);

      return write16(regoff, time);
    }

    bool setPrescale(uint8_t prescale)
    {
      // This will be ignored if the device isn't in SLEEP mode
      return write16(REG_PRESCALE, prescale);
    }

    bool setPrescaleFromHz(float hz)
    {
      float prescale = round( PCA9685_INTERNAL_OSC / (4096.0 * hz) ) - 1;
      return setPrescale(uint8_t(prescale));
    }

    static PCA9685 *GetInstance()
    {
      if (!sInstance)
      {
        sInstance = new PCA9685;
      }
      return sInstance;
    }
};

PCA9685 *PCA9685::sInstance = 0;

extern "C"
{
  // Prototypes
  int init();
  void destroy(void);
  int setServoPwm(int servo, int pwm);

  int init()
  {
    int status = PCA9685::GetInstance()->init();

    printf("PRESCALE: %d\n", PCA9685::GetInstance()->read8(PCA9685::REG_PRESCALE));

    return status;
  }

  void destroy()
  {
    PCA9685::GetInstance()->destroy();
  }

  int setServoPwm(int servo, int pwm)
  {
    bool status = PCA9685::GetInstance()->ledOnTime(servo, 0);
    if (status == false)
    {
      return 1;
    }
    status = PCA9685::GetInstance()->ledOffTime(servo, pwm);
    if (status == false)
    {
      return 1;
    }

    return 0;
  }
}

