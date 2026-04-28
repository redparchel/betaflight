/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(USE_SOFT_I2C)

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "platform/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

#ifdef USE_I2C_DEVICE_1
void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cDevice[I2CDEV_1].handle);
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cDevice[I2CDEV_1].handle);
}
#endif

#ifdef USE_I2C_DEVICE_2
void I2C2_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cDevice[I2CDEV_2].handle);
}

void I2C2_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cDevice[I2CDEV_2].handle);
}
#endif

#ifdef USE_I2C_DEVICE_3
void I2C3_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cDevice[I2CDEV_3].handle);
}

void I2C3_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cDevice[I2CDEV_3].handle);
}
#endif

#ifdef USE_I2C_DEVICE_4
void I2C4_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&i2cDevice[I2CDEV_4].handle);
}

void I2C4_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&i2cDevice[I2CDEV_4].handle);
}
#endif

static volatile uint16_t i2cErrorCount = 0;

static bool i2cHandleHardwareFailure(i2cDevice_e device)
{
    (void)device;
    i2cErrorCount++;
    // reinit peripheral + clock out garbage
    //i2cInit(device);
    return false;
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

// Blocking write
bool i2cWrite(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_HandleTypeDef *pHandle = &i2cDevice[device].handle;

    if (!pHandle->Instance) {
        return false;
    }

    HAL_StatusTypeDef status;

    if (reg_ == 0xFF)
        status = HAL_I2C_Master_Transmit(pHandle ,addr_ << 1, &data, 1, I2C_TIMEOUT_SYS_TICKS);
    else
        status = HAL_I2C_Mem_Write(pHandle ,addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT_SYS_TICKS);

    if (status != HAL_OK)
        return i2cHandleHardwareFailure(device);

    return true;
}

// Background: HAL_I2C_Mem_Write_IT / Mem_Read_IT call I2C_RequestMemoryWrite/
// Read synchronously (polled with I2C_TIMEOUT_FLAG = 25ms in the upstream ST
// HAL) before enabling interrupts. On a slow-to-ACK slave (e.g. BMP280 on a
// shared I2C bus) the foreground task stalls up to 25ms inside what's sold as
// non-blocking — most visibly the BARO task, which then can't clear the
// CALIBRATING arming-disable flag. Use HAL APIs that are actually
// interrupt-driven instead (see i2cWriteBuffer / i2cReadBuffer below).

// Per-device buffer for combining reg + data into one non-blocking write.
// 32 bytes is comfortably above any current caller; i2cWriteBuffer rejects
// requests that would overflow it.
#define I2C_WRITE_BUF_LEN 32
static uint8_t i2cWriteBuf[I2CDEV_COUNT][I2C_WRITE_BUF_LEN];

// Per-device read state for the two-stage register-then-restart-receive flow.
static struct {
    uint8_t  devAddr;       // pre-shifted slave address
    volatile uint8_t reg;   // register byte storage (HAL needs a pointer)
    uint8_t *buf;
    uint8_t  len;
    volatile bool active;
} i2cReadState[I2CDEV_COUNT];

static i2cDevice_e i2cDeviceFromHandle(I2C_HandleTypeDef *hi2c)
{
    for (i2cDevice_e dev = 0; dev < I2CDEV_COUNT; dev++) {
        if (&i2cDevice[dev].handle == hi2c) {
            return dev;
        }
    }
    return I2CINVALID;
}

// Non-blocking write: combine reg + data into one buffer, ship via
// HAL_I2C_Master_Transmit_IT (no synchronous setup phase, unlike Mem_Write_IT).
bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_HandleTypeDef *pHandle = &i2cDevice[device].handle;

    if (!pHandle->Instance) {
        return false;
    }

    HAL_StatusTypeDef status;

    if (reg_ == 0xFF) {
        status = HAL_I2C_Master_Transmit_IT(pHandle, addr_ << 1, data, len_);
    } else {
        if ((uint16_t)len_ + 1 > I2C_WRITE_BUF_LEN) {
            return false;
        }
        uint8_t *buf = i2cWriteBuf[device];
        buf[0] = reg_;
        memcpy(&buf[1], data, len_);
        status = HAL_I2C_Master_Transmit_IT(pHandle, addr_ << 1, buf, 1 + len_);
    }

    if (status == HAL_BUSY) {
        return false;
    }
    if (status != HAL_OK) {
        return i2cHandleHardwareFailure(device);
    }
    return true;
}

// Blocking read
bool i2cRead(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_HandleTypeDef *pHandle = &i2cDevice[device].handle;

    if (!pHandle->Instance) {
        return false;
    }

    HAL_StatusTypeDef status;

    if (reg_ == 0xFF)
        status = HAL_I2C_Master_Receive(pHandle ,addr_ << 1, buf, len, I2C_TIMEOUT_SYS_TICKS);
    else
        status = HAL_I2C_Mem_Read(pHandle, addr_ << 1, reg_, I2C_MEMADD_SIZE_8BIT,buf, len, I2C_TIMEOUT_SYS_TICKS);

    if (status != HAL_OK) {
        return i2cHandleHardwareFailure(device);
    }

    return true;
}

// Non-blocking read: send reg byte as a SOFTEND frame, then in the TxCplt
// callback issue Seq_Receive with AUTOEND (HAL emits the restart+read+STOP).
// Both calls are interrupt-driven, no synchronous setup.
bool i2cReadBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_HandleTypeDef *pHandle = &i2cDevice[device].handle;

    if (!pHandle->Instance) {
        return false;
    }

    HAL_StatusTypeDef status;

    if (reg_ == 0xFF) {
        status = HAL_I2C_Master_Receive_IT(pHandle, addr_ << 1, buf, len);
    } else {
        if (i2cReadState[device].active) {
            return false;
        }
        i2cReadState[device].devAddr = addr_ << 1;
        i2cReadState[device].reg     = reg_;
        i2cReadState[device].buf     = buf;
        i2cReadState[device].len     = len;
        i2cReadState[device].active  = true;
        status = HAL_I2C_Master_Seq_Transmit_IT(pHandle, addr_ << 1,
                                                (uint8_t *)&i2cReadState[device].reg, 1,
                                                I2C_FIRST_FRAME);
        if (status != HAL_OK) {
            i2cReadState[device].active = false;
        }
    }

    if (status == HAL_BUSY) {
        return false;
    }
    if (status != HAL_OK) {
        return i2cHandleHardwareFailure(device);
    }
    return true;
}

// HAL weak callback overrides. TxCplt fires after the reg byte for a chained
// read (kick off the receive); for a write or the final read frame it's a
// no-op. ErrorCallback clears in-flight read state on NACK / bus error.
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2cDevice_e dev = i2cDeviceFromHandle(hi2c);
    if (dev == I2CINVALID || !i2cReadState[dev].active) {
        return;
    }
    i2cReadState[dev].active = false;
    HAL_I2C_Master_Seq_Receive_IT(hi2c, i2cReadState[dev].devAddr,
                                  i2cReadState[dev].buf, i2cReadState[dev].len,
                                  I2C_LAST_FRAME);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    i2cDevice_e dev = i2cDeviceFromHandle(hi2c);
    if (dev == I2CINVALID) {
        return;
    }
    i2cReadState[dev].active = false;
    i2cErrorCount++;
}

bool i2cBusy(i2cDevice_e device, bool *error)
{
    I2C_HandleTypeDef *pHandle = &i2cDevice[device].handle;

    if (error) {
        *error = pHandle->ErrorCode;
    }

    if (pHandle->State == HAL_I2C_STATE_READY)
    {
        if (__HAL_I2C_GET_FLAG(pHandle, I2C_FLAG_BUSY) == SET)
        {
            return true;
        }

        return false;
    }

    return true;
}

#endif
