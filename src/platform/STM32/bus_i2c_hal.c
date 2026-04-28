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

// Non-blocking write
//
// HAL_I2C_Mem_Write_IT calls I2C_RequestMemoryWrite, which spins on TXIS and
// TCR with a 25ms HAL timeout (I2C_TIMEOUT_FLAG) before enabling interrupts.
// On a slow-to-ACK slave (e.g. BMP280 on a shared I2C bus during boot) this
// 25ms foreground stall blocks the calling task — most visibly the BARO task,
// which prevents the CALIBRATING arming-disable flag from clearing.
//
// Replace the Mem_Write_IT path with chained Master_Seq_Transmit_IT calls.
// The Seq variants are truly non-blocking (no synchronous TXIS/TCR polling);
// the register-address byte is sent as the first frame, and the data bytes
// are sent from HAL_I2C_MasterTxCpltCallback once that frame completes.
typedef enum {
    I2C_TXN_IDLE = 0,
    I2C_TXN_REG_THEN_TX,    // After reg byte sent, continue with data write
    I2C_TXN_REG_THEN_RX,    // After reg byte sent, restart and read data
} i2cTxnPhase_t;

typedef struct {
    volatile i2cTxnPhase_t phase;
    uint8_t  devAddr;       // pre-shifted slave address
    volatile uint8_t reg;   // register byte storage (HAL needs a pointer)
    uint8_t *dataPtr;
    uint16_t dataLen;
} i2cTxnState_t;

static i2cTxnState_t i2cTxnState[I2CDEV_COUNT];

static i2cDevice_e i2cDeviceFromHandle(I2C_HandleTypeDef *hi2c)
{
    for (i2cDevice_e dev = 0; dev < I2CDEV_COUNT; dev++) {
        if (&i2cDevice[dev].handle == hi2c) {
            return dev;
        }
    }
    return I2CINVALID;
}

bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_HandleTypeDef *pHandle = &i2cDevice[device].handle;

    if (!pHandle->Instance) {
        return false;
    }

    i2cTxnState_t *st = &i2cTxnState[device];

    if (st->phase != I2C_TXN_IDLE) {
        return false;
    }

    HAL_StatusTypeDef status;

    if (reg_ == 0xFF) {
        // No register address; one-shot non-blocking transmit
        status = HAL_I2C_Master_Transmit_IT(pHandle, addr_ << 1, data, len_);
    } else {
        // Send the register byte as the first frame (RELOAD+SOFTEND, no STOP).
        // The TxCplt callback continues with the data bytes (AUTOEND).
        st->devAddr = addr_ << 1;
        st->reg     = reg_;
        st->dataPtr = data;
        st->dataLen = len_;
        st->phase   = I2C_TXN_REG_THEN_TX;
        status = HAL_I2C_Master_Seq_Transmit_IT(pHandle, st->devAddr,
                                                (uint8_t *)&st->reg, 1,
                                                I2C_FIRST_AND_NEXT_FRAME);
        if (status != HAL_OK) {
            st->phase = I2C_TXN_IDLE;
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

// Non-blocking read
//
// Same root-cause as i2cWriteBuffer: HAL_I2C_Mem_Read_IT spins on TXIS/TCR
// inside I2C_RequestMemoryRead with a 25ms HAL timeout before enabling
// interrupts. Replace with chained Seq calls: send the register byte as
// I2C_FIRST_FRAME (SOFTEND, no STOP), then in HAL_I2C_MasterTxCpltCallback
// issue a Seq_Receive_IT with I2C_LAST_FRAME (restart + AUTOEND).
bool i2cReadBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    I2C_HandleTypeDef *pHandle = &i2cDevice[device].handle;

    if (!pHandle->Instance) {
        return false;
    }

    i2cTxnState_t *st = &i2cTxnState[device];

    if (st->phase != I2C_TXN_IDLE) {
        return false;
    }

    HAL_StatusTypeDef status;

    if (reg_ == 0xFF) {
        // No register address; one-shot non-blocking receive
        status = HAL_I2C_Master_Receive_IT(pHandle, addr_ << 1, buf, len);
    } else {
        st->devAddr = addr_ << 1;
        st->reg     = reg_;
        st->dataPtr = buf;
        st->dataLen = len;
        st->phase   = I2C_TXN_REG_THEN_RX;
        status = HAL_I2C_Master_Seq_Transmit_IT(pHandle, st->devAddr,
                                                (uint8_t *)&st->reg, 1,
                                                I2C_FIRST_FRAME);
        if (status != HAL_OK) {
            st->phase = I2C_TXN_IDLE;
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

// HAL callback chain: continue or finish a multi-stage Seq transaction.
// HAL declares these as __weak so this definition overrides the default.
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2cDevice_e dev = i2cDeviceFromHandle(hi2c);
    if (dev == I2CINVALID) {
        return;
    }
    i2cTxnState_t *st = &i2cTxnState[dev];
    i2cTxnPhase_t phase = st->phase;
    // Clear phase before issuing the next stage so a recursive callback (final
    // stage's TxCplt) sees IDLE.
    st->phase = I2C_TXN_IDLE;

    switch (phase) {
        case I2C_TXN_REG_THEN_TX:
            HAL_I2C_Master_Seq_Transmit_IT(hi2c, st->devAddr,
                                           st->dataPtr, st->dataLen,
                                           I2C_LAST_FRAME);
            break;
        case I2C_TXN_REG_THEN_RX:
            HAL_I2C_Master_Seq_Receive_IT(hi2c, st->devAddr,
                                          st->dataPtr, st->dataLen,
                                          I2C_LAST_FRAME);
            break;
        default:
            break;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    i2cDevice_e dev = i2cDeviceFromHandle(hi2c);
    if (dev == I2CINVALID) {
        return;
    }
    i2cTxnState[dev].phase = I2C_TXN_IDLE;
    i2cErrorCount++;
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2cDevice_e dev = i2cDeviceFromHandle(hi2c);
    if (dev == I2CINVALID) {
        return;
    }
    i2cTxnState[dev].phase = I2C_TXN_IDLE;
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
