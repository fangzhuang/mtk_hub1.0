/*
 * Copyright (C) 2016 The Android Open Source Project
 * Copyright (C) 2018 Yamaha Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <seos.h>
#include <util.h>
#include <timer.h>
#include <sensors.h>
#include <plat/inc/rtc.h>

#include <magnetometer.h>
#include <contexthub_core.h>
#include <cust_mag.h>
#include "cache_internal.h"


#define MAG_NAME "yas537"
#define I2C_SPEED (400000)


enum YAS537State {
    STATE_SAMPLE = CHIP_SAMPLING,
    STATE_CONVERT = CHIP_CONVERT,
    STATE_SAMPLE_DONE = CHIP_SAMPLING_DONE,
    STATE_ENABLE = CHIP_ENABLE,
    STATE_ENABLE_DONE = CHIP_ENABLE_DONE,
    STATE_DISABLE = CHIP_DISABLE,
    STATE_DISABLE_DONE = CHIP_DISABLE_DONE,
    STATE_RATECHG = CHIP_RATECHG,
    STATE_RATECHG_DONE = CHIP_RATECHG_DONE,
    STATE_INIT_DONE = CHIP_INIT_DONE,
    STATE_IDLE = CHIP_IDLE,
    STATE_RESET = CHIP_RESET,
    STATE_CORE,

    STATE_RESET1,
    STATE_RESET2,
    STATE_RESET3,
    STATE_RESET4,
    STATE_RESET5,
    STATE_RESET6,
    STATE_RESET7,
    STATE_RESET8,
    STATE_RESET9,
    STATE_RESET10,
    STATE_RESET11,
    STATE_RESET12,
    STATE_RESET13,
    STATE_RESET14,
    STATE_RESET15,

    STATE_ENABLE_RESET1,
    STATE_ENABLE_RESET2,
    STATE_ENABLE_RESET3,
    STATE_ENABLE_RESET4,
    STATE_ENABLE_RESET5,
    STATE_ENABLE_RESET6,
    STATE_ENABLE_RESET7,
    STATE_ENABLE_RESET8,
    STATE_ENABLE_RESET9,
    STATE_ENABLE_RESET10,
    STATE_ENABLE_RESET11,
    STATE_ENABLE_RESET12,
    STATE_ENABLE_RESET13,
    STATE_ENABLE_RESET14,
    STATE_ENABLE_RESET15,
    STATE_ENABLE_RCOIL1,
    STATE_ENABLE_RCOIL2,
    STATE_ENABLE_RCOIL3,
    STATE_ENABLE_RCOIL4,
    STATE_ENABLE_RCOIL5,
    STATE_ENABLE_RCOIL6,
    STATE_ENABLE_RCOIL7,
    STATE_ENABLE_RCOIL8,
    STATE_ENABLE_RCOIL9,
    STATE_ENABLE_RCOIL10,
    STATE_ENABLE_RCOIL11,
    STATE_ENABLE_RCOIL12,
    STATE_ENABLE_RCOIL13,
    STATE_ENABLE_RCOIL14,
    STATE_ENABLE_RCOIL15,
    STATE_ENABLE_RCOIL16,
    STATE_ENABLE_RCOIL17,
    STATE_ENABLE_RCOIL18,
    STATE_ENABLE_RCOIL19,
    STATE_ENABLE_RCOIL20,
    STATE_ENABLE_RCOIL21,
    STATE_ENABLE_CONT_START,
    STATE_ENABLE_CONT_START_WAIT,

    STATE_RATECHG_RESET1,
    STATE_RATECHG_RESET2,
    STATE_RATECHG_RESET3,
    STATE_RATECHG_RESET4,
    STATE_RATECHG_RESET5,
    STATE_RATECHG_RESET6,
    STATE_RATECHG_RESET7,
    STATE_RATECHG_RESET8,
    STATE_RATECHG_RESET9,
    STATE_RATECHG_RESET10,
    STATE_RATECHG_RESET11,
    STATE_RATECHG_RESET12,
    STATE_RATECHG_RESET13,
    STATE_RATECHG_RESET14,
    STATE_RATECHG_RESET15,
    STATE_RATECHG_RCOIL1,
    STATE_RATECHG_RCOIL2,
    STATE_RATECHG_RCOIL3,
    STATE_RATECHG_RCOIL4,
    STATE_RATECHG_RCOIL5,
    STATE_RATECHG_RCOIL6,
    STATE_RATECHG_RCOIL7,
    STATE_RATECHG_RCOIL8,
    STATE_RATECHG_RCOIL9,
    STATE_RATECHG_RCOIL10,
    STATE_RATECHG_RCOIL11,
    STATE_RATECHG_RCOIL12,
    STATE_RATECHG_RCOIL13,
    STATE_RATECHG_RCOIL14,
    STATE_RATECHG_RCOIL15,
    STATE_RATECHG_RCOIL16,
    STATE_RATECHG_RCOIL17,
    STATE_RATECHG_RCOIL18,
    STATE_RATECHG_RCOIL19,
    STATE_RATECHG_RCOIL20,
    STATE_RATECHG_RCOIL21,
    STATE_RATECHG_CONT_START,
    STATE_RATECHG_CONT_START_WAIT,
};

typedef void (*delayCallbackFunc)(uint32_t timerId, void *data);

struct delayTimer {
    uint32_t timerHandle;
    I2cCallbackF i2cCallBack;
    void *next_state;
};

/********************************************/
/* yamaha core driver common definition     */
/********************************************/
#define YAS_NO_ERROR                    (0) /*!< Succeed */
#define YAS_ERROR_ARG                   (-1) /*!< Invalid argument */
#define YAS_ERROR_INITIALIZE            (-2) /*!< Invalid initialization status
                         */
#define YAS_ERROR_BUSY                  (-3) /*!< Sensor is busy */
#define YAS_ERROR_DEVICE_COMMUNICATION  (-4) /*!< Device communication error */
#define YAS_ERROR_CHIP_ID               (-5) /*!< Invalid chip id */
#define YAS_ERROR_CALREG                (-6) /*!< Invalid CAL register */
#define YAS_ERROR_OVERFLOW              (-7) /*!< Overflow occured */
#define YAS_ERROR_UNDERFLOW             (-8) /*!< Underflow occured */
#define YAS_ERROR_DIRCALC               (-9) /*!< Direction calcuration error */
#define YAS_ERROR_ERROR                 (-128) /*!< other error */

#define YAS_DEFAULT_SENSOR_DELAY        (50)

#ifndef ABS
#define ABS(a)      ((a) > 0 ? (a) : -(a)) /*!< Absolute value */
#endif
#ifndef CLIP
#define CLIP(in, min, max) \
    ((in) < (min) ? (min) : ((max) < (in) ? (max) : (in)))
#endif

struct yas_matrix {
    int16_t m[9]; /*!< matrix data */
};

/********************************************/
/* yamaha yas537 driver definition          */
/********************************************/
#define YAS537_REG_DIDR         (0x80)
#define YAS537_REG_CMDR         (0x81)
#define YAS537_REG_CONFR        (0x82)
#define YAS537_REG_INTRVLR      (0x83)
#define YAS537_REG_OXR          (0x84)
#define YAS537_REG_OY1R         (0x85)
#define YAS537_REG_OY2R         (0x86)
#define YAS537_REG_AVRR         (0x87)
#define YAS537_REG_HCKR         (0x88)
#define YAS537_REG_LCKR         (0x89)
#define YAS537_REG_SRSTR        (0x90)
#define YAS537_REG_ADCCALR      (0x91)
#define YAS537_REG_MTCR         (0x93)
#define YAS537_REG_OCR          (0x9e)
#define YAS537_REG_TRMR         (0x9f)
#define YAS537_REG_RCMR         (0xa0)
#define YAS537_REG_DATAR        (0xb0)
#define YAS537_REG_CALR         (0xc0)

#define YAS537_DATA_UNDERFLOW       (0)
#define YAS537_DATA_OVERFLOW        (16383)
#define YAS537_DEVICE_ID        (0x07)  /* YAS537 (MS-3T) */

#define YAS_X_OVERFLOW          (0x01)
#define YAS_X_UNDERFLOW         (0x02)
#define YAS_Y1_OVERFLOW         (0x04)
#define YAS_Y1_UNDERFLOW        (0x08)
#define YAS_Y2_OVERFLOW         (0x10)
#define YAS_Y2_UNDERFLOW        (0x20)
#define YAS_OVERFLOW    (YAS_X_OVERFLOW|YAS_Y1_OVERFLOW|YAS_Y2_OVERFLOW)
#define YAS_UNDERFLOW   (YAS_X_UNDERFLOW|YAS_Y1_UNDERFLOW|YAS_Y2_UNDERFLOW)

#define YAS537_MAG_STATE_NORMAL     (0)
#define YAS537_MAG_STATE_INIT_COIL  (1)
#define YAS537_MAG_INITCOIL_TIMEOUT (1000)  /* msec */
#define YAS537_MAG_POWER_ON_RESET_TIME  (4000)  /* usec */
#define YAS537_MAG_NOTRANS_POSITION (2)

#define YAS537_MAG_AVERAGE_8        (0)
#define YAS537_MAG_AVERAGE_16       (1)
#define YAS537_MAG_AVERAGE_32       (2)
#define YAS537_MAG_AVERAGE_64       (3)
#define YAS537_MAG_AVERAGE_128      (4)
#define YAS537_MAG_AVERAGE_256      (5)

#define YAS537_MAG_RCOIL_TIME       (65)

#define YAS537_BUSY_RETRY_MAX       (3)

#define set_vector(to, from) \
    {int _l; for (_l = 0; _l < AXES_NUM; _l++) (to)[_l] = (from)[_l]; }


struct yas537_cal {
    uint8_t data[17];
    int16_t a[9];
    uint8_t k, ver;
};

struct yas537_cdriver {
    int enable;
    int measure_state;
    int invalid_data;
    int flowflag;
    int busy_retry;
    uint32_t delay;
    int average;
    uint32_t current_time;
    uint32_t invalid_data_time;
    uint16_t last_after_rcoil[3];
    struct yas537_cal cal;
    int8_t hard_offset[3];
    int16_t overflow[3], underflow[3];
    struct yas_matrix static_matrix;
    int measure_time_worst[6];
};

struct yas_state_sample {
    int mode;
    int step;
};

enum {
    YAS537_MODE_SAMPLE_NORMAL,
    YAS537_MODE_SAMPLE_INIT_COIL,
    YAS537_MODE_SAMPLE_INIT_COIL_OF,
};


// we use txbuf for write ,rxbuf for read ,autoDetect for id check ,
// if you need any gloable variable, you can write at the end of the struct
SRAM_REGION_BSS static struct YAS537Task {
    /* txBuf for i2c operation, fill register and fill value */
    uint8_t txBuf[8];
    /* rxBuf for i2c operation, receive rawdata */
    uint8_t rxBuf[16];
    uint8_t fuse[3];
    uint8_t autoDetect[2];
    uint64_t hwSampleTime;
    struct magTimeStampBuffer magTimeBuf;
    struct transferDataInfo dataInfo;
    struct magDataPacket magPacket;
    /* data for factory */
    struct TripleAxisDataPoint factoryData;
    struct mag_hw *hw;
    struct sensorDriverConvert cvt;
    uint8_t i2c_addr;
    struct mag_dev_info_t mag_dev_info;

    struct delayTimer delayTimer;

    struct yas537_cdriver yascore;
    int32_t last_xyz[AXES_NUM];
    uint16_t enable_rcoil_xy1y2[3];
    int state_enable_rcoil;
    int state_reset;
    struct yas_state_sample state_sample;
} mTask;

static struct YAS537Task *yas537DebugPoint;

SRAM_REGION_FUNCTION void yas537TimerCbkF(uint64_t time)
{
    magTimeBufferWrite(&mTask.magTimeBuf, time);
}

static void magGetCalibration(int32_t *cali, int32_t size)
{
}
static void magSetCalibration(int32_t *cali, int32_t size)
{
}
static void magGetData(void *sample)
{
    struct TripleAxisDataPoint *tripleSample
        = (struct TripleAxisDataPoint *) sample;

    tripleSample->ix = mTask.factoryData.ix;
    tripleSample->iy = mTask.factoryData.iy;
    tripleSample->iz = mTask.factoryData.iz;
}

static void magGetSensorInfo(struct sensorInfo_t *data)
{
    strncpy(data->name, MAG_NAME, sizeof(data->name));
    memcpy(&data->mag_dev_info, &mTask.mag_dev_info,
           sizeof(struct mag_dev_info_t));
}

static int yas537RegisterCore(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                              void *inBuf, uint8_t inSize, uint8_t elemInSize,
                              void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    struct sensorCoreInfo mInfo;

    memset(&mInfo, 0x00, sizeof(struct sensorCoreInfo));

    /* Register sensor Core */
    mInfo.sensType = SENS_TYPE_MAG;
    mInfo.getCalibration = magGetCalibration;
    mInfo.setCalibration = magSetCalibration;
    mInfo.getData = magGetData;
    mInfo.getSensorInfo = magGetSensorInfo;
    sensorCoreRegister(&mInfo);

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static void registerDelayTimer(uint32_t delay, delayCallbackFunc func,
                               I2cCallbackF i2cCallBack, void *next_state)
{
    mTask.delayTimer.i2cCallBack = i2cCallBack;
    mTask.delayTimer.next_state = next_state;
    if (mTask.delayTimer.timerHandle)
        timTimerCancel(mTask.delayTimer.timerHandle);
    mTask.delayTimer.timerHandle
        = timTimerSet(delay, 0, 50, func, NULL, true);
    if (!mTask.delayTimer.timerHandle)
        configASSERT(0);
}

static int yascore_invalid_magnetic_field(uint16_t *cur, uint16_t *last)
{
    int16_t invalid_thresh[] = {1000, 1000, 1000};
    int i;

    for (i = 0; i < AXES_NUM; i++)
        if (invalid_thresh[i] < ABS(cur[i] - last[i]))
            return 1;
    return 0;
}

static uint8_t yascore_update_intrvlr(uint32_t delay)
{
    const int time_worst
        = mTask.yascore.measure_time_worst[mTask.yascore.average];

    /* delay 4.1 x SMPLTIM [7:0] msec */
    if (((uint32_t)4100 * 255 + time_worst) / 1000 < delay)
        delay = (uint32_t)4100 * 255 + time_worst;
    else
        delay *= 1000;
    delay = (delay - time_worst) / 4100;

    return delay <= 1 ? 2 : (uint8_t) delay;
}

static void yascore_sensitivity_correction(uint16_t *xy1y2, uint16_t t)
{
    int32_t h[3];
    int i;

    if (t == 8176)
        return;
    for (i = 0; i < AXES_NUM; i++) {
        h[i] = (int32_t) xy1y2[i] - 8192;
        h[i] = 100000 * h[i] / (100000 - 11 * (t - 8176));
        xy1y2[i] = CLIP(h[i], -8192, 8191) + 8192;
    }
}

static void yascore_xy1y2_to_xyz(uint16_t *xy1y2, int32_t *xyz)
{
    xyz[0] = ((int32_t)xy1y2[0] - 8192) * 300;
    xyz[1] = ((int32_t)xy1y2[1] - xy1y2[2]) * 1732 / 10;
    xyz[2] = ((int32_t) - xy1y2[1] - xy1y2[2] + 16384) * 300;
}

static void yascore_apply_matrix(int32_t *xyz, struct yas_matrix *m)
{
    int32_t tmp[3];
    int i;

    for (i = 0; i < AXES_NUM; i++)
        tmp[i] = ((m->m[i * 3] / 10) * (xyz[0] / 10)
                  + (m->m[i * 3 + 1] / 10) * (xyz[1] / 10)
                  + (m->m[i * 3 + 2] / 10) * (xyz[2] / 10)) / 100;
    for (i = 0; i < AXES_NUM; i++)
        xyz[i] = tmp[i];
}

static int yas537_start_yas537(int ldtc, int fors, int cont,
                               I2cCallbackF i2cCallBack, void *next_state)
{
    uint8_t data = 0x01;

    data = (uint8_t)(data | (ldtc << 1));
    data = (uint8_t)(data | (fors << 2));
    data = (uint8_t)(data | (cont << 5));

    mTask.txBuf[0] = YAS537_REG_CMDR;
    mTask.txBuf[1] = data;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                       mTask.txBuf, 2, i2cCallBack, next_state);
}

static int yas537_cont_start(
    I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)

{
    return yas537_start_yas537(0, 0, 1, i2cCallBack, next_state);
}

static void delayCallback_cont_start_wait(uint32_t timerId, void *data)
{
    mTask.delayTimer.timerHandle = 0;

    sensorFsmEnqueueFakeI2cEvt(
        mTask.delayTimer.i2cCallBack,
        mTask.delayTimer.next_state,
        SUCCESS_EVT);
}

static int yas537_cont_start_wait(
    I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)

{
    registerDelayTimer(
        mTask.yascore.measure_time_worst[mTask.yascore.average] * 1000,
        delayCallback_cont_start_wait,
        i2cCallBack, next_state);
    magTimeBufferReset(&mTask.magTimeBuf);    //reset time buffer
    return 0;
}

static int yas537_read_yas537_req(I2cCallbackF i2cCallBack, void *next_state)
{
    mTask.txBuf[0] = YAS537_REG_DATAR;
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr,
                         mTask.txBuf, 1,
                         mTask.rxBuf, 8,
                         i2cCallBack,
                         next_state);
}

static int yas537_read_yas537_recv(int *busy, uint16_t *t, uint16_t *xy1y2,
                                   int *ouflow, int corr)
{
    struct yas537_cal *c = &mTask.yascore.cal;
    uint8_t *data = mTask.rxBuf;
    int32_t s[3], h[3];
    int i;

    *busy = data[2] >> 7;
    *t = (uint16_t)((data[0] << 8) | data[1]);
    xy1y2[0] = (uint16_t)(((data[2] & 0x3f) << 8) | data[3]);
    xy1y2[1] = (uint16_t)((data[4] << 8) | data[5]);
    xy1y2[2] = (uint16_t)((data[6] << 8) | data[7]);

    for (i = 0; i < AXES_NUM; i++)
        s[i] = xy1y2[i] - 8192;
    *ouflow = 0;
    for (i = 0; i < AXES_NUM; i++) {
        h[i] = (c->k * (c->a[i * 3] * s[0] + c->a[i * 3 + 1] * s[1]
                        + c->a[i * 3 + 2] * s[2])) / 8192;
        h[i] = CLIP(h[i], -8192, 8191) + 8192;
        if (corr)
            xy1y2[i] = h[i];
        if (mTask.yascore.overflow[i] <= h[i])
            *ouflow |= (1 << (i * 2));
        if (h[i] <= mTask.yascore.underflow[i])
            *ouflow |= (1 << (i * 2 + 1));
    }

    return 0;
}

static void delayCallback_read_yas537_req(uint32_t timerId, void *data)
{
    mTask.delayTimer.timerHandle = 0;

    yas537_read_yas537_req(
        mTask.delayTimer.i2cCallBack,
        mTask.delayTimer.next_state);
}

static int yas537_single_read1(int ldtc, int fors,
                               I2cCallbackF i2cCallBack,  void *next_state)
{
    return yas537_start_yas537(ldtc, fors, 0,
                               i2cCallBack, next_state);
}

static int yas537_single_read2_with_delay(
    I2cCallbackF i2cCallBack, void *next_state)
{
    registerDelayTimer(
        mTask.yascore.measure_time_worst[mTask.yascore.average] * 1000,
        delayCallback_read_yas537_req,
        i2cCallBack, next_state);
    return 0;
}

static int yas537_single_read2(I2cCallbackF i2cCallBack,  void *next_state)
{
    return yas537_read_yas537_req(i2cCallBack, next_state);
}


static int yas537_single_read3(int *busy, uint16_t *t, uint16_t *xy1y2,
                               int *ouflow, int corr)
{
    return yas537_read_yas537_recv(busy, t, xy1y2, ouflow, corr);
}

static int yas537_rcoil_req1(int mode,
                             I2cCallbackF i2cCallBack, void *next_state)
{
    struct yas537_cal *c = &mTask.yascore.cal;
    uint8_t mode0 = c->ver == 1 ? 0x02 : 0x00;
    uint8_t mode1 = c->ver == 1 ? 0x00 : 0x02;

    mTask.txBuf[0] = YAS537_REG_RCMR;
    mTask.txBuf[1] = mode == 0 ? mode0 : mode1;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                       mTask.txBuf, 2, i2cCallBack, next_state);
}

static void delayCallback_rcoil_req2(uint32_t timerId, void *data)
{
    mTask.delayTimer.timerHandle = 0;

    mTask.txBuf[0] = YAS537_REG_CONFR;
    mTask.txBuf[1] = 0x08;
    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                mTask.txBuf, 2,
                mTask.delayTimer.i2cCallBack,
                mTask.delayTimer.next_state);
}

static int yas537_rcoil_req2_with_delay(
    I2cCallbackF i2cCallBack, void *next_state)
{
    registerDelayTimer(
        YAS537_MAG_RCOIL_TIME * 1000, delayCallback_rcoil_req2,
        i2cCallBack, next_state);
    return 0;
}

static int yas537_rcoil_req2(I2cCallbackF i2cCallBack, void *next_state)
{
    mTask.txBuf[0] = YAS537_REG_CONFR;
    mTask.txBuf[1] = 0x08;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                       mTask.txBuf, 2, i2cCallBack, next_state);
}

static int yas537_reset(
    I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)

{
    const uint8_t avrr[] = {0x50, 0x60, 0x70, 0x71, 0x72, 0x73};


    switch (mTask.state_reset++) {
        case 0:
            mTask.txBuf[0] = YAS537_REG_SRSTR;
            mTask.txBuf[1] = 0x02;
            return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                               mTask.txBuf, 2, i2cCallBack, next_state);
        case 1:
            mTask.txBuf[0] = YAS537_REG_ADCCALR;
            mTask.txBuf[1] = 0x03;
            return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                               mTask.txBuf, 2, i2cCallBack, next_state);
        case 2:
            mTask.txBuf[0] = YAS537_REG_ADCCALR + 1;
            mTask.txBuf[1] = 0xf8;
            return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                               mTask.txBuf, 2, i2cCallBack, next_state);
        case 3:
            mTask.txBuf[0] = YAS537_REG_MTCR;
            mTask.txBuf[1] = mTask.yascore.cal.data[0];
            return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                               mTask.txBuf, 2, i2cCallBack, next_state);
        case 4:
            mTask.txBuf[0] = YAS537_REG_MTCR + 1;
            mTask.txBuf[1] = mTask.yascore.cal.data[1];
            return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                               mTask.txBuf, 2, i2cCallBack, next_state);
        case 5:
            mTask.txBuf[0] = YAS537_REG_MTCR + 2;
            mTask.txBuf[1] = mTask.yascore.cal.data[2];
            return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                               mTask.txBuf, 2, i2cCallBack, next_state);
        case 6:
            mTask.txBuf[0] = YAS537_REG_MTCR + 3;
            mTask.txBuf[1] = (mTask.yascore.cal.data[3] & 0xe0) | 0x10;
            return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                               mTask.txBuf, 2, i2cCallBack, next_state);
        case 7:
            mTask.txBuf[0] = YAS537_REG_OXR;
            mTask.txBuf[1] = mTask.yascore.cal.data[12];
            return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                               mTask.txBuf, 2, i2cCallBack, next_state);
        case 8:
            mTask.txBuf[0] = YAS537_REG_OXR + 1;
            mTask.txBuf[1] = mTask.yascore.cal.data[13];
            return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                               mTask.txBuf, 2, i2cCallBack, next_state);
        case 9:
            mTask.txBuf[0] = YAS537_REG_OXR + 2;
            mTask.txBuf[1] = mTask.yascore.cal.data[14];
            return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                               mTask.txBuf, 2, i2cCallBack, next_state);
        case 10:
            mTask.txBuf[0] = YAS537_REG_HCKR;
            mTask.txBuf[1] = (mTask.yascore.cal.data[15] >> 3) & 0x1e;
            return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                               mTask.txBuf, 2, i2cCallBack, next_state);
        case 11:
            mTask.txBuf[0] = YAS537_REG_LCKR;
            mTask.txBuf[1] = (mTask.yascore.cal.data[15] << 1) & 0x1e;
            return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                               mTask.txBuf, 2, i2cCallBack, next_state);
        case 12:
            mTask.txBuf[0] = YAS537_REG_OCR;
            mTask.txBuf[1] = mTask.yascore.cal.data[16] & 0x3f;
            return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                               mTask.txBuf, 2, i2cCallBack, next_state);
        case 13:
            mTask.txBuf[0] = YAS537_REG_TRMR;
            mTask.txBuf[1] = 0xff;
            return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                               mTask.txBuf, 5, i2cCallBack, next_state);
        case 14:
            mTask.txBuf[0] = YAS537_REG_INTRVLR;
            mTask.txBuf[1] = yascore_update_intrvlr(mTask.yascore.delay);
            return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                               mTask.txBuf, 2, i2cCallBack, next_state);
        case 15:
            mTask.txBuf[0] = YAS537_REG_AVRR;
            mTask.txBuf[1] = avrr[mTask.yascore.average];
            return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                               mTask.txBuf, 2, i2cCallBack, next_state);

        default:
            osLog(LOG_ERROR, "invalid state_reset\n");
            break;
    }

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int yas537_enable_rcoil(
    I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)

{
    uint16_t t;
    int busy, overflow;

    switch (mTask.state_enable_rcoil++) {
        case 0:
            if (!mTask.yascore.enable)
                break;
            return yas537_single_read1(0, 0, i2cCallBack, next_state);
        case 1:
            if (!mTask.yascore.enable)
                break;
            return yas537_single_read2_with_delay(i2cCallBack, next_state);
        case 2:
            if (!mTask.yascore.enable)
                break;
            yas537_single_read3(&busy, &t,
                                mTask.enable_rcoil_xy1y2, &mTask.yascore.flowflag, 1);
            if (busy)
                osLog(LOG_WARN, "YAS537 busy in enable_rcoil\n");
            yascore_sensitivity_correction(mTask.enable_rcoil_xy1y2, t);

            if (mTask.yascore.flowflag)
                break;
            return yas537_rcoil_req1(1, i2cCallBack, next_state);
        case 3:
            if (!mTask.yascore.enable || mTask.yascore.flowflag)
                break;
            return yas537_rcoil_req2_with_delay(i2cCallBack, next_state);
        case 4:
            if (!mTask.yascore.enable || mTask.yascore.flowflag)
                break;
            return yas537_single_read1(0, 0, i2cCallBack, next_state);
        case 5:
            if (!mTask.yascore.enable || mTask.yascore.flowflag)
                break;
            return yas537_single_read2_with_delay(i2cCallBack, next_state);
        case 6:
            if (!mTask.yascore.enable)
                break;
            mTask.yascore.invalid_data = 1;
            if (!mTask.yascore.flowflag) {
                uint16_t xy1y2_after[3];

                yas537_single_read3(
                    &busy, &t, xy1y2_after, &overflow, 1);
                if (busy)
                    osLog(LOG_WARN, \
                          "YAS537 busy in enable_rcoil\n");

                yascore_sensitivity_correction(xy1y2_after, t);
                set_vector(mTask.yascore.last_after_rcoil,
                           xy1y2_after);
                if (!yascore_invalid_magnetic_field(
                            mTask.enable_rcoil_xy1y2, xy1y2_after)
                        && !overflow) {
                    mTask.yascore.flowflag = 0;
                    mTask.yascore.invalid_data = 0;
                    mTask.yascore.measure_state
                        = YAS537_MAG_STATE_NORMAL;
                }
            }

            if (!mTask.yascore.invalid_data)
                break;
            return yas537_rcoil_req1(0, i2cCallBack, next_state);
        case 8:
        case 10:
        case 12:
        case 14:
        case 16:
            if (!mTask.yascore.enable || !mTask.yascore.invalid_data)
                break;
            return yas537_rcoil_req1(0, i2cCallBack, next_state);
        case 7:
        case 9:
        case 11:
        case 13:
        case 15:
        case 17:
            if (!mTask.yascore.enable || !mTask.yascore.invalid_data)
                break;
            return yas537_rcoil_req2_with_delay(i2cCallBack, next_state);
        case 18:
            if (!mTask.yascore.enable || !mTask.yascore.invalid_data)
                break;
            return yas537_single_read1(0, 0, i2cCallBack, next_state);
        case 19:
            if (!mTask.yascore.enable || !mTask.yascore.invalid_data)
                break;
            return yas537_single_read2_with_delay(i2cCallBack, next_state);
        case 20:
            if (!mTask.yascore.enable)
                break;
            if (mTask.yascore.invalid_data) {
                uint16_t xy1y2[3];

                yas537_single_read3(&busy, &t, xy1y2, &overflow, 1);
                if (busy)
                    osLog(LOG_WARN, \
                          "YAS537 busy in enable_rcoil\n");

                yascore_sensitivity_correction(xy1y2, t);
                set_vector(mTask.yascore.last_after_rcoil, xy1y2);

                mTask.yascore.invalid_data_time
                    = mTask.yascore.current_time;
                mTask.yascore.flowflag = 1;
                mTask.yascore.measure_state
                    = YAS537_MAG_STATE_INIT_COIL;
            } else {
                mTask.yascore.flowflag = 0;
                mTask.yascore.measure_state = YAS537_MAG_STATE_NORMAL;
            }
            break;

        default:
            osLog(LOG_ERROR, "invalid state_enable_rcoil\n");
            break;
    }

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int yas537_sample_mode_initcoil(
    I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int step, step_rcoil_end, rcoil_mode;

    switch (mTask.state_sample.mode) {
        case YAS537_MODE_SAMPLE_INIT_COIL:
            step_rcoil_end = 8;
            rcoil_mode = 1;
            break;
        case YAS537_MODE_SAMPLE_INIT_COIL_OF:
            step_rcoil_end = 13;
            rcoil_mode = 0;
            break;

        case YAS537_MODE_SAMPLE_NORMAL:
        default:
            goto error_exit;
    }

    step = mTask.state_sample.step;
    if (step == 0) {
        mTask.state_reset = 0;
        return yas537_reset(i2cCallBack, spiCallBack, next_state,
                            inBuf, inSize, elemInSize,
                            outBuf, outSize, elemOutSize);
    } else if (1 <= step && step <= 7)
        return yas537_reset(i2cCallBack, spiCallBack, next_state,
                            inBuf, inSize, elemInSize,
                            outBuf, outSize, elemOutSize);
    else if (8 <= step && step <= step_rcoil_end)
        return yas537_rcoil_req1(rcoil_mode, i2cCallBack, next_state);
    else if (step == step_rcoil_end + 1) {
        sensorFsmEnqueueFakeI2cEvt(
            i2cCallBack, next_state, SUCCESS_EVT);
        return 0;
    } else if (step == step_rcoil_end + 2)
        return yas537_single_read2(i2cCallBack, next_state);
    else
        osLog(LOG_ERROR, "invalid state_sample.step\n");

error_exit:
    mTask.state_sample.mode = YAS537_MODE_SAMPLE_NORMAL;
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int yas537_convert_mode_initcoil(
    I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    uint16_t xy1y2[3];
    int busy, overflow;
    uint16_t t;
    int step, step_rcoil_end;

    switch (mTask.state_sample.mode) {
        case YAS537_MODE_SAMPLE_INIT_COIL:
            step_rcoil_end = 8;
            break;
        case YAS537_MODE_SAMPLE_INIT_COIL_OF:
            step_rcoil_end = 13;
            break;

        case YAS537_MODE_SAMPLE_NORMAL:
        default:
            goto error_exit;
    }

    step = mTask.state_sample.step++;
    if (0 <= step && step <= 7)
        return yas537_reset(i2cCallBack, spiCallBack, next_state,
                            inBuf, inSize, elemInSize,
                            outBuf, outSize, elemOutSize);
    else if (8 <= step && step <= step_rcoil_end)
        return yas537_rcoil_req2(i2cCallBack, next_state);
    else if (step == step_rcoil_end + 1)
        return yas537_single_read1(0, 0, i2cCallBack, next_state);
    else if (step == step_rcoil_end + 2) {
        yas537_single_read3(&busy, &t, xy1y2, &overflow, 1);
        if (busy) {
            if (mTask.yascore.busy_retry++
                    < YAS537_BUSY_RETRY_MAX) {
                mTask.state_sample.step--;
                osLog(LOG_WARN, "yas537 busy. Retrying.\n");
                sensorFsmEnqueueFakeI2cEvt(
                    i2cCallBack, next_state, SUCCESS_EVT);
                return 0;
            }
            osLog(LOG_ERROR, "yas537 busy.\n");
        }
        mTask.yascore.busy_retry = 0;

        yascore_sensitivity_correction(xy1y2, t);
        set_vector(mTask.yascore.last_after_rcoil, xy1y2);
        if (!overflow) {
            mTask.yascore.flowflag = 0;
            mTask.yascore.invalid_data = 0;
            mTask.yascore.measure_state = YAS537_MAG_STATE_NORMAL;
        }
        mTask.state_sample.mode = YAS537_MODE_SAMPLE_NORMAL;
        return yas537_cont_start(
                   i2cCallBack, spiCallBack, next_state,
                   inBuf, inSize, elemInSize,
                   outBuf, outSize, elemOutSize);
    } else
        osLog(LOG_ERROR, "invalid state_sample.step\n");

error_exit:
    mTask.state_sample.mode = YAS537_MODE_SAMPLE_NORMAL;
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static void yas537_convert(int32_t *xyz)
{
    uint16_t xy1y2[3], t;
    int32_t tmp_xyz[3];
    int busy, ouflow;
    int i;

    switch (mTask.state_sample.mode) {
        case YAS537_MODE_SAMPLE_INIT_COIL:
        case YAS537_MODE_SAMPLE_INIT_COIL_OF:
            for (i = 0; i < AXES_NUM; i++)
                xyz[i] = mTask.last_xyz[i];
            return;

        case YAS537_MODE_SAMPLE_NORMAL:
            break;
        default:
            osLog(LOG_ERROR, "invalid state_sample.mode\n");
            mTask.state_sample.mode = YAS537_MODE_SAMPLE_NORMAL;
            break;
    }

    yas537_read_yas537_recv(&busy, &t, xy1y2, &ouflow, 1);
    if (busy) {
        osLog(LOG_WARN, "yas537Convert: device busy\n");
        for (i = 0; i < AXES_NUM; i++)
            xyz[i] = mTask.last_xyz[i];
        return;
    }

    yascore_sensitivity_correction(xy1y2, t);
    yascore_xy1y2_to_xyz(xy1y2, tmp_xyz);

    xyz[mTask.cvt.map[AXIS_X]] = mTask.cvt.sign[AXIS_X] * tmp_xyz[AXIS_X];
    xyz[mTask.cvt.map[AXIS_Y]] = mTask.cvt.sign[AXIS_Y] * tmp_xyz[AXIS_Y];
    xyz[mTask.cvt.map[AXIS_Z]] = mTask.cvt.sign[AXIS_Z] * tmp_xyz[AXIS_Z];

    yascore_apply_matrix(xyz, &mTask.yascore.static_matrix);
    for (i = 0; i < AXES_NUM; i++) {
        xyz[i] -= xyz[i] % 10;
        if (ouflow & (1 << (i * 2)))
            xyz[i] += 1; /* set overflow */
        if (ouflow & (1 << (i * 2 + 1)))
            xyz[i] += 2; /* set underflow */
    }

    if (ouflow || yascore_invalid_magnetic_field(
                xy1y2, mTask.yascore.last_after_rcoil)) {
        if (!mTask.yascore.invalid_data) {
            osLog(LOG_INFO, "yas537Convert: detect invalid data\n");
            mTask.yascore.invalid_data_time
                = mTask.yascore.current_time;
            mTask.yascore.invalid_data = 1;
        }
        if (ouflow) {
            osLog(LOG_INFO, "yas537Convert: detect overflow\n");
            mTask.yascore.flowflag = 1;
        }
        mTask.yascore.measure_state = YAS537_MAG_STATE_INIT_COIL;
    }

    for (i = 0; i < AXES_NUM; i++)
        mTask.last_xyz[i] = xyz[i];
}

static int yas537Reset(
    I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.state_reset = 0;
    return yas537_reset(i2cCallBack, spiCallBack, next_state,
                        inBuf, inSize, elemInSize,
                        outBuf, outSize, elemOutSize);
}

static int yas537Enable(
    I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.yascore.enable = 1;
    mTask.state_reset = 0;
    mTask.state_enable_rcoil = 0;
    return yas537_reset(i2cCallBack, spiCallBack, next_state,
                        inBuf, inSize, elemInSize,
                        outBuf, outSize, elemOutSize);
}

static int yas537Disable(
    I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mTask.yascore.enable = 0;

    mTask.txBuf[0] = YAS537_REG_SRSTR;
    mTask.txBuf[1] = 0x02;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                       mTask.txBuf, 2, i2cCallBack, next_state);
}

static int yas537Rate(
    I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    uint32_t sample_rate, water_mark;
    uint32_t sample_period;
    struct magCntlPacket cntlPacket;

    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf,
                        outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "yas537Rate, rx inSize and elemSize error\n");
        return -1;
    }

    sample_rate = cntlPacket.rate;
    water_mark = cntlPacket.waterMark;
    sample_period = 1024 * 1000 / sample_rate;
    mTask.yascore.delay = sample_period;

    osLog(LOG_INFO, "yas537Rate: %u, water_mark:%u, sample_period=%u\n", \
          sample_rate, water_mark, mTask.yascore.delay);

    mTask.state_reset = 0;
    mTask.state_enable_rcoil = 0;
    return yas537_reset(i2cCallBack, spiCallBack, next_state,
                        inBuf, inSize, elemInSize,
                        outBuf, outSize, elemOutSize);
}

static int yas537Sample(
    I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret;

    ret = rxTransferDataInfo(&mTask.dataInfo,
                             inBuf, inSize, elemInSize,
                             outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "yas537Sample, rx dataInfo error\n");
        return -1;
    }
    mTask.yascore.current_time += mTask.yascore.delay;

    switch (mTask.state_sample.mode) {
        case YAS537_MODE_SAMPLE_NORMAL:
            if (mTask.yascore.measure_state == YAS537_MAG_STATE_INIT_COIL
                    && YAS537_MAG_INITCOIL_TIMEOUT
                    <= mTask.yascore.current_time
                    - mTask.yascore.invalid_data_time) {
                mTask.yascore.invalid_data_time
                    = mTask.yascore.current_time;
                mTask.state_sample.mode = mTask.yascore.flowflag
                                          ? YAS537_MODE_SAMPLE_INIT_COIL_OF
                                          : YAS537_MODE_SAMPLE_INIT_COIL;
                mTask.state_sample.step = 0;
                return yas537_sample_mode_initcoil(
                           i2cCallBack, spiCallBack, next_state,
                           inBuf, inSize, elemInSize,
                           outBuf, outSize, elemOutSize);
            }
            break;
        case YAS537_MODE_SAMPLE_INIT_COIL:
        case YAS537_MODE_SAMPLE_INIT_COIL_OF:
            return yas537_sample_mode_initcoil(
                       i2cCallBack, spiCallBack, next_state,
                       inBuf, inSize, elemInSize,
                       outBuf, outSize, elemOutSize);
        default:
            mTask.state_sample.mode = YAS537_MODE_SAMPLE_NORMAL;
            osLog(LOG_ERROR, "invalid state in yas537Sample\n");
            break;
    }

    return yas537_read_yas537_req(i2cCallBack, next_state);
}

static int yas537Convert(
    I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    struct magData *data = mTask.magPacket.outBuf;
    int32_t remap_data[AXES_NUM];
    uint64_t timestamp = 0;
    uint8_t data_size = 0;

    yas537_convert(remap_data);

    timestamp = rtcGetTime();
    /*
    osLog(LOG_DEBUG, "msensor raw data: %d, %d, %d, timestamp: %u!\n", \
        remap_data[AXIS_X], remap_data[AXIS_Y], remap_data[AXIS_Z], \
        (uint32_t) (timestamp / 1000 / 1000));
    */

    data_size = magTimeBufferSize(&mTask.magTimeBuf);
    for (uint8_t i = 0; i < data_size; i++) {
        data[i].x = remap_data[AXIS_X];
        data[i].y = remap_data[AXIS_Y];
        data[i].z = remap_data[AXIS_Z];
        magTimeBufferRead(&mTask.magTimeBuf, &mTask.hwSampleTime);
        timestamp = addThenRetreiveAverageMagTimeStamp(mTask.hwSampleTime);
    }

    txTransferDataInfo(&mTask.dataInfo, data_size, timestamp, data);
    /*osLog(LOG_ERROR, "yas537Convert raw data: %f, %f, %f, timestamp: %llu size: %u!\n", (double)remap_data[AXIS_X],
    (double)remap_data[AXIS_Y], (double)remap_data[AXIS_Z], timestamp, data_size); */

    mTask.factoryData.ix
        = (int32_t)(remap_data[AXIS_X] * MAGNETOMETER_INCREASE_NUM_AP);
    mTask.factoryData.iy
        = (int32_t)(remap_data[AXIS_Y] * MAGNETOMETER_INCREASE_NUM_AP);
    mTask.factoryData.iz
        = (int32_t)(remap_data[AXIS_Z] * MAGNETOMETER_INCREASE_NUM_AP);

    switch (mTask.state_sample.mode) {
        case YAS537_MODE_SAMPLE_NORMAL:
            break;
        case YAS537_MODE_SAMPLE_INIT_COIL:
        case YAS537_MODE_SAMPLE_INIT_COIL_OF:
            return yas537_convert_mode_initcoil(
                       i2cCallBack, spiCallBack, next_state,
                       inBuf, inSize, elemInSize,
                       outBuf, outSize, elemOutSize);
        default:
            mTask.state_sample.mode = YAS537_MODE_SAMPLE_NORMAL;
            osLog(LOG_ERROR, "invalid state_sample.mode\n");
            break;
    }

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}


static struct sensorFsm yas537Fsm[] = {
    sensorFsmCmd(STATE_SAMPLE, STATE_CONVERT, yas537Sample),
    sensorFsmCmd(STATE_CONVERT, STATE_SAMPLE_DONE, yas537Convert),

    sensorFsmCmd(STATE_ENABLE, STATE_ENABLE_RESET1, yas537Enable),
    sensorFsmCmd(STATE_ENABLE_RESET1, STATE_ENABLE_RESET2, yas537_reset),
    sensorFsmCmd(STATE_ENABLE_RESET2, STATE_ENABLE_RESET3, yas537_reset),
    sensorFsmCmd(STATE_ENABLE_RESET3, STATE_ENABLE_RESET4, yas537_reset),
    sensorFsmCmd(STATE_ENABLE_RESET4, STATE_ENABLE_RESET5, yas537_reset),
    sensorFsmCmd(STATE_ENABLE_RESET5, STATE_ENABLE_RESET6, yas537_reset),
    sensorFsmCmd(STATE_ENABLE_RESET6, STATE_ENABLE_RESET7, yas537_reset),
    sensorFsmCmd(STATE_ENABLE_RESET7, STATE_ENABLE_RESET8, yas537_reset),
    sensorFsmCmd(STATE_ENABLE_RESET8, STATE_ENABLE_RESET9, yas537_reset),
    sensorFsmCmd(STATE_ENABLE_RESET9, STATE_ENABLE_RESET10, yas537_reset),
    sensorFsmCmd(STATE_ENABLE_RESET10, STATE_ENABLE_RESET11, yas537_reset),
    sensorFsmCmd(STATE_ENABLE_RESET11, STATE_ENABLE_RESET12, yas537_reset),
    sensorFsmCmd(STATE_ENABLE_RESET12, STATE_ENABLE_RESET13, yas537_reset),
    sensorFsmCmd(STATE_ENABLE_RESET13, STATE_ENABLE_RESET14, yas537_reset),
    sensorFsmCmd(STATE_ENABLE_RESET14, STATE_ENABLE_RESET15, yas537_reset),
    sensorFsmCmd(STATE_ENABLE_RESET15, STATE_ENABLE_RCOIL1, yas537_reset),
    sensorFsmCmd(STATE_ENABLE_RCOIL1, STATE_ENABLE_RCOIL2, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL2, STATE_ENABLE_RCOIL3, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL3, STATE_ENABLE_RCOIL4, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL4, STATE_ENABLE_RCOIL5, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL5, STATE_ENABLE_RCOIL6, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL6, STATE_ENABLE_RCOIL7, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL7, STATE_ENABLE_RCOIL8, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL8, STATE_ENABLE_RCOIL9, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL9, STATE_ENABLE_RCOIL10, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL10, STATE_ENABLE_RCOIL11, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL11, STATE_ENABLE_RCOIL12, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL12, STATE_ENABLE_RCOIL13, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL13, STATE_ENABLE_RCOIL14, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL14, STATE_ENABLE_RCOIL15, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL15, STATE_ENABLE_RCOIL16, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL16, STATE_ENABLE_RCOIL17, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL17, STATE_ENABLE_RCOIL18, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL18, STATE_ENABLE_RCOIL19, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL19, STATE_ENABLE_RCOIL20, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL20, STATE_ENABLE_RCOIL21, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_RCOIL21, STATE_ENABLE_CONT_START, yas537_enable_rcoil),
    sensorFsmCmd(STATE_ENABLE_CONT_START, STATE_ENABLE_CONT_START_WAIT, yas537_cont_start),
    sensorFsmCmd(STATE_ENABLE_CONT_START_WAIT, STATE_ENABLE_DONE, yas537_cont_start_wait),

    sensorFsmCmd(STATE_DISABLE, STATE_DISABLE_DONE, yas537Disable),

    sensorFsmCmd(STATE_RATECHG, STATE_RATECHG_RESET1, yas537Rate),
    sensorFsmCmd(STATE_RATECHG_RESET1, STATE_RATECHG_RESET2, yas537_reset),
    sensorFsmCmd(STATE_RATECHG_RESET2, STATE_RATECHG_RESET3, yas537_reset),
    sensorFsmCmd(STATE_RATECHG_RESET3, STATE_RATECHG_RESET4, yas537_reset),
    sensorFsmCmd(STATE_RATECHG_RESET4, STATE_RATECHG_RESET5, yas537_reset),
    sensorFsmCmd(STATE_RATECHG_RESET5, STATE_RATECHG_RESET6, yas537_reset),
    sensorFsmCmd(STATE_RATECHG_RESET6, STATE_RATECHG_RESET7, yas537_reset),
    sensorFsmCmd(STATE_RATECHG_RESET7, STATE_RATECHG_RESET8, yas537_reset),
    sensorFsmCmd(STATE_RATECHG_RESET8, STATE_RATECHG_RESET9, yas537_reset),
    sensorFsmCmd(STATE_RATECHG_RESET9, STATE_RATECHG_RESET10, yas537_reset),
    sensorFsmCmd(STATE_RATECHG_RESET10, STATE_RATECHG_RESET11, yas537_reset),
    sensorFsmCmd(STATE_RATECHG_RESET11, STATE_RATECHG_RESET12, yas537_reset),
    sensorFsmCmd(STATE_RATECHG_RESET12, STATE_RATECHG_RESET13, yas537_reset),
    sensorFsmCmd(STATE_RATECHG_RESET13, STATE_RATECHG_RESET14, yas537_reset),
    sensorFsmCmd(STATE_RATECHG_RESET14, STATE_RATECHG_RESET15, yas537_reset),
    sensorFsmCmd(STATE_RATECHG_RESET15, STATE_RATECHG_RCOIL1, yas537_reset),
    sensorFsmCmd(STATE_RATECHG_RCOIL1, STATE_RATECHG_RCOIL2, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL2, STATE_RATECHG_RCOIL3, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL3, STATE_RATECHG_RCOIL4, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL4, STATE_RATECHG_RCOIL5, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL5, STATE_RATECHG_RCOIL6, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL6, STATE_RATECHG_RCOIL7, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL7, STATE_RATECHG_RCOIL8, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL8, STATE_RATECHG_RCOIL9, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL9, STATE_RATECHG_RCOIL10, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL10, STATE_RATECHG_RCOIL11, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL11, STATE_RATECHG_RCOIL12, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL12, STATE_RATECHG_RCOIL13, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL13, STATE_RATECHG_RCOIL14, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL14, STATE_RATECHG_RCOIL15, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL15, STATE_RATECHG_RCOIL16, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL16, STATE_RATECHG_RCOIL17, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL17, STATE_RATECHG_RCOIL18, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL18, STATE_RATECHG_RCOIL19, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL19, STATE_RATECHG_RCOIL20, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL20, STATE_RATECHG_RCOIL21, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_RCOIL21, STATE_RATECHG_CONT_START, yas537_enable_rcoil),
    sensorFsmCmd(STATE_RATECHG_CONT_START, STATE_RATECHG_CONT_START_WAIT, yas537_cont_start),
    sensorFsmCmd(STATE_RATECHG_CONT_START_WAIT, STATE_RATECHG_DONE, yas537_cont_start_wait),

    sensorFsmCmd(STATE_RESET, STATE_RESET1, yas537Reset),
    sensorFsmCmd(STATE_RESET1, STATE_RESET2, yas537_reset),
    sensorFsmCmd(STATE_RESET2, STATE_RESET3, yas537_reset),
    sensorFsmCmd(STATE_RESET3, STATE_RESET4, yas537_reset),
    sensorFsmCmd(STATE_RESET4, STATE_RESET5, yas537_reset),
    sensorFsmCmd(STATE_RESET5, STATE_RESET6, yas537_reset),
    sensorFsmCmd(STATE_RESET6, STATE_RESET7, yas537_reset),
    sensorFsmCmd(STATE_RESET7, STATE_RESET8, yas537_reset),
    sensorFsmCmd(STATE_RESET8, STATE_RESET9, yas537_reset),
    sensorFsmCmd(STATE_RESET9, STATE_RESET10, yas537_reset),
    sensorFsmCmd(STATE_RESET10, STATE_RESET11, yas537_reset),
    sensorFsmCmd(STATE_RESET11, STATE_RESET12, yas537_reset),
    sensorFsmCmd(STATE_RESET12, STATE_RESET13, yas537_reset),
    sensorFsmCmd(STATE_RESET13, STATE_RESET14, yas537_reset),
    sensorFsmCmd(STATE_RESET14, STATE_RESET15, yas537_reset),
    sensorFsmCmd(STATE_RESET15, STATE_CORE, yas537_reset),
    sensorFsmCmd(STATE_CORE, STATE_INIT_DONE, yas537RegisterCore),
};

static int yas537Init_ReadCALR(void)
{
    struct yas537_cal *c = &mTask.yascore.cal;
    uint8_t *data = c->data;
    int16_t cxy1y2[3];
    int32_t of[3], efxy1y2[3];
    int cal_valid = 0;
    int ret;
    int i;

    mTask.txBuf[0] = YAS537_REG_CALR;
    ret = i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr,
                            mTask.txBuf, 1, data, 8, NULL, NULL);
    if (ret < 0)
        return ret;

    mTask.txBuf[0] = YAS537_REG_CALR + 8;
    ret = i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr,
                            mTask.txBuf, 1, data + 8, 8, NULL, NULL);
    if (ret < 0)
        return ret;

    mTask.txBuf[0] = YAS537_REG_CALR + 16;
    ret = i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr,
                            mTask.txBuf, 1, data + 16, 1, NULL, NULL);
    if (ret < 0)
        return ret;

    c->ver = data[16] >> 6;
    for (i = 0; i < 17; i++) {
        if (i < 16 && data[i] != 0)
            cal_valid = 1;
        if (i == 16 && (data[i] & 0x3f) != 0)
            cal_valid = 1;
    }
    if (!cal_valid || (c->ver != 1 && c->ver != 3))
        return YAS_ERROR_CALREG;

    cxy1y2[0] = ((data[0] << 1) | (data[1] >> 7)) - 256;
    cxy1y2[1] = (((data[1] << 2) & 0x1fc) | (data[2] >> 6)) - 256;
    cxy1y2[2] = (((data[2] << 3) & 0x1f8) | (data[3] >> 5)) - 256;
    c->a[0] = 128;
    c->a[1] = (((data[3] << 2) & 0x7c) | (data[4] >> 6)) - 64;
    c->a[2] = (((data[4] << 1) & 0x7e) | (data[5] >> 7)) - 64;
    c->a[3] = (((data[5] << 1) & 0xfe) | (data[6] >> 7)) - 128;
    c->a[4] = (((data[6] << 2) & 0x1fc) | (data[7] >> 6)) - 112;
    c->a[5] = (((data[7] << 1) & 0x7e) | (data[8] >> 7)) - 64;
    c->a[6] = (((data[8] << 1) & 0xfe) | (data[9] >> 7)) - 128;
    c->a[7] = (data[9] & 0x7f) - 64;
    c->a[8] = (((data[10] << 1) & 0x1fe) | (data[11] >> 7)) - 112;
    c->k = data[11] & 0x7f;
    for (i = 0; i < AXES_NUM; i++)
        efxy1y2[i] = 8000 - (int32_t)ABS(cxy1y2[i]) * 225 / 16;
    of[0] = c->k * (c->a[0] * efxy1y2[0] - ABS(c->a[1]) * efxy1y2[1]
                    - ABS(c->a[2]) * efxy1y2[2]) / 8192;
    of[1] = c->k * (-ABS(c->a[3]) * efxy1y2[0] + c->a[4] * efxy1y2[1]
                    - ABS(c->a[5]) * efxy1y2[2]) / 8192;
    of[2] = c->k * (-ABS(c->a[6]) * efxy1y2[0] - ABS(c->a[7]) * efxy1y2[1]
                    + c->a[8] * efxy1y2[2]) / 8192;
    for (i = 0; i < AXES_NUM; i++) {
        mTask.yascore.hard_offset[i] = data[i + 12];
        if (YAS537_DATA_OVERFLOW < 8192 + of[i])
            mTask.yascore.overflow[i] = YAS537_DATA_OVERFLOW;
        else
            mTask.yascore.overflow[i] = (int16_t)(8192 + of[i]);
        if (8192 - of[i] < YAS537_DATA_UNDERFLOW)
            mTask.yascore.underflow[i] = YAS537_DATA_UNDERFLOW;
        else
            mTask.yascore.underflow[i] = (int16_t)(8192 - of[i]);
    }

    return ret;
}

static int yas537Init(void)
{
    int ret = 0;
    int i;

    yas537DebugPoint = &mTask;
    insertMagicNum(&mTask.magPacket);

    mTask.hw = get_cust_mag("yas537");
    if (NULL == mTask.hw) {
        osLog(LOG_ERROR, "yas537 get_cust_mag fail\n");
        ret = -1;
        goto err_out;
    }

    mTask.i2c_addr = mTask.hw->i2c_addr[0];
    osLog(LOG_INFO, "mag i2c_num: %d, i2c_addr: 0x%x\n", \
          mTask.hw->i2c_num, mTask.i2c_addr);

    mTask.hw->direction = 0;

    ret = sensorDriverGetConvert(mTask.hw->direction, &mTask.cvt);
    if (ret) {
        osLog(LOG_ERROR, "invalid direction: %d\n", \
              mTask.hw->direction);
    }
    osLog(LOG_INFO, "mag map[0]:%d, map[1]:%d, map[2]:%d\n", \
          mTask.cvt.map[AXIS_X], mTask.cvt.map[AXIS_Y], mTask.cvt.map[AXIS_Z]);
    osLog(LOG_INFO, "mag sign[0]:%d, sign[1]:%d, sign[2]:%d\n", \
          mTask.cvt.sign[AXIS_X], mTask.cvt.sign[AXIS_Y], mTask.cvt.sign[AXIS_Z]);

    i2cMasterRequest(mTask.hw->i2c_num, I2C_SPEED);

    /* read sensor id */
    mTask.txBuf[0] = YAS537_REG_DIDR;
    for (i = 0; i < AXES_NUM; i++) {
        ret = i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr,
                                mTask.txBuf, 1, (uint8_t *) mTask.autoDetect,
                                1, NULL, NULL);
        if (ret >= 0)
            break;
    }
    if (ret < 0) {
        osLog(LOG_ERROR, "fail to read DIDR\n");
        ret = -1;
        i2cMasterRelease(mTask.hw->i2c_num);
        sendSensorErrToAp(ERR_SENSOR_MAG, ERR_CASE_MAG_INIT, MAG_NAME);
        goto err_out;
    }
    /* judge the sensor id */
    if (mTask.autoDetect[0] == YAS537_DEVICE_ID) {
        const struct yas_matrix no_conversion
            = { {10000, 0, 0, 0, 10000, 0, 0, 0, 10000} };
        const int measure_time_worst[]
            = {800, 1100, 1500, 3000, 6000, 12000};

        osLog(LOG_INFO, "yas537: auto detect success, id=0x%x\n", \
              mTask.autoDetect[0]);

        mTask.delayTimer.timerHandle = 0;
        mTask.last_xyz[0] = mTask.last_xyz[1] = mTask.last_xyz[2] = 0;

        mTask.state_enable_rcoil = 0;
        mTask.state_reset = 0;
        mTask.state_sample.mode = YAS537_MODE_SAMPLE_NORMAL;
        mTask.state_sample.step = 0;


        /* yas537 core driver initialize */
        ret = yas537Init_ReadCALR();
        if (ret < 0) {
            ret = -1;
            i2cMasterRelease(mTask.hw->i2c_num);
            sendSensorErrToAp(
                ERR_SENSOR_MAG, ERR_CASE_MAG_INIT, MAG_NAME);
            goto err_out;
        }

        mTask.yascore.measure_state = YAS537_MAG_STATE_NORMAL;
        mTask.yascore.current_time = 0;
        mTask.yascore.invalid_data = 0;
        mTask.yascore.invalid_data_time = 0;
        mTask.yascore.delay = YAS_DEFAULT_SENSOR_DELAY;
        mTask.yascore.enable = 0;
        mTask.yascore.average = YAS537_MAG_AVERAGE_32;
        mTask.yascore.flowflag = 0;
        mTask.yascore.busy_retry = 0;
        for (i = 0; i < AXES_NUM; i++)
            mTask.yascore.last_after_rcoil[i] = 0;
        for (i = 0; i < 6; i++)
            mTask.yascore.measure_time_worst[i]
                = measure_time_worst[i];
        mTask.yascore.static_matrix = no_conversion;
        goto success_out;
    } else {
        ret = -1;
        i2cMasterRelease(mTask.hw->i2c_num);
        sendSensorErrToAp(ERR_SENSOR_MAG, ERR_CASE_MAG_INIT, MAG_NAME);
        osLog(LOG_ERROR, "yas537: auto detect fail\n");
        goto err_out;
    }

success_out:
    mTask.mag_dev_info.layout = 0x00;
    mTask.mag_dev_info.deviceid = 0x10;
    strncpy(mTask.mag_dev_info.libname, "yas",
            sizeof(mTask.mag_dev_info.libname));

    magSensorRegister();
    magRegisterInterruptMode(MAG_UNFIFO);
    registerMagDriverFsm(yas537Fsm, ARRAY_SIZE(yas537Fsm));
    registerMagTimerCbk(yas537TimerCbkF);

err_out:
    return ret;
}

#ifndef CFG_OVERLAY_INIT_SUPPORT
MODULE_DECLARE(yas537, SENS_TYPE_MAG, yas537Init);
#else
#include "mtk_overlay_init.h"
OVERLAY_DECLARE(yas537, OVERLAY_ID_MAG, yas537Init);
#endif
