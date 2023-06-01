/*
 * Copyright (C) 2016 The Android Open Source Project
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
#include <sensors.h>
#include <plat/inc/rtc.h>
#include <math.h>

#include <magnetometer.h>
#include <contexthub_core.h>
#include <cust_mag.h>

#include <vendor_fusion.h>
#include "mmc5603nj.h"

#include "cache_internal.h"

#define MMC5603_DRIVER_VERSION "1.00.20069"

#define I2C_SPEED 400000

enum MMC5603State
{
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

    STATE_FUSE_EN = CHIP_RESET,
    STATE_FUSE_RD,
    STATE_CORE,
    STATE_CHIP_RESET,
    STATE_CHIP_CTRL1,
    STATE_CHIP_CTRL2,
    STATE_CHIP_CTRL3,
    STATE_CHIP_CTRL4,
    STATE_CHIP_BITS,
    STATE_CHIP_TM,
    STATE_CHIP_READ_OTP,
    STATE_CHIP_SAMPLE,
    STATE_CHIP_MMC_CHANGE,
    STATE_CHIP_READ_SA_BIT,
    STATE_CHIP_ST_ENABLE,
    STATE_CHIP_SAMPLE_OVER,
};
enum SensorSate
{
    AUTO_SET_CONTINU = 1,
    MANU_SET_SINGLE = 2,
};
static uint8_t mmc5603_state = AUTO_SET_CONTINU;
SRAM_REGION_BSS static struct MMC5603Task
{
    /* txBuf for i2c operation, fill register and fill value */
    uint8_t txBuf[8];
    /* rxBuf for i2c operation, receive rawdata */
    uint8_t rxBuf[8];
    uint8_t fuse[3];

    bool b_auto_st;
    uint8_t sat_reg;
    uint8_t autoDetect[2];
    uint64_t hwSampleTime;
    struct transferDataInfo dataInfo;
    struct magDataPacket magPacket;
    /* data for factory */
    struct TripleAxisDataPoint factoryData;
    struct mag_hw *hw;
    struct sensorDriverConvert cvt;
    uint8_t i2c_addr;
    struct mag_dev_info_t mag_dev_info;
} mTask;
static struct MMC5603Task *mmc5603DebugPoint;

void mmc5603TimerCbkF(uint64_t time)
{
    mTask.hwSampleTime = time;
}
static void magGetCalibration(int32_t *cali, int32_t size)
{
}
static void magSetCalibration(int32_t *cali, int32_t size)
{
}
static void magGetData(void *sample)
{
    struct TripleAxisDataPoint *tripleSample = (struct TripleAxisDataPoint *)sample;

    tripleSample->ix = mTask.factoryData.ix;
    tripleSample->iy = mTask.factoryData.iy;
    tripleSample->iz = mTask.factoryData.iz;
}

#ifdef VENDOR_EDIT
#define abs(x) (x >= 0 ? (x) : (-x))
static void mmc5603_selftest(int32_t *testResult)
{
    int temp_result = -1;
    unsigned char buf[2] = {0};
    int32_t prev_raw[3] = {0};
    int32_t curr_raw[3] = {0};

     buf[0] = MMC5603NJ_REG_CTRL2;
    buf[1] = MMC5603NJ_CMD_CMM_DISEN;
    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, buf, 2, NULL, NULL);
    vTaskDelay(20);

    buf[0] = MMC5603NJ_REG_CTRL0;
    buf[1] = MMC5603NJ_MANU_RESET;
    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, buf, 2, NULL, NULL);
    vTaskDelay(1);

    buf[0] = MMC5603NJ_REG_CTRL0;
    buf[1] = MMC5603NJ_MANU_TM;
    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, buf, 2, NULL, NULL);
    vTaskDelay(10);

    buf[0] = MMC5603_REG_DATA;
    i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, buf, 1, mTask.rxBuf, 6, NULL, NULL);

    prev_raw[0] = (int)((uint16_t)((mTask.rxBuf[0] << 8) + (uint16_t)(mTask.rxBuf[1])));
    prev_raw[1] = (int)((uint16_t)((mTask.rxBuf[2] << 8) + (uint16_t)(mTask.rxBuf[3])));
    prev_raw[2] = (int)((uint16_t)((mTask.rxBuf[4] << 8) + (uint16_t)(mTask.rxBuf[5])));

    buf[0] = MMC5603NJ_REG_CTRL0;
    buf[1] = MMC5603NJ_MANU_SET;
    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, buf, 2, NULL, NULL);
    vTaskDelay(1);

    buf[0] = MMC5603NJ_REG_CTRL0;
    buf[1] = MMC5603NJ_MANU_TM;
    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, buf, 2, NULL, NULL);
    vTaskDelay(10);

    buf[0] = MMC5603_REG_DATA;
    i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, buf, 1, mTask.rxBuf, 6, NULL, NULL);

    curr_raw[0] = (int)((uint16_t)((mTask.rxBuf[0] << 8) + (uint16_t)(mTask.rxBuf[1])));
    curr_raw[1] = (int)((uint16_t)((mTask.rxBuf[2] << 8) + (uint16_t)(mTask.rxBuf[3])));
    curr_raw[2] = (int)((uint16_t)((mTask.rxBuf[4] << 8) + (uint16_t)(mTask.rxBuf[5])));
    osLog(LOG_ERROR, "%s curr_raw: %d %d %d \n", __func__, curr_raw[0], curr_raw[1], curr_raw[2]);

    osLog(LOG_ERROR, "%s diff: %d %d %d \n", __func__, abs(curr_raw[0] - prev_raw[0]), abs(curr_raw[1] - prev_raw[1]), abs(curr_raw[2] - prev_raw[2]));

    if ((abs(curr_raw[0] - prev_raw[0]) > 100) || (abs(curr_raw[1] - prev_raw[1]) > 100) || (abs(curr_raw[2] - prev_raw[2]) > 100))
    {
        temp_result = 1;
        osLog(LOG_INFO, "%s : MEMSIC5603_selftest pass \n", __func__);
    }
    else
    {
        temp_result = -1;
        osLog(LOG_INFO, "%s : MEMSIC5603_selftest fail \n", __func__);
    }

    *testResult = (int32_t)temp_result;
}
#endif
static void magGetSensorInfo(struct sensorInfo_t *data)
{
    memcpy(&data->mag_dev_info, &mTask.mag_dev_info, sizeof(struct mag_dev_info_t));
}
static int mmc5603RegisterCore(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                               void *inBuf, uint8_t inSize, uint8_t elemInSize,
                               void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    struct sensorCoreInfo mInfo;

    /* Register sensor Core */
    memset(&mInfo, 0x00, sizeof(struct sensorCoreInfo));
    mInfo.sensType = SENS_TYPE_MAG;
    mInfo.getCalibration = magGetCalibration;
    mInfo.setCalibration = magSetCalibration;
    mInfo.getData = magGetData;
    mInfo.getSensorInfo = magGetSensorInfo;
    sensorCoreRegister(&mInfo);

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int mmc5603Enable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "%s :  MMC5603 enable mag\n", __func__);

    mTask.txBuf[0] = MMC5603NJ_REG_CTRL1;
    mTask.txBuf[1] = MMC5603NJ_CMD_BW01;
    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);
    //  vTaskDelay(1);

    mTask.txBuf[2] = MMC5603NJ_REG_ODR;
    mTask.txBuf[3] = MMC5603NJ_ODR_120HZ;
    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, &mTask.txBuf[2], 2, NULL, NULL);
    //  vTaskDelay(1);

    mTask.txBuf[4] = MMC5603NJ_REG_CTRL0;
    mTask.txBuf[5] = MMC5603NJ_CMD_AUTO_SR_EN;
    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, &mTask.txBuf[4], 2, NULL, NULL);
    // vTaskDelay(1);

    mTask.txBuf[6] = MMC5603NJ_REG_CTRL2;
    mTask.txBuf[7] = MMC5603NJ_CMD_CMM_EN;

    mmc5603_state = AUTO_SET_CONTINU;

    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, &mTask.txBuf[6], 2, i2cCallBack,
                       next_state);
}
static int mmc5603Disable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                          void *inBuf, uint8_t inSize, uint8_t elemInSize,
                          void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "%s :  MMC5603 enable mag\n", __func__);

    mTask.txBuf[6] = MMC5603NJ_REG_CTRL2;
    mTask.txBuf[7] = MMC5603NJ_CMD_CMM_DISEN;
    vTaskDelay(20);

    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, &mTask.txBuf[6], 2, i2cCallBack,
                       next_state);
}
static int mmc5603Rate(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                       void *inBuf, uint8_t inSize, uint8_t elemInSize,
                       void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    struct magCntlPacket cntlPacket;

    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0)
    {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "mmc5603Rate, rx inSize and elemSize error\n");
        return -1;
    }

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int mmc5603GetStatus(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                            void *inBuf, uint8_t inSize, uint8_t elemInSize,
                            void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;

    ret = rxTransferDataInfo(&mTask.dataInfo, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0)
    {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "%s mmc5603Sample, rx dataInfo error\n", __func__);
        return -1;
    }
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int mmc5603ChangState(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                             void *inBuf, uint8_t inSize, uint8_t elemInSize,
                             void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    if (mmc5603_state == AUTO_SET_CONTINU)
    {
        if((fabs(mTask.factoryData.ix)>(double)10000000.0f) || (fabs(mTask.factoryData.iy)>(double)10000000.0f))
        {
            osLog(LOG_ERROR, " mmc5603 external file is more than 10 guass  \n");
            mmc5603_state = MANU_SET_SINGLE;

            mTask.txBuf[0] = MMC5603NJ_REG_CTRL2;
            mTask.txBuf[1] = 0x00;
            i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, (void *)STATE_IDLE);

            mTask.txBuf[0] = MMC5603NJ_REG_CTRL0;
            mTask.txBuf[1] = MMC5603NJ_MANU_SET;
            i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, next_state);
        }
        else
        {
            sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
        }
    }
    else if (mmc5603_state == MANU_SET_SINGLE)
    {
        if((fabs(mTask.factoryData.ix)<(double)8000000.0f) && (fabs(mTask.factoryData.iy)<(double)8000000.0f))
        {
            osLog(LOG_ERROR, " mmc5603 external file is less than  8 guass  \n");
            mmc5603_state = AUTO_SET_CONTINU;

            mTask.txBuf[0] = MMC5603NJ_REG_CTRL0;
            mTask.txBuf[1] = 0xA0;
            i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, (void *)STATE_IDLE);

            mTask.txBuf[0] = MMC5603NJ_REG_CTRL2;
            mTask.txBuf[1] = 0x10;
            i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, next_state);
        }
        else
        {
            if (mTask.sat_reg & MMC5603NJ_SAT_SENSOR)
            {
                // sensor is saturation need to do set
                mTask.txBuf[0] = MMC5603NJ_REG_CTRL0;
                mTask.txBuf[1] = 0x08;
                i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, next_state);
                mTask.sat_reg = 0;
            }
            else
            {
                //TM command
                /* sensor is OK, not saturated */
                mTask.txBuf[0] = MMC5603NJ_REG_CTRL0;
                mTask.txBuf[1] = 0x01;
                i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, next_state);
            }
        }
    }
    else
    {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    }
    return 0;
}

static int mmc5603StEnable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    static int cnt = 0;
    static int NumOfSamples = 250;

    if (mmc5603_state == MANU_SET_SINGLE)
    {
        if ((cnt++) >= NumOfSamples)
        {
            cnt = 0;
            mTask.b_auto_st = true;
            mTask.txBuf[0] = MMC5603NJ_REG_CTRL0;
            mTask.txBuf[1] = 0x40;
            i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, next_state);
        }
        else
        {
            sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
        }
    }
    else
    {
        cnt = 0;
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    }
    return 0;
}

static int mmc5603ReadSaBit(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                            void *inBuf, uint8_t inSize, uint8_t elemInSize,
                            void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{

    if (mTask.b_auto_st)
    {
        /* Read Sat_sensor bit */
        // I2C_Read_Reg(MMC5603NJ_7BITI2C_ADDRESS, MMC5603NJ_REG_STATUS1, &reg_status);
        mTask.txBuf[0] = MMC5603NJ_REG_STATUS1;
        i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1, &mTask.sat_reg, 1, i2cCallBack, next_state);
        mTask.b_auto_st = false;
    }
    else
    {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    }

    return 0;
}

static int mmc5603Sample(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{

    mTask.txBuf[0] = MMC5603_REG_DATA;
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         mTask.rxBuf, 6, i2cCallBack,
                         next_state);
}

static int mmc5603Convert(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                          void *inBuf, uint8_t inSize, uint8_t elemInSize,
                          void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    struct magData *data = mTask.magPacket.outBuf;
    float remap_data[AXES_NUM];
    int32_t idata[AXES_NUM];
    uint64_t timestamp = 0;

    //osLog(LOG_ERROR, "mmc5603Convert direction: %d\n", mTask.hw->direction);

#if 0
    osLog(LOG_ERROR, "%s mmc5603 raw reg data: \n"
                     "[0]:%d, [1]:%d\n"
                     "[2]:%d, [3]:%d\n"
                     "[4]:%d, [5]:%d\n",
          __func__, mTask.rxBuf[0], mTask.rxBuf[1], mTask.rxBuf[2], mTask.rxBuf[3], mTask.rxBuf[4], mTask.rxBuf[5]);
#endif

    idata[AXIS_X] = (int)((uint16_t)((mTask.rxBuf[0] << 8) + (uint16_t)(mTask.rxBuf[1])));
    idata[AXIS_Y] = (int)((uint16_t)((mTask.rxBuf[2] << 8) + (uint16_t)(mTask.rxBuf[3])));
    idata[AXIS_Z] = (int)((uint16_t)((mTask.rxBuf[4] << 8) + (uint16_t)(mTask.rxBuf[5])));

    data[0].x = idata[AXIS_X];
    data[0].y = idata[AXIS_Y];
    data[0].z = idata[AXIS_Z];
    data[0].x -= 32768;
    data[0].y -= 32768;
    data[0].z -= 32768;

    remap_data[mTask.cvt.map[AXIS_X]] = mTask.cvt.sign[AXIS_X] * data[0].x;
    remap_data[mTask.cvt.map[AXIS_Y]] = mTask.cvt.sign[AXIS_Y] * data[0].y;
    remap_data[mTask.cvt.map[AXIS_Z]] = mTask.cvt.sign[AXIS_Z] * data[0].z;
    //printf("%s:: remaped: %f %f %f;;;raw: %f %f %f\n",__func__,(double)remap_data[0],(double)remap_data[1],(double)remap_data[2],(double)data[0].x,(double)data[0].y,(double)data[0].z);
    data[0].x = remap_data[AXIS_X] * 100.0f / 1024;
    data[0].y = remap_data[AXIS_Y] * 100.0f / 1024;
    data[0].z = remap_data[AXIS_Z] * 100.0f / 1024;

    mTask.factoryData.ix = (int32_t)(data[0].x * MAGNETOMETER_INCREASE_NUM_AP * MAGNETOMETER_INCREASE_NUM_AP);
    mTask.factoryData.iy = (int32_t)(data[0].y * MAGNETOMETER_INCREASE_NUM_AP * MAGNETOMETER_INCREASE_NUM_AP);
    mTask.factoryData.iz = (int32_t)(data[0].z * MAGNETOMETER_INCREASE_NUM_AP * MAGNETOMETER_INCREASE_NUM_AP);

    timestamp = addThenRetreiveAverageMagTimeStamp(mTask.hwSampleTime);
    txTransferDataInfo(&mTask.dataInfo, 1, timestamp, data);
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
#if 0
    osLog(LOG_ERROR, "%s factoryData x:%d,y:%d,z:%d\n",
          __func__,mTask.factoryData.ix,mTask.factoryData.iy,mTask.factoryData.iz);
#endif
    return 0;
}

static int mmc5603SampleDone(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                             void *inBuf, uint8_t inSize, uint8_t elemInSize,
                             void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}
static struct sensorFsm mmc5603Fsm[] = {

    sensorFsmCmd(STATE_SAMPLE, STATE_CHIP_SAMPLE, mmc5603GetStatus),
    sensorFsmCmd(STATE_CHIP_SAMPLE, STATE_CONVERT, mmc5603Sample),
    sensorFsmCmd(STATE_CONVERT, STATE_CHIP_MMC_CHANGE, mmc5603Convert),
    sensorFsmCmd(STATE_CHIP_MMC_CHANGE, STATE_CHIP_READ_SA_BIT, mmc5603ChangState),
    sensorFsmCmd(STATE_CHIP_READ_SA_BIT, STATE_CHIP_ST_ENABLE, mmc5603ReadSaBit),
    sensorFsmCmd(STATE_CHIP_ST_ENABLE, STATE_CHIP_SAMPLE_OVER, mmc5603StEnable),
    sensorFsmCmd(STATE_CHIP_SAMPLE_OVER, STATE_SAMPLE_DONE, mmc5603SampleDone),

    sensorFsmCmd(STATE_ENABLE, STATE_ENABLE_DONE, mmc5603Enable),
    sensorFsmCmd(STATE_DISABLE, STATE_DISABLE_DONE, mmc5603Disable),

    sensorFsmCmd(STATE_RATECHG, STATE_RATECHG_DONE, mmc5603Rate),

    sensorFsmCmd(STATE_FUSE_EN, STATE_INIT_DONE, mmc5603RegisterCore),
};

static int mmc5603Init(void)
{
    int ret = 0;

    mmc5603DebugPoint = &mTask;
    insertMagicNum(&mTask.magPacket);
    mTask.hw = get_cust_mag("mmc5603");
    if (NULL == mTask.hw)
    {
        osLog(LOG_ERROR, "mmc5603 get_cust_mag fail\n");
        ret = -1;
        goto err_out;
    }
    mTask.i2c_addr = mTask.hw->i2c_addr[0];
    osLog(LOG_ERROR, "driver version : %s, mag i2c_num: %d, i2c_addr: 0x%x\n", MMC5603_DRIVER_VERSION, mTask.hw->i2c_num, mTask.i2c_addr);

    if (0 != (ret = sensorDriverGetConvert(mTask.hw->direction, &mTask.cvt)))
    {
        osLog(LOG_ERROR, "invalid direction: %d\n", mTask.hw->direction);
    }
    osLog(LOG_ERROR, "mag map[0]:%d, map[1]:%d, map[2]:%d, sign[0]:%d, sign[1]:%d, sign[2]:%d\n\r",
          mTask.cvt.map[AXIS_X], mTask.cvt.map[AXIS_Y], mTask.cvt.map[AXIS_Z],
          mTask.cvt.sign[AXIS_X], mTask.cvt.sign[AXIS_Y], mTask.cvt.sign[AXIS_Z]);

    i2cMasterRequest(mTask.hw->i2c_num, I2C_SPEED);
    mTask.txBuf[0] = MMC5603_REG_PRODUCT;
    ret = i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                            (uint8_t *)mTask.autoDetect, 1, NULL, NULL);
    if (ret < 0)
    {
        osLog(LOG_ERROR, "mmc5603 i2cMasterTxRxSync fail!!!\n");
        ret = -1;
        i2cMasterRelease(mTask.hw->i2c_num);
        goto err_out;
    }

    if (mTask.autoDetect[0] == MMC5603_PRODUCT_ID_VALUE)
    { // mmc5603 sensor id
        osLog(LOG_ERROR, "%s read id:0x%x suceess!!!\n", __func__, mTask.autoDetect[0]);
        osLog(LOG_INFO, "mmc5603: auto detect success\n");
#ifdef VENDOR_EDIT
        sensor_register_devinfo(SENS_TYPE_MAG, MAG_MMC5603);
#endif
        mTask.mag_dev_info.layout = 0x00;
        mTask.mag_dev_info.deviceid = 0x10;
        strncpy(mTask.mag_dev_info.libname, "memsic", sizeof(mTask.mag_dev_info.libname));
        magSensorRegister();
        magRegisterInterruptMode(MAG_UNFIFO);
        registerMagDriverFsm(mmc5603Fsm, ARRAY_SIZE(mmc5603Fsm));
        //  registerMagCaliAPI(&mmc5603CaliAPI);
        registerMagTimerCbk(mmc5603TimerCbkF);
#ifdef CFG_VENDOR_FUSION_SUPPORT
        //registerVendorInterface(&mmc5603Interface);
#endif
    }
    else
    {
        i2cMasterRelease(mTask.hw->i2c_num);
        osLog(LOG_ERROR, "mmc5603 read id fail!!!\n");
        ret = -1;
        goto err_out;
    }
        mTask.b_auto_st = false;
        mTask.sat_reg = 0;

err_out:
    return ret;
}
#ifndef CFG_OVERLAY_INIT_SUPPORT
MODULE_DECLARE(mmc5603, SENS_TYPE_MAG, mmc5603Init);
#else
#include "mtk_overlay_init.h"
OVERLAY_DECLARE(mmc5603, OVERLAY_WORK_01, mmc5603Init);
#endif
