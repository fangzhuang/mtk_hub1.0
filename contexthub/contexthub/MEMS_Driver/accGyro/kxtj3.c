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

#include <accGyro.h>
#include <contexthub_core.h>
#include <cust_accGyro.h>
#include <mt_gpt.h>
#include <API_sensor_calibration.h>
#define I2C_SPEED                       400000

/* KXTJ3 Register Map  (Please refer to KXTJ3 Specifications) */
#include "kxtj3_registers.h"

enum KXTJ3State {
    STATE_SAMPLE = CHIP_SAMPLING,
    STATE_CONVERT = CHIP_CONVERT,
    STATE_SAMPLE_DONE = CHIP_SAMPLING_DONE,
    STATE_ACC_ENABLE = CHIP_ACC_ENABLE,
    STATE_ACC_ENABLE_DONE = CHIP_ACC_ENABLE_DONE,
    STATE_ACC_DISABLE = CHIP_ACC_DISABLE,
    STATE_ACC_DISABLE_DONE = CHIP_ACC_DISABLE_DONE,
    STATE_ACC_RATECHG = CHIP_ACC_RATECHG,
    STATE_ACC_RATECHG_DONE = CHIP_ACC_RATECHG_DONE,
    STATE_ACC_CALI = CHIP_ACC_CALI,
    STATE_ACC_CALI_DONE = CHIP_ACC_CALI_DONE,
    STATE_ACC_CFG = CHIP_ACC_CFG,
    STATE_ACC_CFG_DONE = CHIP_ACC_CFG_DONE,
    STATE_INIT_DONE = CHIP_INIT_DONE,
    STATE_IDLE = CHIP_IDLE,
    STATE_RESET_R = CHIP_RESET,
    STATE_RESET_W,
    STATE_DEVID,
    STATE_ENPOWER_W,
    STATE_ACC_TURNOFF_W,
    STATE_CALC_RESO,
    STATE_CORE,
    STATE_ACC_RATE,
};

struct scale_factor {
    unsigned char  whole;
    unsigned char  fraction;
};

struct acc_data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};

static struct acc_data_resolution kxtj3g_data_resolution[] = {
    {{ 0, 9}, 1024},
};

#define MAX_I2C_PER_PACKET  8
#define KXTJ3_DATA_LEN   6
#define MAX_RXBUF 512
#define MAX_TXBUF (MAX_RXBUF / MAX_I2C_PER_PACKET)
static struct KXTJ3Task {
    bool accPowerOn;
    /* txBuf for i2c operation, fill register and fill value */
    uint8_t txBuf[MAX_TXBUF];
    /* rxBuf for i2c operation, receive rawdata */
    uint8_t rxBuf[MAX_RXBUF];
    uint8_t deviceId;
    uint8_t waterMark;
    uint32_t accRate;
    uint16_t accRateDiv;

    uint64_t sampleTime;
    struct transferDataInfo dataInfo;
    struct accGyroDataPacket accGyroPacket;
    /* data for factory */
    struct TripleAxisDataPoint accFactoryData;

    int32_t accSwCali[AXES_NUM];
    struct acc_data_resolution *accReso;
    struct accGyro_hw *hw;
    struct sensorDriverConvert cvt;
    uint8_t i2c_addr;
    bool autoDetectDone;
    bool startCali;
    float staticCali[AXES_NUM];
    int32_t accuracy;
    int32_t debug_trace;
} mTask;
static struct KXTJ3Task *kxtj3DebugPoint;

static int kxtj3ResetRead(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "kxtj3ResetRead\n");
    mTask.txBuf[0] = KXTJ3_CTRL_REG2;
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         mTask.rxBuf, 1, i2cCallBack,
                         next_state);
}

static int kxtj3ResetWrite(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "read 0x%x before kxtj3ResetWrite\n", mTask.rxBuf[0]);

    mTask.txBuf[0] = KXTJ3_CTRL_REG2;
    mTask.txBuf[1] = mTask.rxBuf[0] | KXTJ3_CTRL_REG2_SRST;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);
}

static int kxtj3DeviceId(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    /* Wait sensor reset to be ready  */
    mdelay(10);
    osLog(LOG_INFO, "kxtj3DeviceId\n");

    mTask.txBuf[0] = KXTJ3_WHO_AM_I ;
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         &mTask.deviceId, 1, i2cCallBack,
                         next_state);
}
static int kxtj3PowerRead(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "kxtj3PowerRead\n");
    mTask.txBuf[0] = KXTJ3_CTRL_REG1;
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         mTask.rxBuf, 1, i2cCallBack,
                         next_state);
}
static int kxtj3PowerEnableWrite(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "kxtj3PowerEnableWrite: 0x%x\n", mTask.rxBuf[0]);
    mTask.txBuf[0] = KXTJ3_CTRL_REG1;
    mTask.txBuf[1] = mTask.rxBuf[0] | (KXTJ3_CTRL_REG1_PC | KXTJ3_CTRL_REG1_RES);
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);
}
static int kxtj3PowerDisableWrite(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "kxtj3PowerDisableWrite: 0x%x\n", mTask.rxBuf[0]);
    if (mTask.accPowerOn) {
        osLog(LOG_INFO, "kxtj3PowerDisableWrite should not disable, acc:%d\n",
            mTask.accPowerOn);
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    } else {
        mTask.txBuf[0] = KXTJ3_CTRL_REG1;
        mTask.txBuf[1] = mTask.rxBuf[0] & ~KXTJ3_CTRL_REG1_PC;
        return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                           next_state);
    }
    return 0;
}

static int kxtj3gEnable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "kxtj3gEnable\n");

    mTask.accPowerOn = true;
    return kxtj3PowerRead(i2cCallBack, spiCallBack, next_state,
        inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
}
static int kxtj3gDisable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    struct accGyroCntlPacket cntlPacket;

    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        osLog(LOG_INFO, "kxtj3gDisable, rx water_mark err\n");
        return -1;
    }
  
    mTask.accPowerOn = false;
    mTask.accRate = 0;
    return kxtj3PowerRead(i2cCallBack, spiCallBack, next_state,
        inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
}
static int kxtj3gRate(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    uint32_t sample_rate, water_mark;
    uint8_t odr_reg;
    struct accGyroCntlPacket cntlPacket;

    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "kxtj3gRate, rx inSize and elemSize error\n");
        return -1;
    }
    sample_rate = cntlPacket.rate;
    water_mark = cntlPacket.waterMark;

    if (sample_rate > SENSOR_HZ(200)) {
        mTask.accRate = SENSOR_HZ(400);
        odr_reg = KXTJ3_DATA_CTRL_REG_OSA_400;
    }
    else if (sample_rate > SENSOR_HZ(100)) {
        mTask.accRate = SENSOR_HZ(200);
        odr_reg = KXTJ3_DATA_CTRL_REG_OSA_200;
    }
    else if (sample_rate > SENSOR_HZ(50)) {
        mTask.accRate = SENSOR_HZ(100);
        odr_reg = KXTJ3_DATA_CTRL_REG_OSA_100;
    }
    else if (sample_rate > SENSOR_HZ(25)) {
        mTask.accRate = SENSOR_HZ(50);
        odr_reg = KXTJ3_DATA_CTRL_REG_OSA_50;
    }
    else if (sample_rate > SENSOR_HZ(25.0f/2.0f)) {
        mTask.accRate = SENSOR_HZ(25);
        odr_reg = KXTJ3_DATA_CTRL_REG_OSA_25;
    }
    else if (sample_rate > SENSOR_HZ(25.0f/4.0f)) {
        mTask.accRate = SENSOR_HZ(25.0f/2.0f);
        odr_reg = KXTJ3_DATA_CTRL_REG_OSA_12P5;
    }
    else if (sample_rate > SENSOR_HZ(25.0f/8.0f)) {
        mTask.accRate = SENSOR_HZ(25.0f/4.0f);
        odr_reg = KXTJ3_DATA_CTRL_REG_OSA_6P25;
    }
    else if (sample_rate > SENSOR_HZ(25.0f/16.0f)) {
        mTask.accRate = SENSOR_HZ(25.0f/8.0f);
        odr_reg = KXTJ3_DATA_CTRL_REG_OSA_3P125;
    }
    else if (sample_rate > SENSOR_HZ(25.0f/32.0f)) {
        mTask.accRate = SENSOR_HZ(25.0f/16.0f);
        odr_reg = KXTJ3_DATA_CTRL_REG_OSA_1P563;
    }
    else {
        mTask.accRate = SENSOR_HZ(25.0f/32.0f);
        odr_reg = KXTJ3_DATA_CTRL_REG_OSA_0P781;
    }

    mTask.waterMark = water_mark;
    osLog(LOG_INFO, "kxtj3gRate: mTask.accRate:%d, odr_reg:0x%x\n", mTask.accRate, odr_reg);

    registerAccGyroFifoInfo((mTask.accRate == 0) ? 0 : 1024000000000 / mTask.accRate, 0);

    mTask.txBuf[0] = KXTJ3_DATA_CTRL_REG;
    mTask.txBuf[1] = odr_reg;
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);
}
static int kxtj3CalcReso(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    uint8_t reso = 0;
    //osLog(LOG_INFO, "kxtj3CalcReso:acc(0x%x), gyro(0x%x)\n", mTask.rxBuf[0], mTask.rxBuf[1]);
    reso = 0x00;
    // value from sensor
    if (reso < sizeof(kxtj3g_data_resolution) / sizeof(kxtj3g_data_resolution[0])) {
        mTask.accReso = &kxtj3g_data_resolution[reso];
        osLog(LOG_INFO, "acc reso: %d, sensitivity: %d\n", reso, mTask.accReso->sensitivity);
    }
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}
static void accGetCalibration(int32_t *cali, int32_t size)
{
    cali[AXIS_X] = mTask.accSwCali[AXIS_X];
    cali[AXIS_Y] = mTask.accSwCali[AXIS_Y];
    cali[AXIS_Z] = mTask.accSwCali[AXIS_Z];
    osLog(LOG_INFO, "accGetCalibration cali x:%d, y:%d, z:%d\n", cali[AXIS_X], cali[AXIS_Y], cali[AXIS_Z]);
}
static void accSetCalibration(int32_t *cali, int32_t size)
{
    mTask.accSwCali[AXIS_X] = cali[AXIS_X];
    mTask.accSwCali[AXIS_Y] = cali[AXIS_Y];
    mTask.accSwCali[AXIS_Z] = cali[AXIS_Z];
    osLog(LOG_INFO, "accSetCalibration cali x:%d, y:%d, z:%d\n", mTask.accSwCali[AXIS_X],
        mTask.accSwCali[AXIS_Y], mTask.accSwCali[AXIS_Z]);
}
static void accGetData(void *sample)
{
    struct TripleAxisDataPoint *tripleSample = (struct TripleAxisDataPoint *)sample;
    tripleSample->ix = mTask.accFactoryData.ix;
    tripleSample->iy = mTask.accFactoryData.iy;
    tripleSample->iz = mTask.accFactoryData.iz;
}

static void kxtj3SetDebugTrace(int32_t trace) {
    mTask.debug_trace = trace;
    osLog(LOG_INFO, "%s ==> trace:%d\n", __func__, mTask.debug_trace);
}

static void kxtj3GetSensorInfo(struct sensorInfo_t *data)
{
    strncpy(data->name, "kxtj3", sizeof(data->name));
}

static int kxtj3RegisterCore(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    struct sensorCoreInfo mInfo;

    osLog(LOG_INFO, "kxtj3RegisterCore deviceId 0x%x\n\r", mTask.deviceId);

    if (mTask.deviceId != KXTJ3_WHO_AM_I_WIA_ID) {
        osLog(LOG_INFO, "kxtj3RegisterCore deviceId 0x%x\n\r", mTask.deviceId);
        return 0;
    }

    /* Register sensor Core */
    mInfo.sensType = SENS_TYPE_ACCEL;
    mInfo.gain = GRAVITY_EARTH_1000;
    mInfo.sensitivity = mTask.accReso->sensitivity;
    mInfo.cvt = mTask.cvt;
    mInfo.getCalibration = accGetCalibration;
    mInfo.setCalibration = accSetCalibration;
    mInfo.getData = accGetData;
    mInfo.setDebugTrace = kxtj3SetDebugTrace;
    mInfo.getSensorInfo = kxtj3GetSensorInfo;
    sensorCoreRegister(&mInfo);

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}
static int kxtj3Sample(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    osLog(LOG_INFO, "kxtj3Sample\n\r");

    ret = rxTransferDataInfo(&mTask.dataInfo, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "kxtj3Sample, rx dataInfo error\n");
        return -1;
    }

    mTask.sampleTime = rtcGetTime();

    mTask.txBuf[0] = KXTJ3_XOUT_L;
    ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, &mTask.txBuf[0], 1,
                         &mTask.rxBuf[0], 6, i2cCallBack,
                         next_state);
    if (ret != 0)
        osLog(LOG_INFO, "KXTJ3_XOUT_L err\n");

    return 0;
}
static int kxtj3gConvert(uint8_t *databuf, struct accGyroData *data)
{
    int32_t raw_data[AXES_NUM];
    int32_t remap_data[AXES_NUM];

    float temp_data[AXES_NUM] = {0};
    int32_t caliResult[AXES_NUM] = {0};
    float calibrated_data_output[AXES_NUM] = {0};
    int32_t delta_time = 0;
    int16_t status = 0;
    // TODO: update shift based on 14/12bit data, 2/4
    uint8_t right_shift = 4;
    osLog(LOG_INFO, "kxtj3gConvert\n\r");

    raw_data[AXIS_X] = (int16_t)((databuf[AXIS_X * 2 + 1] << 8) | databuf[AXIS_X * 2]);
    raw_data[AXIS_Y] = (int16_t)((databuf[AXIS_Y * 2 + 1] << 8) | databuf[AXIS_Y * 2]);
    raw_data[AXIS_Z] = (int16_t)((databuf[AXIS_Z * 2 + 1] << 8) | databuf[AXIS_Z * 2]);
    osLog(LOG_INFO, "acc rawdata x:%d, y:%d, z:%d\n", raw_data[AXIS_X],
        raw_data[AXIS_Y], raw_data[AXIS_Z]);

    /* shift data to correct format */
    raw_data[AXIS_X] = raw_data[AXIS_X] >> right_shift;
    raw_data[AXIS_Y] = raw_data[AXIS_Y] >> right_shift;
    raw_data[AXIS_Z] = raw_data[AXIS_Z] >> right_shift;

    raw_data[AXIS_X] = raw_data[AXIS_X] + mTask.accSwCali[AXIS_X];
    raw_data[AXIS_Y] = raw_data[AXIS_Y] + mTask.accSwCali[AXIS_Y];
    raw_data[AXIS_Z] = raw_data[AXIS_Z] + mTask.accSwCali[AXIS_Z];

    /* remap coordinate */
    remap_data[mTask.cvt.map[AXIS_X]] = mTask.cvt.sign[AXIS_X] * raw_data[AXIS_X];
    remap_data[mTask.cvt.map[AXIS_Y]] = mTask.cvt.sign[AXIS_Y] * raw_data[AXIS_Y];
    remap_data[mTask.cvt.map[AXIS_Z]] = mTask.cvt.sign[AXIS_Z] * raw_data[AXIS_Z];

    /* data to m/s2*/
    temp_data[AXIS_X] = (float)remap_data[AXIS_X] * GRAVITY_EARTH_SCALAR / mTask.accReso->sensitivity;
    temp_data[AXIS_Y] = (float)remap_data[AXIS_Y] * GRAVITY_EARTH_SCALAR / mTask.accReso->sensitivity;
    temp_data[AXIS_Z] = (float)remap_data[AXIS_Z] * GRAVITY_EARTH_SCALAR / mTask.accReso->sensitivity;

    /* calibrate loop */
    if (UNLIKELY(mTask.startCali)) {
#ifdef MT6799_EVB
        delta_time++;
        status = 0;
        mTask.accuracy = 3;
        /* mt6799 test data */
        calibrated_data_output[AXIS_X] = temp_data[AXIS_X] - 0.5;
        calibrated_data_output[AXIS_Y] = temp_data[AXIS_Y] - 0.0;
        calibrated_data_output[AXIS_Z] = temp_data[AXIS_Z] + 0.5;
#else
        status = Acc_run_factory_calibration_timeout(delta_time,
            temp_data, calibrated_data_output, (int *)&mTask.accuracy, rtcGetTime());
#endif
        if (status != 0) {
            mTask.startCali = false;
            if (status > 0) {
                osLog(LOG_INFO, "ACC cali detect shake\n");
                caliResult[AXIS_X] = (int32_t)(mTask.staticCali[AXIS_X] * 1000);
                caliResult[AXIS_Y] = (int32_t)(mTask.staticCali[AXIS_Y] * 1000);
                caliResult[AXIS_Z] = (int32_t)(mTask.staticCali[AXIS_Z] * 1000);
		accGyroSendCalibrationResult(SENS_TYPE_ACCEL, (int32_t *)&caliResult[0], (uint8_t)status);
            } else
                osLog(LOG_INFO, "ACC cali time out\n");
        } else if (mTask.accuracy == 3) {
            mTask.startCali = false;
            mTask.staticCali[AXIS_X] = calibrated_data_output[AXIS_X] - temp_data[AXIS_X];
            mTask.staticCali[AXIS_Y] = calibrated_data_output[AXIS_Y] - temp_data[AXIS_Y];
            mTask.staticCali[AXIS_Z] = calibrated_data_output[AXIS_Z] - temp_data[AXIS_Z];
            caliResult[AXIS_X] = (int32_t)(mTask.staticCali[AXIS_X] * 1000);
            caliResult[AXIS_Y] = (int32_t)(mTask.staticCali[AXIS_Y] * 1000);
            caliResult[AXIS_Z] = (int32_t)(mTask.staticCali[AXIS_Z] * 1000);
	    accGyroSendCalibrationResult(SENS_TYPE_ACCEL, (int32_t *)&caliResult[0], (uint8_t)status);
            osLog(LOG_INFO, "ACC cali done:caliResult[0]:%d, caliResult[1]:%d, caliResult[2]:%d, offset[0]:%f, offset[1]:%f, offset[2]:%f\n",
                caliResult[AXIS_X], caliResult[AXIS_Y], caliResult[AXIS_Z],
                (double)mTask.staticCali[AXIS_X],
                (double)mTask.staticCali[AXIS_Y],
                (double)mTask.staticCali[AXIS_Z]);
        }
    }

    data->sensType = SENS_TYPE_ACCEL;

    data->x = temp_data[AXIS_X] + mTask.staticCali[AXIS_X];
    data->y = temp_data[AXIS_Y] + mTask.staticCali[AXIS_Y];
    data->z = temp_data[AXIS_Z] + mTask.staticCali[AXIS_Z];

    mTask.accFactoryData.ix = (int32_t)(data->x * ACCELEROMETER_INCREASE_NUM_AP);
    mTask.accFactoryData.iy = (int32_t)(data->y * ACCELEROMETER_INCREASE_NUM_AP);
    mTask.accFactoryData.iz = (int32_t)(data->z * ACCELEROMETER_INCREASE_NUM_AP);

    osLog(LOG_INFO, "acc data x:%f, y:%f, z:%f\n", 
        (double)data->x, (double)data->y, (double)data->z);

    return 0;
}
static int kxtj3Convert(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    struct accGyroData *data = mTask.accGyroPacket.outBuf;
    uint8_t accEventSize = 0;
    uint8_t *databuf;

    osLog(LOG_INFO, "kxtj3Convert");
    databuf = &mTask.rxBuf[0];
    accEventSize++;
    kxtj3gConvert(databuf, &data[0]);

    /*if startcali true , can't send to runtime cali in parseRawData to accGyro*/
    if (mTask.startCali) {
        accEventSize = 0;
    }

    txTransferDataInfo(&mTask.dataInfo, accEventSize, 0, mTask.sampleTime, data, 0);

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int kxtj3AccCali(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                            void *inBuf, uint8_t inSize, uint8_t elemInSize,
                            void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
#ifdef MT6799_EVB
    MPE_SENSOR_DATA bias;
#else
    float bias[AXES_NUM] = {0};
#endif

    mTask.startCali = true;

    osLog(LOG_INFO, "kxtj3AccCali\n");

    Acc_init_calibration(bias);

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}
static int kxtj3AccCfgCali(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                            void *inBuf, uint8_t inSize, uint8_t elemInSize,
                            void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "kxtj3AccCfgCali\n");

    int ret = 0;
    struct accGyroCaliCfgPacket caliCfgPacket;

    ret = rxCaliCfgInfo(&caliCfgPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);

    if (ret < 0) {
       sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state,  ERROR_EVT);
       osLog(LOG_ERROR, "kxtj3AccCfgCali, rx inSize and elemSize error\n");
       return -1;
    }
    osLog(LOG_INFO, "kxtj3AccCfgCali: cfgData[0]:%d, cfgData[1]:%d, cfgData[2]:%d\n",
       caliCfgPacket.caliCfgData[0], caliCfgPacket.caliCfgData[1], caliCfgPacket.caliCfgData[2]);

    mTask.staticCali[0] = (float)caliCfgPacket.caliCfgData[0] / 1000;
    mTask.staticCali[1] = (float)caliCfgPacket.caliCfgData[1] / 1000;
    mTask.staticCali[2] = (float)caliCfgPacket.caliCfgData[2] / 1000;

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static struct sensorFsm kxtj3Fsm[] = {
    sensorFsmCmd(STATE_SAMPLE, STATE_CONVERT, kxtj3Sample),
    sensorFsmCmd(STATE_CONVERT, STATE_SAMPLE_DONE, kxtj3Convert),
    /* acc enable state */
    sensorFsmCmd(STATE_ACC_ENABLE, STATE_ENPOWER_W, kxtj3gEnable),
    sensorFsmCmd(STATE_ENPOWER_W, STATE_ACC_ENABLE_DONE, kxtj3PowerEnableWrite),
    /* acc disable state */
    sensorFsmCmd(STATE_ACC_DISABLE, STATE_ACC_TURNOFF_W, kxtj3gDisable),
    sensorFsmCmd(STATE_ACC_TURNOFF_W, STATE_ACC_DISABLE_DONE, kxtj3PowerDisableWrite),
    /* acc rate state */
    sensorFsmCmd(STATE_ACC_RATECHG, STATE_ACC_RATECHG_DONE, kxtj3gRate),
    /* cali state */
    sensorFsmCmd(STATE_ACC_CALI, STATE_ACC_CALI_DONE, kxtj3AccCali),
    /* cfg state */
    sensorFsmCmd(STATE_ACC_CFG, STATE_ACC_CFG_DONE, kxtj3AccCfgCali),
    /* init state */
    sensorFsmCmd(STATE_RESET_R, STATE_RESET_W, kxtj3ResetRead),
    sensorFsmCmd(STATE_RESET_W, STATE_DEVID, kxtj3ResetWrite),
    sensorFsmCmd(STATE_DEVID, STATE_CALC_RESO, kxtj3DeviceId),
    sensorFsmCmd(STATE_CALC_RESO, STATE_CORE, kxtj3CalcReso),
    sensorFsmCmd(STATE_CORE, STATE_INIT_DONE, kxtj3RegisterCore),
};
void kxtj3TimerCbkF(void)
{
    //osLog(LOG_INFO, "kxtj3TimerCbkF\n");
}
static void i2cAutoDetect(void *cookie, size_t tx, size_t rx, int err)
{
    if (err == 0)
        osLog(LOG_INFO, "kxtj3: auto detect success:0x%x\n", mTask.deviceId);
    else
        osLog(LOG_ERROR, "kxtj3: auto detect error (%d)\n", err);

    mTask.autoDetectDone = true;
}
int kxtj3Init(void)
{
    int ret = 0;
    osLog(LOG_INFO, "kxtj3: kxtj3Init\n");

    kxtj3DebugPoint = &mTask;
    insertMagicNum(&mTask.accGyroPacket);
    mTask.hw = get_cust_accGyro("kxtj3");
    if (NULL == mTask.hw) {
        osLog(LOG_ERROR, "get_cust_acc_hw fail\n");
        return 0;
    }
    mTask.i2c_addr = mTask.hw->i2c_addr[0];
    osLog(LOG_ERROR, "acc i2c_num: %d, i2c_addr: 0x%x\n", mTask.hw->i2c_num, mTask.i2c_addr);

    if (0 != (ret = sensorDriverGetConvert(mTask.hw->direction, &mTask.cvt))) {
        osLog(LOG_ERROR, "invalid direction: %d\n", mTask.hw->direction);
    }
    osLog(LOG_ERROR, "acc map[0]:%d, map[1]:%d, map[2]:%d, sign[0]:%d, sign[1]:%d, sign[2]:%d\n\r",
        mTask.cvt.map[AXIS_X], mTask.cvt.map[AXIS_Y], mTask.cvt.map[AXIS_Z],
        mTask.cvt.sign[AXIS_X], mTask.cvt.sign[AXIS_Y], mTask.cvt.sign[AXIS_Z]);
    i2cMasterRequest(mTask.hw->i2c_num, I2C_SPEED);

    mTask.autoDetectDone = false;
    mTask.txBuf[0] = KXTJ3_WHO_AM_I ;
    ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                        &mTask.deviceId, 1, i2cAutoDetect, NULL);
    if (ret != 0) {
        osLog(LOG_ERROR, "kxtj3: auto detect i2cMasterTxRx error (%d)\n", ret);
        ret = -1;
        goto err_out;
    }

    /* wait i2c rxdata */
    if(!mTask.autoDetectDone)
        mdelay(1);
    if(!mTask.autoDetectDone)
        mdelay(3);

    if (mTask.deviceId != KXTJ3_WHO_AM_I_WIA_ID) {
        osLog(LOG_ERROR, "kxtj3: auto detect error wai 0x%x\n", mTask.deviceId);
        ret = -1;
        goto err_out;
    }    

    osLog(LOG_INFO, "kxtj3: auto detect success wai 0x%x\n", mTask.deviceId);
    accSensorRegister();
    registerAccGyroInterruptMode(ACC_GYRO_FIFO_UNINTERRUPTIBLE);
    registerAccGyroTimerCbk(kxtj3TimerCbkF);
    registerAccGyroDriverFsm(kxtj3Fsm, ARRAY_SIZE(kxtj3Fsm));

err_out:
    return ret;

}
#ifndef CFG_OVERLAY_INIT_SUPPORT
MODULE_DECLARE(kxtj3, SENS_TYPE_ACCEL, kxtj3Init);
#else
#include "mtk_overlay_init.h"
OVERLAY_DECLARE(kxtj3, OVERLAY_WORK_00, kxtj3Init);
#endif

