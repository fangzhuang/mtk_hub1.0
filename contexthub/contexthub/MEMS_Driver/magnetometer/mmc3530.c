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

#include <magnetometer.h>
#include <contexthub_core.h>
#include <cust_mag.h>
#include <memsic_wrapper.h>
#include <vendor_fusion.h>
#include "mmc3530.h"
#include "magnetometer.h"


#include "./lib/memsic/MemsicOri.h"
#include "./lib/memsic/MemsicGyr.h"
#include "./lib/memsic/MemsicRov.h"
#include "./lib/memsic/MemsicConfig.h"
#include "./lib/memsic/MemsicNineAxisFusion.h"

//#define MEMSIC_DEBUG 1
#define I2C_SPEED  400000
#define RESET_INTV 150
static int has_Gyro=0;
static int iRestart = 1;

enum MMC3530State {
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
    STATE_CHIP_SAMPLE_OVER,
};

static struct MMC3530Task {
    /* txBuf for i2c operation, fill register and fill value */
    uint8_t txBuf[8];
    /* rxBuf for i2c operation, receive rawdata */
    uint8_t rxBuf[8];
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
} mTask;
static struct MMC3530Task *mmc3530DebugPoint;

static unsigned short otpMatrix[3] = {1000,1000,1350};

void mmc3530TimerCbkF(uint64_t time)
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
    struct TripleAxisDataPoint *tripleSample = (struct TripleAxisDataPoint *)sample;

    tripleSample->ix = mTask.factoryData.ix;
    tripleSample->iy = mTask.factoryData.iy;
    tripleSample->iz = mTask.factoryData.iz;
}

static int mmc3530Refill(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret=-1;
    osLog(LOG_INFO, "MMC3530: %s refill\n",__func__);
    mTask.txBuf[0] = MMC3530_REG_CTRL;
    mTask.txBuf[1] = MMC3530_CTRL_REFILL;
    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack, next_state);
    //vTaskDelay(MMC3530_DELAY_RST);
    return ret;
}
static int mmc3530ChipRest(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret=-1;
    osLog(LOG_INFO, "MMC3530: %s \n",__func__);

    mTask.txBuf[0] = MMC3530_REG_CTRL;
    mTask.txBuf[1] = MMC3530_CTRL_RESET;
    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                      i2cCallBack,next_state);
    return ret;
}
static int mmc3530ChipCtrl1(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret=-1;
    osLog(LOG_INFO, "MMC3530: %s \n",__func__);

    mTask.txBuf[0] = MMC3530_REG_CTRL;
    mTask.txBuf[1] = 0;
    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack,next_state);
    //vTaskDelay(MMC3530_DELAY_RST);
    return ret;
}
static int mmc3530ChipCtrl2(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret=-1;
    osLog(LOG_INFO, "MMC3530: %s \n",__func__);

    mTask.txBuf[0] = MMC3530_REG_CTRL;
    mTask.txBuf[1] = MMC3530_CTRL_REFILL;
    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack,next_state);
    //vTaskDelay(MMC3530_DELAY_SET);
    return ret;
}



static int mmc3530ChipCtrl4(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret=-1;
    osLog(LOG_INFO, "MMC3530: %s \n",__func__);

    mTask.txBuf[0] = MMC3530_REG_CTRL;
    mTask.txBuf[1] = 0;
    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack,next_state);
    //vTaskDelay(1);
    return ret;
}
static int mmc3530ChipBits(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret=-1;
    osLog(LOG_INFO, "MMC3530: %s \n",__func__);

    mTask.txBuf[0] = MMC3530_REG_BITS;
    mTask.txBuf[1] = MMC3530_BITS_SLOW_16;
    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack,next_state);
    //vTaskDelay(MMC3530_DELAY_TM);
    return ret;
}
static int mmc3530ChipTm(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret=-1;
    osLog(LOG_INFO, "MMC3530: %s \n",__func__);

    mTask.txBuf[0] = MMC3530_REG_CTRL;
    mTask.txBuf[1] = MMC3530_CTRL_TM;
    ret = i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                       i2cCallBack,next_state);
    vTaskDelay(MMC3530_DELAY_TM);
    return ret;
}

static int mmc3530ReadOtp(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret=-1;
    osLog(LOG_INFO, "MMC3530: %s \n",__func__);

    mTask.txBuf[0] = MMC3530_REG_OTP;
    ret = i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         mTask.rxBuf, 4, i2cCallBack, next_state);
    return ret;
}
static void mmc3530_convert_otp(uint8_t *data)
{
    signed short stemp = 0;
    unsigned short utemp = 0;

    otpMatrix[0] = 1000;
    stemp = (signed short)(((data[1]&0x03) << 4) | (data[2] >> 4));
    if(stemp >= 32)
        stemp = 32 - stemp;
    otpMatrix[1] = (unsigned short)(stemp * 6 + 1000); //*0.006*1000

    stemp = (signed short)(data[3] & 0x3f);
    if(stemp >= 32)
        stemp = 32 - stemp;
    utemp = (unsigned short)(stemp * 6 + 1000);        //magnify 1000 times
    otpMatrix[2] = utemp + (utemp*3)/10 + (utemp*30%100 + utemp*5)/100;

    osLog(LOG_INFO, " mmc3530 read otp %d %d %d ",otpMatrix[0],otpMatrix[1],otpMatrix[2]);
}
#ifdef VENDOR_EDIT
#define abs(x) (x >= 0 ? (x) : (-x))
static void mmc3530_selftest(int32_t *testResult)
{
    int temp_result = -1;
    unsigned char buf[2]={0,};
    int32_t prev_raw[3] = {0};
    int32_t curr_raw[3] = {0};

    buf[0] = MMC3530_REG_CTRL;
    buf[1] = MMC3530_CTRL_SET;
    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, buf, 2,NULL,NULL);
    vTaskDelay(1);
    osLog(LOG_ERROR, "%s  mmc3530 set\n",__func__);

    buf[0] = MMC3530_REG_CTRL;
    buf[1] = MMC3530_CTRL_TM;
    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, buf, 2, NULL,NULL);
    vTaskDelay(MMC3530_DELAY_TM);

    buf[0] = MMC3530_REG_DATA;
    i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, buf, 1,  mTask.rxBuf, 6, NULL,NULL);

    prev_raw[0] = (int)( (uint16_t)((mTask.rxBuf[AXIS_X*2+1] << 8) + (uint16_t)(mTask.rxBuf[AXIS_X*2])) );
    prev_raw[1] = (int)( (uint16_t)((mTask.rxBuf[AXIS_Y*2+1] << 8) + (uint16_t)(mTask.rxBuf[AXIS_Y*2])) );
    prev_raw[2] = (int)( (uint16_t)((mTask.rxBuf[AXIS_Z*2+1] << 8) + (uint16_t)(mTask.rxBuf[AXIS_Z*2])) );

    osLog(LOG_ERROR, "%s prev_raw: %d %d %d \n",__func__,prev_raw[0],prev_raw[1],prev_raw[2]);
    buf[0] = MMC3530_REG_CTRL;
    buf[1] = MMC3530_CTRL_RESET;
    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, buf, 2,NULL,NULL);
    vTaskDelay(1);
    osLog(LOG_ERROR, "%s  mmc3530 reset\n",__func__);

    buf[0] = MMC3530_REG_CTRL;
    buf[1] = MMC3530_CTRL_TM;
    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, buf, 2, NULL,NULL);
    vTaskDelay(MMC3530_DELAY_TM);

    buf[0] = MMC3530_REG_DATA;
    i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, buf, 1,  mTask.rxBuf, 6, NULL,NULL);

    curr_raw[0] = (int)( (uint16_t)((mTask.rxBuf[AXIS_X*2+1] << 8) + (uint16_t)(mTask.rxBuf[AXIS_X*2])) );
    curr_raw[1] = (int)( (uint16_t)((mTask.rxBuf[AXIS_Y*2+1] << 8) + (uint16_t)(mTask.rxBuf[AXIS_Y*2])) );
    curr_raw[2] = (int)( (uint16_t)((mTask.rxBuf[AXIS_Z*2+1] << 8) + (uint16_t)(mTask.rxBuf[AXIS_Z*2])) );
    osLog(LOG_ERROR, "%s curr_raw: %d %d %d \n",__func__,curr_raw[0],curr_raw[1],curr_raw[2]);

    osLog(LOG_ERROR, "%s diff: %d %d %d \n",__func__,abs(curr_raw[0]-prev_raw[0]),abs(curr_raw[1]-prev_raw[1]),abs(curr_raw[2]-prev_raw[2]));

    if ((abs(curr_raw[0]-prev_raw[0]) > 100) || (abs(curr_raw[1]-prev_raw[1]) > 100)||(abs(curr_raw[2]-prev_raw[2]) > 100))
    {
      temp_result=1;
      osLog(LOG_INFO, "%s : MEMSIC3530_selftest pass \n",__func__);

    }
    else
    {
        temp_result=-1;
       osLog(LOG_INFO, "%s : MEMSIC3530_selftest fail \n",__func__);
    }

     *testResult = (int32_t)temp_result;
}
#endif
static int mmc3530RegisterCore(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    struct sensorCoreInfo mInfo;
    memset(&mInfo, 0, sizeof(struct sensorCoreInfo));

    mmc3530_convert_otp(mTask.rxBuf);

    /* Register sensor Core */
    mInfo.sensType = SENS_TYPE_MAG;
    mInfo.getCalibration = magGetCalibration;
    mInfo.setCalibration = magSetCalibration;
    mInfo.getData = magGetData;
#ifdef VENDOR_EDIT
    //ye.zhang@PSE.BSP.Sensor, 2017-12-20, add for sensor self test
        mInfo.selfTest = mmc3530_selftest;
#endif//VENDOR_EDIT
    sensorCoreRegister(&mInfo);

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int mmc3530Enable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "%s :  MMC3530 enable mag\n",__func__);
    iRestart = 1;
    mTask.txBuf[0] = MMC3530_REG_CTRL;
    mTask.txBuf[1] = MMC3530_CTRL_TM;

    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);
}
static int mmc3530Disable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "%s : MMC3530 disable mag\n",__func__);
    iRestart = 0;

    mTask.txBuf[0] = MMC3530_REG_CTRL;
    mTask.txBuf[1] = MMC3530_CTRL_TM;

    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack,
                       next_state);
}
static int mmc3530Rate(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    struct magCntlPacket cntlPacket;

    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "mmc3530Rate, rx inSize and elemSize error\n");
        return -1;
    }

    magTimeBufferReset(&mTask.magTimeBuf);    //reset time buffer

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}


static int mmc3530GetStatus(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;

    ret = rxTransferDataInfo(&mTask.dataInfo, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "%s mmc3530Sample, rx dataInfo error\n",__func__);
        return -1;
    }
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;

}
static int mmc3530Sample(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    static int read_idx = 0;

    //osLog(LOG_ERROR, "%s Enter\n",__func__);
    mTask.txBuf[0] = MMC3530_REG_DATA;
    i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1, mTask.rxBuf, 6, NULL, NULL);
    read_idx++;

    if(!(read_idx % RESET_INTV)) {
        mTask.txBuf[0] = MMC3530_REG_CTRL;
        mTask.txBuf[1] = MMC3530_CTRL_RESET;
        i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, next_state);
        read_idx = 0;
    } else {
         mTask.txBuf[0] = MMC3530_REG_CTRL;
         mTask.txBuf[1] = MMC3530_CTRL_TM;
         i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, i2cCallBack, next_state);
    }

    return 0;
}

static int mmc3530Convert(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    struct magData *data = mTask.magPacket.outBuf;
    float remap_data[AXES_NUM];
    int32_t idata[AXES_NUM];
    uint64_t timestamp = 0;
    uint8_t data_size = 0;

    idata[AXIS_X] = (int)( (uint16_t)((mTask.rxBuf[AXIS_X*2+1] << 8) + (uint16_t)(mTask.rxBuf[AXIS_X*2])) );
    idata[AXIS_Y] = (int)( (uint16_t)((mTask.rxBuf[AXIS_Y*2+1] << 8) + (uint16_t)(mTask.rxBuf[AXIS_Y*2])) );
    idata[AXIS_Z] = (int)( (uint16_t)((mTask.rxBuf[AXIS_Z*2+1] << 8) + (uint16_t)(mTask.rxBuf[AXIS_Z*2])) );

    data[0].x = idata[AXIS_X];
    data[0].y = idata[AXIS_Y] - idata[AXIS_Z] + 32768;
    data[0].z = idata[AXIS_Y] + idata[AXIS_Z] - 32768;
    data[0].x -= 32768;
    data[0].y -= 32768;
    data[0].z -= 32768;

    data[0].x = data[0].x * otpMatrix[0] / 1000;
    data[0].y = data[0].y * otpMatrix[1] / 1000;
    data[0].z = data[0].z * otpMatrix[2] / 1000;

    remap_data[mTask.cvt.map[AXIS_X]] = (mTask.cvt.sign[AXIS_X] * data[0].x) / 1024;
    remap_data[mTask.cvt.map[AXIS_Y]] = (mTask.cvt.sign[AXIS_Y] * data[0].y) / 1024;
    remap_data[mTask.cvt.map[AXIS_Z]] = (mTask.cvt.sign[AXIS_Z] * data[0].z) / 1024;

    mTask.factoryData.ix = (int32_t)(remap_data[AXIS_X] * MAGNETOMETER_INCREASE_NUM_AP * MAGNETOMETER_INCREASE_NUM_AP);
    mTask.factoryData.iy = (int32_t)(remap_data[AXIS_Y] * MAGNETOMETER_INCREASE_NUM_AP * MAGNETOMETER_INCREASE_NUM_AP);
    mTask.factoryData.iz = (int32_t)(remap_data[AXIS_Z] * MAGNETOMETER_INCREASE_NUM_AP * MAGNETOMETER_INCREASE_NUM_AP);

    data_size = magTimeBufferSize(&mTask.magTimeBuf);
    for (uint8_t i = 0; i < data_size; i++) {
        data[i].x = remap_data[AXIS_X];
        data[i].y = remap_data[AXIS_Y];
        data[i].z = remap_data[AXIS_Z];
        magTimeBufferRead(&mTask.magTimeBuf, &mTask.hwSampleTime);
        timestamp = addThenRetreiveAverageMagTimeStamp(mTask.hwSampleTime);
    }

    txTransferDataInfo(&mTask.dataInfo, data_size, timestamp, data);
    /*osLog(LOG_ERROR, "mmc3530Convert raw data: %f, %f, %f, timestamp: %llu size: %u!\n", (double)remap_data[AXIS_X],
        (double)remap_data[AXIS_Y], (double)remap_data[AXIS_Z], timestamp, data_size); */

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int mmc3530SampleDone(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                          void *inBuf, uint8_t inSize, uint8_t elemInSize,
                          void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}
static struct sensorFsm mmc3530Fsm[] = {

    sensorFsmCmd(STATE_SAMPLE, STATE_CHIP_SAMPLE, mmc3530GetStatus),
    sensorFsmCmd(STATE_CHIP_SAMPLE, STATE_CONVERT, mmc3530Sample),
    sensorFsmCmd(STATE_CONVERT, STATE_CHIP_SAMPLE_OVER, mmc3530Convert),
    sensorFsmCmd(STATE_CHIP_SAMPLE_OVER, STATE_SAMPLE_DONE, mmc3530SampleDone),


    sensorFsmCmd(STATE_ENABLE, STATE_ENABLE_DONE, mmc3530Enable),
    sensorFsmCmd(STATE_DISABLE, STATE_DISABLE_DONE, mmc3530Disable),

    sensorFsmCmd(STATE_RATECHG, STATE_RATECHG_DONE, mmc3530Rate),

    sensorFsmCmd(STATE_FUSE_EN, STATE_CHIP_RESET, mmc3530Refill),
    sensorFsmCmd(STATE_CHIP_RESET, STATE_CHIP_CTRL1, mmc3530ChipRest),
    sensorFsmCmd(STATE_CHIP_CTRL1, STATE_CHIP_CTRL2, mmc3530ChipCtrl1),
    sensorFsmCmd(STATE_CHIP_CTRL2, STATE_CHIP_CTRL4, mmc3530ChipCtrl2),
    sensorFsmCmd(STATE_CHIP_CTRL4, STATE_CHIP_BITS, mmc3530ChipCtrl4),
    sensorFsmCmd(STATE_CHIP_BITS, STATE_CHIP_TM, mmc3530ChipBits),
    sensorFsmCmd(STATE_CHIP_TM, STATE_CHIP_READ_OTP, mmc3530ChipTm),
    sensorFsmCmd(STATE_CHIP_READ_OTP, STATE_CORE, mmc3530ReadOtp),

    sensorFsmCmd(STATE_CORE, STATE_INIT_DONE, mmc3530RegisterCore),
};

static int mmc3530CaliApiGetOffset(float offset[AXES_NUM])
{
    osLog(LOG_ERROR, "%s\n",__func__);

    return 0;
}
static int mmc3530CaliApiSetOffset(float offset[AXES_NUM])
{
    osLog(LOG_ERROR, "%s\n",__func__);
    return 0;
}

static int memsic3530CaliApiSetCaliParam(int32_t caliParameter[6])
{
/*
    osLog(LOG_ERROR, "%s %ld, %ld,%ld,%ld  %ld\n\r", __func__,caliParameter[0],caliParameter[1],
          caliParameter[2],caliParameter[3],caliParameter[4]);
*/
    float caliParameterInput[4]={0,0,0,50000};
    caliParameterInput[0]=(float)caliParameter[0]/1000;
    caliParameterInput[1]=(float)caliParameter[1]/1000;
    caliParameterInput[2]=(float)caliParameter[2]/1000;
    caliParameterInput[3]=(float)caliParameter[3]/1000;
    SetMagpara(caliParameterInput);

    return 0;
}
static int memsic3530CaliApiGetCaliParam(int32_t caliParameter[6])
{
    if (has_Gyro) {
        int Accuracy=0;
        static int LastAccuracy=0;
        float cal_para[6];

        Accuracy=GetAccuracy();
        if (LastAccuracy!=3 && Accuracy==3) {
            GetCalibrationPara(cal_para);
            caliParameter[0]=(int32_t)(cal_para[0]*1000);
            caliParameter[1]=(int32_t)(cal_para[1]*1000);
            caliParameter[2]=(int32_t)(cal_para[2]*1000);
            caliParameter[3]=(int32_t)(cal_para[3]*1000);
            caliParameter[4]=(int32_t)(cal_para[4]*1000);
            caliParameter[5]=(int32_t)(cal_para[5]*1000);
            osLog(LOG_ERROR, "%s %ld, %ld,%ld,%ld,%ld\n\r",
                 __func__,caliParameter[0],caliParameter[1],caliParameter[2],caliParameter[3],caliParameter[4]);
        }
        LastAccuracy = Accuracy;
    } else {
        int Accuracy=0;
        static int LastAccuracy=0;
        float cal_para[7];

        Accuracy=GetMagAccuracy();
        if (LastAccuracy!=3 && Accuracy==3 ){
            GetCalPara_6(cal_para);
            caliParameter[0]=(int32_t)(cal_para[0]*1000);
            caliParameter[1]=(int32_t)(cal_para[1]*1000);
            caliParameter[2]=(int32_t)(cal_para[2]*1000);
            caliParameter[3]=(int32_t)(cal_para[3]*1000);
            caliParameter[4]=(int32_t)(cal_para[4]*1000);
            caliParameter[5]=(int32_t)(cal_para[5]*1000);
            osLog(LOG_ERROR, "%s %ld, %ld,%ld,%ld,%ld\n\r",
                  __func__,caliParameter[0],caliParameter[1],caliParameter[2],caliParameter[3],caliParameter[4]);
        }
        LastAccuracy = Accuracy;
    }
    return 0;
}
float acc_inv[3] = {0.0,0.0,9.8};
float gyro_inv[3] = {0.0,};
float mag_inv[3] = {0.0,};

static int mmc3530CaliApiSetGyroData(struct magCaliDataInPut *inputData)
{
#if 1
    osLog(LOG_ERROR, "%s x:%f, y:%f, z:%f, time:%lld\n\r",
          __func__,(double)inputData->x,(double)inputData->y,(double)inputData->z,inputData->timeStamp);
#endif
    gyro_inv[0] = inputData->x;
    gyro_inv[1] = inputData->y;
    gyro_inv[2] = inputData->z;

    return 1;
}
static uint64_t pre_timestamp=0;
static int mmc3530DoCaliAPI(struct magCaliDataInPut *inputData,
        struct magCaliDataOutPut *outputData)
{
    int err = 0;

    int64_t now_timestamp=0,delt_timestamp=0;

    float mag_offset[3]={0,};
    float calmag_para[6];

    float  ts=0.02f;
    float data_cali[3]={0,0,0};
    int8_t Accuracy = 0;
    float mag[3];
    osLog(LOG_ERROR, "%s::enter\n",__func__);

    if (err < 0) {
        osLog(LOG_ERROR, "mmc_set_mag_data fail\n\r");
        return err;
    } else {
        mag[0] = inputData->x;
        mag[1] = inputData->y;
        mag[2] = inputData->z;

        now_timestamp=inputData->timeStamp;
        delt_timestamp=now_timestamp-pre_timestamp;
        pre_timestamp=now_timestamp;
        ts=(float)delt_timestamp/(1000*1000*1000);//convert to seconds
        if(ts>0.02f)
            ts=0.02f;

#if 0
        osLog(LOG_ERROR, "%s: acc_inv:%f, %f, %f; mag:%f, %f, %f gyro_inv:%f %f %f ts: %f\n", __func__,
        (double)acc_inv[0], (double)acc_inv[1], (double)acc_inv[2],
        (double)mag[0], (double)mag[1], (double)mag[2],(double)gyro_inv[0],(double)gyro_inv[1],(double)gyro_inv[2],(double)ts);
#endif
        mag_inv[0] = mag[0]*100;
        mag_inv[1] = mag[1]*100;
        mag_inv[2] = mag[2]*100;

        if (has_Gyro) {
            MainAlgoProcess(acc_inv, mag_inv, gyro_inv, ts, iRestart, 1);
            GetCalibratedMag(data_cali);
            NineAxisFusion(acc_inv,data_cali,gyro_inv,ts,iRestart);
            iRestart = 0;

            GetCalibrationPara(calmag_para);
            mag_offset[0]=(float)calmag_para[0];
            mag_offset[1]=(float)calmag_para[1];
            mag_offset[2]=(float)calmag_para[2];
            Accuracy = GetMagCalAccuracy();
        } else {
            MainAlgorithmProcess(acc_inv,mag_inv);
            GetCalMag(data_cali);
            CalcMemsicGyro(data_cali, acc_inv, ts,iRestart);
            iRestart = 0;
            CalcMemsicRotVec(data_cali, acc_inv);
            Accuracy = GetMagAccuracy();
            GetOffset(mag_offset);
        }
        //osLog(LOG_ERROR, "%s  mmc3530 recv data_cali: %f, %f, %f, Accuracy: %d\n",__func__,
            //  (double)data_cali[0], (double)data_cali[1], (double)data_cali[2], Accuracy);

        outputData->x = data_cali[AXIS_X];
        outputData->y = data_cali[AXIS_Y];
        outputData->z = data_cali[AXIS_Z];

        outputData->x_bias = (float)mag_offset[AXIS_X];
        outputData->y_bias = (float)mag_offset[AXIS_Y];
        outputData->z_bias = (float)mag_offset[AXIS_Z];
        outputData->status = Accuracy;
#if MEMSIC_DEBUG
        osLog(LOG_ERROR, "%s  mmc3530 outputdata:%f, %f, %f\n",__func__,
               (double)outputData->x, (double)outputData->y, (double)outputData->z);
#endif
    }
    return 0;
}
static struct magCalibrationLibAPI mmc3530CaliAPI = {
    .caliApiGetOffset = mmc3530CaliApiGetOffset,
    .caliApiSetOffset = mmc3530CaliApiSetOffset,
    .caliApiGetCaliParam = memsic3530CaliApiGetCaliParam,
    .caliApiSetCaliParam = memsic3530CaliApiSetCaliParam,
    .caliApiSetGyroData = mmc3530CaliApiSetGyroData,
    .doCaliApi = mmc3530DoCaliAPI
};

#ifdef CFG_VENDOR_FUSION_SUPPORT
static int mmc3530FusionInitLib(int hwGyroSupport)
{
    has_Gyro=hwGyroSupport;
    osLog(LOG_ERROR, "%s: hwgyrosupport:%d\n", __func__, hwGyroSupport);
    return MEMSIC_InitLib(hwGyroSupport);
}
static int mmc3530FusionSetGyro(struct InterfaceDataIn *inData)
{
    return 1;
}
static int mmc3530FusionSetAcc(struct InterfaceDataIn *inData)
{
    osLog(LOG_ERROR, "%s: x:%f, y:%f, z:%f\n", __func__, (double)inData->vec[0], (double)inData->vec[1], (double)inData->vec[2]);
    acc_inv[0] = inData->vec[0];
    acc_inv[1] = inData->vec[1];
    acc_inv[2] = inData->vec[2];
    return MEMSIC_FusionSetAccData(inData->vec[0],
        inData->vec[1], inData->vec[2], inData->timeStamp);
}
static int mmc3530FusionSetMag(struct InterfaceDataIn *inData)
{
    //osLog(LOG_ERROR, "%s: x:%f, y:%f, z:%f\n", __func__, (double)inData->vec[0], (double)inData->vec[1], (double)inData->vec[2]);
    return MEMSIC_FusionSetMagData(inData->vec[0],
        inData->vec[1], inData->vec[2], inData->timeStamp);
}

static int mmc3530FusionGetGravity(struct InterfaceDataOut *outData)
{
    osLog(LOG_ERROR, "%s\n",__func__);

    int ret = 0;
    float vec[3] = {0};
    int16_t accurancy = 0;

    ret = MEMSIC_GetGravity(vec, &accurancy);
    outData->vec[0] = vec[0];
    outData->vec[1] = vec[1];
    outData->vec[2] = vec[2];
    outData->status = accurancy;
    //osLog(LOG_ERROR, "%s: x:%f, y:%f, z:%f\n", __func__, (double)outData->vec[0], (double)outData->vec[1], (double)outData->vec[2]);
    return ret;
}

static int mmc3530FusionGetRotationVector(struct InterfaceDataOut *outData)
{
    int ret = 0;
    float vec[4] = {0};
    int16_t accurancy = 0;

    ret = MEMSIC_GetRotaionVector(vec, &accurancy);
    outData->vec[0] = vec[0];
    outData->vec[1] = vec[1];
    outData->vec[2] = vec[2];
    outData->vec[3] = vec[3];
    outData->status = accurancy;
    //osLog(LOG_ERROR, "%s: x:%f, y:%f, z:%f\n", __func__, (double)outData->vec[0], (double)outData->vec[1], (double)outData->vec[2]);
    return ret;
}

static int mmc3530FusionGetOrientation(struct InterfaceDataOut *outData)
{
    //int ret = 0;
    float vec[3] = {0};
    int16_t accurancy = 0;

    MEMSIC_GetOrientaion(vec, &accurancy);
    outData->vec[0] = vec[0];
    outData->vec[1] = vec[1];
    outData->vec[2] = vec[2];
    outData->status = accurancy;
    //osLog(LOG_ERROR, "%s: x:%f, y:%f, z:%f\n", __func__, (double)outData->vec[0], (double)outData->vec[1], (double)outData->vec[2]);
    return 1;
}

static int mmc3530FusionGetLinearaccel(struct InterfaceDataOut *outData)
{
    int ret = 0;
    float vec[3] = {0};
    int16_t accurancy = 0;

    ret = MEMSIC_GetLinearaccel(vec, &accurancy);
    outData->vec[0] = vec[0];
    outData->vec[1] = vec[1];
    outData->vec[2] = vec[2];
    outData->status = accurancy;
    //osLog(LOG_ERROR, "%s: x:%f, y:%f, z:%f\n", __func__, (double)outData->vec[0], (double)outData->vec[1], (double)outData->vec[2]);
    return ret;
}

static int mmc3530FusionGetGameRotationVector(struct InterfaceDataOut *outData)
{
    int ret = 0;
    float vec[4] = {0};
    int16_t accurancy = 0;

    ret = MEMSIC_GetGameRotaionVector(vec, &accurancy);
    outData->vec[0] = vec[0];
    outData->vec[1] = vec[1];
    outData->vec[2] = vec[2];
    outData->vec[3] = vec[3];
    outData->status = accurancy;
    //osLog(LOG_ERROR, "%s: x:%f, y:%f, z:%f\n", __func__, (double)outData->vec[0], (double)outData->vec[1], (double)outData->vec[2]);
    return ret;
}

static int mmc3530FusionGetGeoMagnetic(struct InterfaceDataOut *outData)
{
    int ret = 0;
    float vec[4] = {0};
    int16_t accurancy = 0;

    ret = MEMSIC_GetGeoMagnetic(vec, &accurancy);
    outData->vec[0] = vec[0];
    outData->vec[1] = vec[1];
    outData->vec[2] = vec[2];
    outData->vec[3] = vec[3];
    outData->status = accurancy;
    //osLog(LOG_ERROR, "%s: x:%f, y:%f, z:%f\n", __func__, (double)outData->vec[0], (double)outData->vec[1], (double)outData->vec[2]);
    return ret;
}

static int mmc3530FusionGetVirtualGyro(struct InterfaceDataOut *outData)
{
    int ret = 0;
    float vec[3] = {0};
    int16_t accurancy = 0;


    ret = MEMSIC_GetVirtualGyro(vec, &accurancy);
    outData->vec[0] = vec[0];
    outData->vec[1] = vec[1];
    outData->vec[2] = vec[2];
    outData->status = accurancy;
    //osLog(LOG_ERROR, "%s: x:%f, y:%f, z:%f\n", __func__, (double)outData->vec[0], (double)outData->vec[1], (double)outData->vec[2]);
    return ret;
}

static struct VendorFusionInterfact_t mmc3530Interface = {
    .name = "mmc3530",
    .initLib = mmc3530FusionInitLib,
    .setGyroData = mmc3530FusionSetGyro,
    .setAccData = mmc3530FusionSetAcc,
    .setMagData = mmc3530FusionSetMag,
    .getGravity = mmc3530FusionGetGravity,
    .getRotationVector = mmc3530FusionGetRotationVector,
    .getOrientation = mmc3530FusionGetOrientation,
    .getLinearaccel = mmc3530FusionGetLinearaccel,
    .getGameRotationVector = mmc3530FusionGetGameRotationVector,
    .getGeoMagnetic = mmc3530FusionGetGeoMagnetic,
    .getVirtualGyro = mmc3530FusionGetVirtualGyro
};
#endif

static int mmc3530Init(void)
{
    int ret = 0;

    mmc3530DebugPoint = &mTask;
    insertMagicNum(&mTask.magPacket);
    mTask.hw = get_cust_mag("mmc3530");
    if (NULL == mTask.hw) {
        osLog(LOG_ERROR, "mmc3530 get_cust_mag fail\n");
        ret = -1;
        goto err_out;
    }
    mTask.i2c_addr = mTask.hw->i2c_addr[0];
    osLog(LOG_ERROR, "mag i2c_num: %d, i2c_addr: 0x%x\n", mTask.hw->i2c_num, mTask.i2c_addr);

    if (0 != (ret = sensorDriverGetConvert(mTask.hw->direction, &mTask.cvt))) {
        osLog(LOG_ERROR, "invalid direction: %d\n", mTask.hw->direction);
    }
    osLog(LOG_ERROR, "mag map[0]:%d, map[1]:%d, map[2]:%d, sign[0]:%d, sign[1]:%d, sign[2]:%d\n\r",
        mTask.cvt.map[AXIS_X], mTask.cvt.map[AXIS_Y], mTask.cvt.map[AXIS_Z],
        mTask.cvt.sign[AXIS_X], mTask.cvt.sign[AXIS_Y], mTask.cvt.sign[AXIS_Z]);


    i2cMasterRequest(mTask.hw->i2c_num, I2C_SPEED);
    mTask.txBuf[0] = MMC3530_REG_PID;
    ret = i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
        (uint8_t *)mTask.autoDetect, 1, NULL, NULL);
    if (ret < 0) {
        osLog(LOG_ERROR, "mmc3530 i2cMasterTxRxSync fail!!!\n");
        ret = -1;
        i2cMasterRelease(mTask.hw->i2c_num);
        goto err_out;
    }

    if ( mTask.autoDetect[0] == MMC3530_DEVICE_ID) {  // mmc3530 sensor id
        osLog(LOG_ERROR, "%s read id:0x%x suceess!!!\n", __func__, mTask.autoDetect[0]);
        osLog(LOG_INFO, "mmc3530: auto detect success\n");
#ifdef VENDOR_EDIT
        mTask.txBuf[1] = MMC3530_REG_CTRL;
        mTask.txBuf[2] = MMC3530_CTRL_RESET;
        i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, &mTask.txBuf[1], 2,NULL,NULL);
        vTaskDelay(1);
        osLog(LOG_ERROR, "%s  mmc3530 reset\n",__func__);
        sensor_register_devinfo(SENS_TYPE_MAG, MAG_MMC3530);
#endif
        magSensorRegister();
        magRegisterInterruptMode(MAG_UNFIFO);
        registerMagDriverFsm(mmc3530Fsm, ARRAY_SIZE(mmc3530Fsm));
        registerMagCaliAPI(&mmc3530CaliAPI);
        registerMagTimerCbk(mmc3530TimerCbkF);
#ifdef CFG_VENDOR_FUSION_SUPPORT
        registerVendorInterface(&mmc3530Interface);
#endif
    } else {
        i2cMasterRelease(mTask.hw->i2c_num);
        osLog(LOG_ERROR, "mmc3530 read id fail!!!\n");
        ret = -1;
        goto err_out;
    }

err_out:
    return ret;
}
#ifndef CFG_OVERLAY_INIT_SUPPORT
MODULE_DECLARE(mmc3530, SENS_TYPE_MAG, mmc3530Init);
#else
#include "mtk_overlay_init.h"
OVERLAY_DECLARE(mmc3530, OVERLAY_ID_MAG, mmc3530Init);
#endif
