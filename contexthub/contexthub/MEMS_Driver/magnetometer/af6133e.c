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
 
 /*
  * Copyright (C) 2013 VTC Technology Inc.
  * 
  * Vendor Name: Voltafield
  * Author Name: George Tseng <george.tseng@voltafield.com>
  *
  * Driver Name: MTK Sensorhub MSensor AF6133E Driver Code
  * Driver Version: 2.0
  * Release Date: 2019.04.29
  *
  * Modify History:
  *   v1.0: Initial release.
  *   v1.1: Add temperature compensation.
  *   v2.0: multi-BISC, add debug trace.
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
#include "af6133e.h"
#include <math.h>
#include <timer.h>

#include "cache_internal.h"

#ifndef CFG_MAG_CALIBRATION_IN_AP
#ifdef VTC_HUBALGOLIB_READY
#include "vtc_library_interface.h"
#endif
#endif

/*------------------------------------------------------------------------------------------------*/
enum AF6133EState {
  STATE_SAMPLE = CHIP_SAMPLING,
  STATE_CONVERT = CHIP_CONVERT,
  STATE_SAMPLE_DONE = CHIP_SAMPLING_DONE,
  STATE_ENABLE = CHIP_ENABLE,
  STATE_ENABLE_DONE = CHIP_ENABLE_DONE,
  STATE_DISABLE = CHIP_DISABLE,
  STATE_DISABLE_DONE = CHIP_DISABLE_DONE,
  STATE_RATECHG = CHIP_RATECHG,
  STATE_RATECHG_DONE = CHIP_RATECHG_DONE,
#ifdef AF6133E_SELF_TEST
  STATE_SELFTEST = CHIP_SELFTEST,
  STATE_SELFTEST_DONE = CHIP_SELFTEST_DONE,
#endif
  STATE_INIT_DONE = CHIP_INIT_DONE,
  STATE_IDLE = CHIP_IDLE,
  STATE_SENSOR_INIT_MEAS_CONG1 = CHIP_RESET,
  
  STATE_SENSOR_INIT_MEAS_CONG2,
  STATE_SENSOR_INIT_OFFSET,
  STATE_SENSOR_INIT_TEMP,
  STATE_SENSOR_BIST_INIT,
  STATE_SENSOR_BIST_XPOS,
  STATE_SENSOR_BIST_XNEG,
  STATE_SENSOR_BIST_YPOS,
  STATE_SENSOR_BIST_YNEG,
  STATE_SENSOR_BIST_ZPOS,
  STATE_SENSOR_BIST_ZNEG,
  STATE_SENSOR_BIST_END,
  STATE_CORE,
#ifdef AF6133E_SELF_TEST
  STATE_SELFTEST_FIELD,
  STATE_SELFTEST_END,
#endif
  STATE_READTEMP,
};
/*------------------------------------------------------------------------------------------------*/
SRAM_REGION_BSS static struct AF6133ETask {
  uint8_t txBuf[2];
  uint8_t rxBuf[6];
  
  uint8_t control_8m_disable;
  uint8_t bist_count;
  uint8_t bist_status;
  uint8_t test2_idx;
  uint8_t test2_reg[6];
#ifdef AF6133E_SET_OFFSET
  float offset[3];
#endif
  float bist_x[3];
  float bist_y[3];
  float bist_z[3];
  float bist_pos[3];
  float bist_neg[3];
  float gain[3];
  float comp_coeff[4];
  int16_t bist_pass_count_p;
  int16_t bist_pass_count_n;
  uint32_t odr;
  
  uint32_t timerHandle;
  I2cCallbackF i2cCallBack;
  void *next_state;
  uint8_t read_reg;
  uint8_t deviceId;

  uint64_t hwSampleTime;
  struct transferDataInfo dataInfo;
  struct magDataPacket magPacket;
  /* data for factory */
  struct TripleAxisDataPoint factoryData;
  struct mag_hw *hw;
  struct sensorDriverConvert cvt;
  uint8_t i2c_addr;
  struct mag_dev_info_t mag_dev_info;
#ifdef AF6133E_LC_FILTER
  uint8_t fil_init;
  uint16_t fil_count;
  float fil_buf[3];
#endif
  int32_t control;
#ifdef AF6133E_TEMP_COMP
  int16_t t0;
  int16_t dt;
  int16_t dt_pre;
  uint16_t t_count;
  uint16_t t_index;
  int16_t t_buf[TEMP_MF_NUM];
#endif
#ifdef AF6133E_SELF_TEST
  float normal_field[3];
  float self_bist_x[3];
  float self_bist_y[3];
  float self_bist_z[3];
#endif
} mTask;
/*------------------------------------------------------------------------------------------------*/
static void ReadDataDelayCallback(uint32_t timerId, void *data)
{
  mTask.timerHandle = 0;

  if(mTask.control_8m_disable)
  {
    mTask.txBuf[0] = REG_CHOPPER;  
    mTask.txBuf[1] = 0xC1; // close 8M
    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);
  }

  /* do the delay i2c transfer */
  mTask.txBuf[0] = mTask.read_reg;
  i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr,
                mTask.txBuf, 1,
                mTask.rxBuf, ((mTask.read_reg == REG_PCODE) ? 1 : 6),
                mTask.i2cCallBack , mTask.next_state);
}
/*------------------------------------------------------------------------------------------------*/
static void SetReadDataTimer(I2cCallbackF i2cCallBack, void *next_state, uint64_t delay, uint8_t read_reg)
{
  mTask.read_reg = read_reg;
  mTask.i2cCallBack = i2cCallBack;
  mTask.next_state = next_state;

  if(mTask.timerHandle)
    timTimerCancel(mTask.timerHandle);

  mTask.timerHandle = timTimerSet(delay, 0, 50, ReadDataDelayCallback, NULL, true);

  if(!mTask.timerHandle)
    configASSERT(0);
}
/*------------------------------------------------------------------------------------------------*/
void af6133eTimerCbkF(uint64_t time)
{
    mTask.hwSampleTime = time;
}
/*------------------------------------------------------------------------------------------------*/
static struct AF6133ETask *af6133eDebugPoint;
/*------------------------------------------------------------------------------------------------*/
static void magGetCalibration(int32_t *cali, int32_t size)
{
}
/*------------------------------------------------------------------------------------------------*/
static void magSetCalibration(int32_t *cali, int32_t size)
{
}
/*------------------------------------------------------------------------------------------------*/
static void magGetData(void *sample)
{
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s \n", __func__);
  }

  struct TripleAxisDataPoint *tripleSample = (struct TripleAxisDataPoint *)sample;

  tripleSample->ix = mTask.factoryData.ix;
  tripleSample->iy = mTask.factoryData.iy;
  tripleSample->iz = mTask.factoryData.iz;
}
/*------------------------------------------------------------------------------------------------*/
static void af6133eSetDebugTrace(int32_t trace)
{
  mTask.control = trace;
}
/*------------------------------------------------------------------------------------------------*/
static void magGetSensorInfo(struct sensorInfo_t *data)
{
  memcpy(&data->mag_dev_info, &mTask.mag_dev_info, sizeof(struct mag_dev_info_t));
}
/*------------------------------------------------------------------------------------------------*/
static int af6133eReadTemp(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s, next_state = %d \n",__func__, (int)next_state);
  }

  int ret = 0;

  ret = rxTransferDataInfo(&mTask.dataInfo, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
  if (ret < 0) 
  {
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
    osLog(LOG_ERROR, "af6133eSample, rx dataInfo error\n");
    return -1;
  }
  mTask.txBuf[0] = REG_TEMP;
  return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr,
                       mTask.txBuf, 1,
                       mTask.rxBuf, 2,
                       i2cCallBack, next_state);
}
/*------------------------------------------------------------------------------------------------*/
static int af6133eSample(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s, next_state = %d \n",__func__, (int)next_state);
  }

#ifdef AF6133E_TEMP_COMP
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "af6133e: %s, odr:%d, count:%d,\n", __func__, mTask.odr, mTask.t_count);
  }
  if(mTask.odr > 0 && mTask.t_count++ > mTask.odr)
  {
    int16_t i, j;
    int16_t buf[TEMP_MF_NUM];
    int16_t delta;
    
    mTask.t_count = 0;
  
    mTask.t_buf[mTask.t_index] = (mTask.rxBuf[1] << 8) | mTask.rxBuf[0];
    mTask.t_buf[mTask.t_index] = (int16_t)((float)mTask.t_buf[mTask.t_index] * TEMP_RESOLUTION);
    
    for(i=0;i<TEMP_MF_NUM;i++)
      buf[i] = mTask.t_buf[i];

    for(i=0;i<(TEMP_MF_NUM-1);i++)
      for(j=0;j<(TEMP_MF_NUM-1);j++)
        if(buf[j] > buf[j+1])
        {
          int16_t temp = buf[j+1];
          buf[j+1] = buf[j];
          buf[j] = temp;
        }
    
    mTask.dt = buf[TEMP_MF_IDX] - mTask.t0;
    delta = mTask.dt - mTask.dt_pre;

    if(mTask.control & FUNC_MASK_DEBUG)
    {
      osLog(LOG_INFO, "af6133e: %s, dt:%d, buf[%d]:%d, t0:%d \n", __func__, mTask.dt, TEMP_MF_IDX, buf[TEMP_MF_IDX], mTask.t0);
      osLog(LOG_INFO, "af6133e: %s, delta:%d, dt_pre:%d \n", __func__, delta, mTask.dt_pre);
    }
    
    if(vtc_fabs(delta) > TEMP_DELTA_THRESHOLD)
      mTask.dt_pre = mTask.dt;
    else
      mTask.dt = mTask.dt_pre;
      
    if(++mTask.t_index >= TEMP_MF_NUM)
      mTask.t_index = 0;
  }
#endif

  mTask.txBuf[0] = REG_DATA;
  return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr,
                       mTask.txBuf, 1,
                       mTask.rxBuf, 6,
                       i2cCallBack, next_state);
}
/*------------------------------------------------------------------------------------------------*/
static int af6133eConvert(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                          void *inBuf, uint8_t inSize, uint8_t elemInSize,
                          void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s, next_state = %d \n",__func__, (int)next_state);
  }

  struct magData *data = mTask.magPacket.outBuf;
  
  uint64_t timestamp = 0;
  int16_t idata[AXES_NUM];
  float xyz[AXES_NUM];
  float remap_data[AXES_NUM];
  
  idata[0] = (int16_t)((mTask.rxBuf[1] << 8) | mTask.rxBuf[0]);
  idata[1] = (int16_t)((mTask.rxBuf[3] << 8) | mTask.rxBuf[2]);
  idata[2] = (int16_t)((mTask.rxBuf[5] << 8) | mTask.rxBuf[4]);

  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "af6133e: (int16_t) mag_data: %d, %d, %d \n", (int)(idata[0]), (int)(idata[1]), (int)(idata[2]));
  }

  data[0].x = (float)(idata[0]);
  data[0].y = (float)(idata[1]);
  data[0].z = (float)(idata[2]);
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "af6133e: (float) mag_data: %d, %d, %d \n", (int)(data[0].x), (int)(data[0].y), (int)(data[0].z));
  }
  
#ifdef AF6133E_SET_OFFSET
  /* Reduce sensor offset */
  if(mTask.control & FUNC_MASK_BISC)
  {
    data[0].x = (float)(data[0].x - mTask.offset[0]);
    data[0].y = (float)(data[0].y - mTask.offset[1]);
    data[0].z = (float)(data[0].z - mTask.offset[2]);
    if(mTask.control & FUNC_MASK_DEBUG)
    {
      osLog(LOG_INFO, "af6133e: comp offset mag_data: %d, %d, %d \n", (int)(data[0].x), (int)(data[0].y), (int)(data[0].z));
    }
  }
#endif  
#ifdef AF6133E_TEMP_COMP
  /* Temperature compensation */
  if(mTask.control & FUNC_MASK_TEMP)
  {
    data[0].x += (float)(data[0].x * (float)mTask.dt * TEMP_SENS_COEFF_X);
    data[0].y += (float)(data[0].y * (float)mTask.dt * TEMP_SENS_COEFF_Y);
    data[0].z += (float)(data[0].z * (float)mTask.dt * TEMP_SENS_COEFF_Z);
    if(mTask.control & FUNC_MASK_DEBUG)
    {
      osLog(LOG_INFO, "af6133e: comp temp mag_data: %d, %d, %d, dt:%d \n", (int)(data[0].x), (int)(data[0].y), (int)(data[0].z), mTask.dt);
    }
  }
#endif
    
  /* Multiple sensor gain */
  if(mTask.control & FUNC_MASK_GAIN)
  {
    data[0].x = (data[0].x * mTask.gain[0]);
    data[0].y = (data[0].y * mTask.gain[1]);
    data[0].z = (data[0].z * mTask.gain[2]);
    
    if(mTask.control & FUNC_MASK_DEBUG)
    {
      osLog(LOG_INFO, "af6133e: comp gain mag_data: %d, %d, %d \n", (int)(data[0].x), (int)(data[0].y), (int)(data[0].z));
    }
  }
    
  /* Compensation */
  if(mTask.control & FUNC_MASK_COEF)
  {
    xyz[0] = data[0].x + (data[0].y * mTask.comp_coeff[0]);
    xyz[1] = data[0].y + (data[0].x * mTask.comp_coeff[1]);
    xyz[2] = data[0].z + (data[0].x * mTask.comp_coeff[2])
                       + (data[0].y * mTask.comp_coeff[3]);
  }
  else
  {
    xyz[0] = data[0].x;
    xyz[1] = data[0].y;
    xyz[2] = data[0].z;
  }
  
  xyz[0] = -xyz[0];
  xyz[1] = -xyz[1];
  xyz[2] = -xyz[2];

  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "af6133e: comp coeff mag_data: %d, %d, %d \n", (int)(xyz[0]), (int)(xyz[1]), (int)(xyz[2]));
  }

  /* AXIS change */
  remap_data[mTask.cvt.map[AXIS_X]] = (float)((float)(mTask.cvt.sign[AXIS_X]) * xyz[AXIS_X]);
  remap_data[mTask.cvt.map[AXIS_Y]] = (float)((float)(mTask.cvt.sign[AXIS_Y]) * xyz[AXIS_Y]);
  remap_data[mTask.cvt.map[AXIS_Z]] = (float)((float)(mTask.cvt.sign[AXIS_Z]) * xyz[AXIS_Z]);

  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "af6133e: change axis mag_data: %d, %d, %d \n", (int)(remap_data[0]), (int)(remap_data[1]), (int)(remap_data[2]));
  }

  //timestamp = rtcGetTime();

  data[0].x = remap_data[AXIS_X] * RESOLUTION_M;  //uT
  data[0].y = remap_data[AXIS_Y] * RESOLUTION_M;
  data[0].z = remap_data[AXIS_Z] * RESOLUTION_M;

  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "af6133e: convert from count to uT mag_data: %d, %d, %d (uT)(x100)\n", (int16_t)(data[0].x*100), (int16_t)(data[0].y*100), (int16_t)(data[0].z*100));
  }
  
#ifdef AF6133E_LC_FILTER
  if(mTask.control & FUNC_MASK_LC_FILTER)
  {
    if(((data[0].x - mTask.fil_buf[0]) > +LC_FILTER_THRESHOLD ||
        (data[0].x - mTask.fil_buf[0]) < -LC_FILTER_THRESHOLD ||
        (data[0].y - mTask.fil_buf[1]) > +LC_FILTER_THRESHOLD ||
        (data[0].y - mTask.fil_buf[1]) < -LC_FILTER_THRESHOLD ||
        (data[0].z - mTask.fil_buf[2]) > +LC_FILTER_THRESHOLD ||
        (data[0].z - mTask.fil_buf[2]) < -LC_FILTER_THRESHOLD) && mTask.fil_init == 1)
    {
      if(mTask.fil_count < LC_FILTER_NUMBER)
      {
        data[0].x = mTask.fil_buf[0];
        data[0].y = mTask.fil_buf[1];
        data[0].z = mTask.fil_buf[2];
      }
      else
      {
        mTask.fil_buf[0] = data[0].x;
        mTask.fil_buf[1] = data[0].y;
        mTask.fil_buf[2] = data[0].z;
      }
      mTask.fil_count++;
    }	
    else
    {
      mTask.fil_buf[0] = data[0].x;
      mTask.fil_buf[1] = data[0].y;
      mTask.fil_buf[2] = data[0].z;
      mTask.fil_count = 0;
      mTask.fil_init = 1;
    }
    if(mTask.control & FUNC_MASK_DEBUG)
    {
      osLog(LOG_INFO, "af6133e: LC Filter mag_data: %d, %d, %d (uT)(x100)\n", (int16_t)(data[0].x*100), (int16_t)(data[0].y*100), (int16_t)(data[0].z*100));
    }
  }
#endif

  mTask.factoryData.ix = (int32_t)(data[0].x * MAGNETOMETER_INCREASE_NUM_AP);
  mTask.factoryData.iy = (int32_t)(data[0].y * MAGNETOMETER_INCREASE_NUM_AP);
  mTask.factoryData.iz = (int32_t)(data[0].z * MAGNETOMETER_INCREASE_NUM_AP);

  timestamp = addThenRetreiveAverageMagTimeStamp(mTask.hwSampleTime);
  txTransferDataInfo(&mTask.dataInfo, 1, timestamp, data);

  mTask.txBuf[0] = REG_MEASURE;
  mTask.txBuf[1] = 0x01;
  return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                     mTask.txBuf, 2,
                     i2cCallBack, next_state);
}
/*------------------------------------------------------------------------------------------------*/
static int af6133eEnable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s, next_state = %d \n",__func__, (int)next_state);
    osLog(LOG_INFO, "%s, gain: %d, %d, %d (x100)\n",__func__, (int)(mTask.gain[0]*100),(int)(mTask.gain[1]*100),(int)(mTask.gain[2]*100));
  }

#ifdef AF6133E_LC_FILTER
  mTask.fil_init = 0;
  mTask.fil_count = 0;
#endif

  mTask.txBuf[0] = REG_AVG;
  mTask.txBuf[1] = 0x47;
  i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

  mTask.txBuf[0] = REG_SR_MODE;
  mTask.txBuf[1] = 0x0D;
  i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

  mTask.txBuf[0] = REG_OSR;
  mTask.txBuf[1] = 0x3D;
  i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

  mTask.txBuf[0] = REG_OSC_FREQ;
  mTask.txBuf[1] = 0x3F;
  i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);
  
  mTask.txBuf[0] = REG_AVG_2ND;
  mTask.txBuf[1] = 0x50;
  i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

  mTask.txBuf[0] = REG_CHOPPER;
  mTask.txBuf[1] = 0xC1;
  i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

  sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
  return 0;
}
/*------------------------------------------------------------------------------------------------*/
static int af6133eDisable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                          void *inBuf, uint8_t inSize, uint8_t elemInSize,
                          void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s, next_state = %d \n",__func__, (int)next_state);
  }

  mTask.txBuf[0] = REG_MEASURE;
  mTask.txBuf[1] = 0x00;

  return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                     mTask.txBuf, 2,
                     i2cCallBack, next_state);
}
/*------------------------------------------------------------------------------------------------*/
static int af6133eRate(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                       void *inBuf, uint8_t inSize, uint8_t elemInSize,
                       void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s, next_state = %d \n",__func__, (int)next_state);
  }

  struct magCntlPacket cntlPacket;

  int ret = 0;
  uint32_t sample_rate, water_mark;
  
  ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
  
  if (ret < 0) 
  {
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
    osLog(LOG_ERROR, "af6133eRate, rx inSize and elemSize error\n");
    return -1;
  }
  
  sample_rate = cntlPacket.rate;
  water_mark = cntlPacket.waterMark;

  mTask.odr = (uint32_t)(sample_rate / 1000);
  
  osLog(LOG_INFO, "af6133eRate: %d, water_mark:%d\n", (int)sample_rate, (int)water_mark);
 
  sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);

  return 0;
}
/*------------------------------------------------------------------------------------------------*/
static int af6133e_init_measurement_conf1(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                          void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                          void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
#ifdef AF6133E_SENHUB_DEBUG
    osLog(LOG_INFO, "%s, next_state = %d \n",__func__, (int)next_state);
#endif

  mTask.control = FUNC_MASK_ACTIVE;

  mTask.txBuf[0] = REG_AVG;
  mTask.txBuf[1] = 0x47; 
  i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

  mTask.txBuf[0] = REG_SR_MODE;
  mTask.txBuf[1] = 0x0D; 
  i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

  mTask.txBuf[0] = REG_OSR;
  mTask.txBuf[1] = 0x3D; 
  i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

  mTask.txBuf[0] = REG_OSC_FREQ;
  mTask.txBuf[1] = 0x3F; 

  return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                     mTask.txBuf, 2,
                     i2cCallBack, next_state);
}
/*------------------------------------------------------------------------------------------------*/
static int af6133e_init_measurement_conf2(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                          void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                          void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
#ifdef AF6133E_SENHUB_DEBUG
    osLog(LOG_INFO, "%s, next_state = %d \n",__func__, (int)next_state);
#endif

  mTask.txBuf[0] = REG_XY_WAITING;
  mTask.txBuf[1] = 0x07; 
  i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

#ifdef AF6133E_SET_OFFSET
  mTask.txBuf[0] = REG_AVG_2ND;
  mTask.txBuf[1] = 0x10; 
  i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

  mTask.txBuf[0] = REG_MEASURE;
  mTask.txBuf[1] = 0x01; 
  i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

  SetReadDataTimer(i2cCallBack, next_state, AF6133E_MEASURE_DELAY, REG_DATA);
#else  
  sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
#endif //AF6133E_SET_OFFSET

  return 0;
}
/*------------------------------------------------------------------------------------------------*/
static int af6133e_init_offset(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                               void *inBuf, uint8_t inSize, uint8_t elemInSize,
                               void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
#ifdef AF6133E_SENHUB_DEBUG
    osLog(LOG_INFO, "%s, next_state = %d \n",__func__, (int)next_state);
#endif

#ifdef AF6133E_SET_OFFSET
  int16_t idata[3];

  idata[0] = (int16_t)((mTask.rxBuf[1] << 8) | mTask.rxBuf[0]);
  idata[1] = (int16_t)((mTask.rxBuf[3] << 8) | mTask.rxBuf[2]);
  idata[2] = (int16_t)((mTask.rxBuf[5] << 8) | mTask.rxBuf[4]);

  mTask.offset[0] = (float)(idata[0]);
  mTask.offset[1] = (float)(idata[1]);
  mTask.offset[2] = (float)(idata[2]);

  mTask.txBuf[0] = REG_AVG_2ND;
  mTask.txBuf[1] = 0x50; 
  i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

  mTask.txBuf[0] = REG_MEASURE;
  mTask.txBuf[1] = 0x01; 
  i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

  SetReadDataTimer(i2cCallBack, next_state, AF6133E_MEASURE_DELAY, REG_DATA);
#else
  sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
#endif

  return 0;
}
/*------------------------------------------------------------------------------------------------*/
static int af6133e_init_temp(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                             void *inBuf, uint8_t inSize, uint8_t elemInSize,
                             void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
#ifdef AF6133E_SENHUB_DEBUG
    osLog(LOG_INFO, "%s, next_state = %d \n",__func__, (int)next_state);
#endif

#ifdef AF6133E_SET_OFFSET
  int16_t idata[3];

  idata[0] = (int16_t)((mTask.rxBuf[1] << 8) | mTask.rxBuf[0]);
  idata[1] = (int16_t)((mTask.rxBuf[3] << 8) | mTask.rxBuf[2]);
  idata[2] = (int16_t)((mTask.rxBuf[5] << 8) | mTask.rxBuf[4]);

  mTask.offset[0] += (float)(idata[0]);
  mTask.offset[1] += (float)(idata[1]);
  mTask.offset[2] += (float)(idata[2]);
  
  mTask.offset[0] = 0;
  mTask.offset[1] = 0;
  mTask.offset[2] = mTask.offset[2] / 2;
#endif

  sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
  return 0;
}
/*------------------------------------------------------------------------------------------------*/
static int af6133eRegisterCore(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                               void *inBuf, uint8_t inSize, uint8_t elemInSize,
                               void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
#ifdef AF6133E_SENHUB_DEBUG
    osLog(LOG_INFO, "%s, next_state = %d \n",__func__, (int)next_state);
#endif

  struct sensorCoreInfo mInfo;
  memset(&mInfo, 0x00, sizeof(struct sensorCoreInfo));

  mInfo.sensType = SENS_TYPE_MAG;
  mInfo.getCalibration = magGetCalibration;
  mInfo.setCalibration = magSetCalibration;
  mInfo.getData = magGetData;
  mInfo.getSensorInfo = magGetSensorInfo;
  mInfo.setDebugTrace = af6133eSetDebugTrace;

  sensorCoreRegister(&mInfo);

  sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
  return 0;
}
/*------------------------------------------------------------------------------------------------*/
static int calculation_BISC_parameter()
{
  if(mTask.bist_x[0] > 0 && mTask.bist_y[1] > 0 && mTask.bist_z[2] > 0)
  {
    float SSensz = ((float)mTask.bist_x[2]*BIST_COEFF_X) * ((float)mTask.bist_x[2]*BIST_COEFF_X) +
                   ((float)mTask.bist_y[2]*BIST_COEFF_Y) * ((float)mTask.bist_y[2]*BIST_COEFF_Y) +
                   ((float)mTask.bist_z[2]*BIST_COEFF_Z) * ((float)mTask.bist_z[2]*BIST_COEFF_Z);

    SSensz = sqrt(SSensz);
    
    //calculate sens. compensation coeff.
    mTask.gain[0] = BIST_GAIN_COEFF_X / mTask.bist_x[0];
    mTask.gain[1] = BIST_GAIN_COEFF_Y / mTask.bist_y[1];
    mTask.gain[2] = BIST_GAIN_COEFF_Z / mTask.bist_z[2];

    mTask.comp_coeff[0] = -(mTask.bist_y[0]*BIST_COEFF_Y) / (mTask.bist_x[0]*BIST_COEFF_X);
    mTask.comp_coeff[1] = -(mTask.bist_x[1]*BIST_COEFF_X) / (mTask.bist_y[1]*BIST_COEFF_Y);
    mTask.comp_coeff[2] = -(mTask.bist_x[2]*BIST_COEFF_X) / SSensz;
    mTask.comp_coeff[3] = -(mTask.bist_y[2]*BIST_COEFF_Y) / SSensz;
  }

#ifdef AF6133E_SENHUB_DEBUG
  //if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "af6133e_Bist: bisc_x: %d, %d, %d \n", (int)(mTask.bist_x[0]),(int)(mTask.bist_x[1]),(int)(mTask.bist_x[2]));
    osLog(LOG_INFO, "af6133e_Bist: bisc_y: %d, %d, %d \n", (int)(mTask.bist_y[0]),(int)(mTask.bist_y[1]),(int)(mTask.bist_y[2]));
    osLog(LOG_INFO, "af6133e_Bist: bisc_z: %d, %d, %d \n", (int)(mTask.bist_z[0]),(int)(mTask.bist_z[1]),(int)(mTask.bist_z[2]));
    osLog(LOG_INFO, "af6133e_Bist: gain: %d, %d, %d (x100)\n", (int)(mTask.gain[0]*100),(int)(mTask.gain[1]*100),(int)(mTask.gain[2]*100));
    osLog(LOG_INFO, "af6133e_Bist: coeff: %d, %d, %d, %d (x100)\n", (int)(mTask.comp_coeff[0]*100),(int)(mTask.comp_coeff[1]*100),(int)(mTask.comp_coeff[2]*100),(int)(mTask.comp_coeff[3]*100));
}
#endif

  if((mTask.gain[0]) <= GAIN_X_MIN || (mTask.gain[0]) >= GAIN_X_MAX ||
     (mTask.gain[1]) <= GAIN_Y_MIN || (mTask.gain[1]) >= GAIN_Y_MAX ||
     (mTask.gain[2]) <= GAIN_Z_MIN || (mTask.gain[2]) >= GAIN_Z_MAX ||
     (mTask.comp_coeff[0] + mTask.comp_coeff[1]) >  BIST_COEFF_LIMIT_XY ||
     (mTask.comp_coeff[0] + mTask.comp_coeff[1]) < -BIST_COEFF_LIMIT_XY ||
      vtc_fabs(mTask.comp_coeff[2]) > BIST_COEFF_LIMIT_Z ||
      vtc_fabs(mTask.comp_coeff[3]) > BIST_COEFF_LIMIT_Z )
  {
    mTask.gain[0] = 1;
    mTask.gain[1] = 1;
    mTask.gain[2] = 1;
    
    mTask.comp_coeff[0] = 0;
    mTask.comp_coeff[1] = 0;
    mTask.comp_coeff[2] = 0;
    mTask.comp_coeff[3] = 0;

    osLog(LOG_INFO, "af6133e_Bist: parameters invalid\n");
  }

  return 1;
}
/*------------------------------------------------------------------------------------------------*/
static int af6133e_Bist_init(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                             void *inBuf, uint8_t inSize, uint8_t elemInSize,
                             void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s, next_state = %d \n",__func__, (int)next_state);
  }

#ifdef AF6133E_SET_OFFSET  
 #ifdef AF6133E_TEMP_COMP
  int16_t i;

  mTask.t0 = (mTask.rxBuf[1] << 8) | mTask.rxBuf[0];
  mTask.t0 = (int16_t)((float)mTask.t0 * TEMP_RESOLUTION);
  
  mTask.dt = 0;
  mTask.dt_pre = 0;
  mTask.t_count = 0;
  mTask.t_index = 0;

  for(i=0;i<TEMP_MF_NUM;i++)
    mTask.t_buf[i] = mTask.t0;
 #endif
#endif

  mTask.gain[0] = 1;
  mTask.gain[1] = 1;
  mTask.gain[2] = 1;
  
  mTask.bist_count = 0;
  mTask.bist_status = 0;
  
  mTask.comp_coeff[0] = 0;
  mTask.comp_coeff[1] = 0;
  mTask.comp_coeff[2] = 0;
  mTask.comp_coeff[3] = 0;

  mTask.bist_x[0] = 0;
  mTask.bist_x[1] = 0;
  mTask.bist_x[2] = 0;
  mTask.bist_y[0] = 0;
  mTask.bist_y[1] = 0;
  mTask.bist_y[2] = 0;
  mTask.bist_z[0] = 0;
  mTask.bist_z[1] = 0;
  mTask.bist_z[2] = 0;
  
  mTask.test2_idx = 0;
  mTask.test2_reg[0] = 0x04;
  mTask.test2_reg[1] = 0x0C;
  mTask.test2_reg[2] = 0x02;
  mTask.test2_reg[3] = 0x0A;
  mTask.test2_reg[4] = 0x01;
  mTask.test2_reg[5] = 0x09;

  mTask.control_8m_disable = 0;
  
  mTask.txBuf[0] = REG_CHOPPER;
  mTask.txBuf[1] = 0xF1; // open 8M
  
  return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr,
                     mTask.txBuf, 2,
                     i2cCallBack, next_state);
}
/*------------------------------------------------------------------------------------------------*/
static int af6133e_Bist_end(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                            void *inBuf, uint8_t inSize, uint8_t elemInSize,
                            void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s, next_state=%d \n", __func__, (int)next_state);
  }

  calculation_BISC_parameter();

  osLog(LOG_ERROR, "af6133e: Init successfully! \n");

  sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
  return 0;
}
/*------------------------------------------------------------------------------------------------*/
static int af6133e_Bist_Flow(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                             void *inBuf, uint8_t inSize, uint8_t elemInSize,
                             void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
  int16_t idata[3];
  int16_t bist_num_p;
  int16_t bist_num_n;
  uint64_t delay_time = 0;

  idata[0] = (int16_t)((mTask.rxBuf[1] << 8) | mTask.rxBuf[0]);
  idata[1] = (int16_t)((mTask.rxBuf[3] << 8) | mTask.rxBuf[2]);
  idata[2] = (int16_t)((mTask.rxBuf[5] << 8) | mTask.rxBuf[4]);

  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s status:%d, idata: %d, %d, %d, bist_count:%d \n", __func__, mTask.bist_status, idata[0], idata[1], idata[2], mTask.bist_count);
  }

  switch(mTask.bist_status)
  {
    case 0: /* give field and delay 1ms, initial count. */
           if(mTask.test2_idx == 0 || mTask.test2_idx == 2 || mTask.test2_idx == 4)
           {
             mTask.bist_pos[0] = 0;
             mTask.bist_pos[1] = 0;
             mTask.bist_pos[2] = 0;
             mTask.bist_pass_count_p = 0;

             if(mTask.control & FUNC_MASK_DEBUG){
               osLog(LOG_INFO, "%s +Field: 0x%x = 0x%x \n", __func__, REG_TEST2, mTask.test2_reg[mTask.test2_idx]);
             }
           }
           else
           {
             mTask.bist_neg[0] = 0;
             mTask.bist_neg[1] = 0;
             mTask.bist_neg[2] = 0;
             mTask.bist_pass_count_n = 0;

             if(mTask.control & FUNC_MASK_DEBUG){
               osLog(LOG_INFO, "%s -Field: 0x%x = 0x%x \n", __func__, REG_TEST2, mTask.test2_reg[mTask.test2_idx]);
             }
           }
           mTask.bist_count = 0;
           mTask.bist_status = 1;
      
           if(mTask.test2_idx < 6)
           {
             mTask.txBuf[0] = REG_TEST2;
             mTask.txBuf[1] = mTask.test2_reg[mTask.test2_idx];
             i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);
           }
           else
           {
             delay_time = 0;
             mTask.bist_status = 3;
           }
      
           delay_time = AF6133E_FIELD_DELAY;
      break;
    case 1: /* first measure bisc and delay 6ms after give field. */
           mTask.txBuf[0] = REG_MEASURE;
           mTask.txBuf[1] = 0x01; 
           i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);
      
           delay_time = AF6133E_BIST_DELAY;
           mTask.bist_status = 2;

           if(mTask.control & FUNC_MASK_DEBUG){
             osLog(LOG_INFO, "%s new field first measure bist data \n", __func__);
           }
      break;
    case 2: /* (keep field) read mag data, measure bisc and delay 6ms. */
           if(mTask.test2_idx == 0 || mTask.test2_idx == 2 || mTask.test2_idx == 4)
           {
             if(idata[0] <  16383 && idata[1] <  16383 && idata[2] <  16383 &&
                idata[0] > -16384 && idata[1] > -16384 && idata[2] > -16384)
             {
               mTask.bist_pos[0] += (float)(idata[0]);
               mTask.bist_pos[1] += (float)(idata[1]);
               mTask.bist_pos[2] += (float)(idata[2]);
             }
             else
             {
               mTask.bist_pass_count_p++;
             }
           }
           else
           {
             if(idata[0] <  16383 && idata[1] <  16383 && idata[2] <  16383 &&
                idata[0] > -16384 && idata[1] > -16384 && idata[2] > -16384)
             {
               mTask.bist_neg[0] += (float)(idata[0]);
               mTask.bist_neg[1] += (float)(idata[1]);
               mTask.bist_neg[2] += (float)(idata[2]);
             }
             else
             {
               mTask.bist_pass_count_n++;
             }
           }
           if(++mTask.bist_count == MAG_BIST_LOOP)
           {
             bist_num_p = MAG_BIST_LOOP - mTask.bist_pass_count_p;
             bist_num_n = MAG_BIST_LOOP - mTask.bist_pass_count_n;
             
             if(mTask.test2_idx == 1 && (bist_num_p != 0 && bist_num_n != 0))
             {
               mTask.bist_x[0] = (float)(mTask.bist_neg[0] / bist_num_n - mTask.bist_pos[0] / bist_num_p);
               mTask.bist_x[1] = (float)(mTask.bist_neg[1] / bist_num_n - mTask.bist_pos[1] / bist_num_p);
               mTask.bist_x[2] = (float)(mTask.bist_neg[2] / bist_num_n - mTask.bist_pos[2] / bist_num_p);

               if(mTask.control & FUNC_MASK_DEBUG){
                 osLog(LOG_INFO, "%s bist_x:%d, %d, %d, neg_num:%d, bist_neg:%d, %d, %d, pos_num:%d, bist_pos:%d, %d, %d \n"
                               , __func__, (int)(mTask.bist_x[0]), (int)(mTask.bist_x[1]), (int)(mTask.bist_x[2])
                               , bist_num_n, (int)(mTask.bist_neg[0]), (int)(mTask.bist_neg[1]), (int)(mTask.bist_neg[2])
                               , bist_num_p, (int)(mTask.bist_pos[0]), (int)(mTask.bist_pos[1]), (int)(mTask.bist_pos[2]));
               }
             }
             else if(mTask.test2_idx == 3 && (bist_num_p != 0 && bist_num_n != 0))
             {
               mTask.bist_y[0] = (float)(mTask.bist_neg[0] / bist_num_n - mTask.bist_pos[0] / bist_num_p);
               mTask.bist_y[1] = (float)(mTask.bist_neg[1] / bist_num_n - mTask.bist_pos[1] / bist_num_p);
               mTask.bist_y[2] = (float)(mTask.bist_neg[2] / bist_num_n - mTask.bist_pos[2] / bist_num_p);

               if(mTask.control & FUNC_MASK_DEBUG){
                 osLog(LOG_INFO, "%s bist_y:%d, %d, %d, neg_num:%d, bist_neg:%d, %d, %d, pos_num:%d, bist_pos:%d, %d, %d \n"
                               , __func__, (int)(mTask.bist_y[0]), (int)(mTask.bist_y[1]), (int)(mTask.bist_y[2])
                               , bist_num_n, (int)(mTask.bist_neg[0]), (int)(mTask.bist_neg[1]), (int)(mTask.bist_neg[2])
                               , bist_num_p, (int)(mTask.bist_pos[0]), (int)(mTask.bist_pos[1]), (int)(mTask.bist_pos[2]));
               }
             }
             else if(mTask.test2_idx == 5 && (bist_num_p != 0 && bist_num_n != 0))
             {
               mTask.bist_z[0] = (float)(mTask.bist_neg[0] / bist_num_n - mTask.bist_pos[0] / bist_num_p);
               mTask.bist_z[1] = (float)(mTask.bist_neg[1] / bist_num_n - mTask.bist_pos[1] / bist_num_p);
               mTask.bist_z[2] = (float)(mTask.bist_neg[2] / bist_num_n - mTask.bist_pos[2] / bist_num_p);

               if(mTask.control & FUNC_MASK_DEBUG){
                 osLog(LOG_INFO, "%s bist_z:%d, %d, %d, neg_num:%d, bist_neg:%d, %d, %d, pos_num:%d, bist_pos:%d, %d, %d \n"
                               , __func__, (int)(mTask.bist_z[0]), (int)(mTask.bist_z[1]), (int)(mTask.bist_z[2])
                               , bist_num_n, (int)(mTask.bist_neg[0]), (int)(mTask.bist_neg[1]), (int)(mTask.bist_neg[2])
                               , bist_num_p, (int)(mTask.bist_pos[0]), (int)(mTask.bist_pos[1]), (int)(mTask.bist_pos[2]));
               }
               
               if(mTask.bist_z[2] < 0)
                 mTask.bist_z[2] = -mTask.bist_z[2];
             }
             mTask.bist_status = 0;
             delay_time = 0;

             if(++mTask.test2_idx == 6)
             {
               mTask.txBuf[0] = REG_CHOPPER;  
               mTask.txBuf[1] = 0xC1; // close 8M
               i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

               mTask.txBuf[0] = REG_TEST2;
               mTask.txBuf[1] = 0x00;
               i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

               mTask.txBuf[0] = REG_MEASURE;
               mTask.txBuf[1] = 0x01; 
               i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);
     
               delay_time = AF6133E_MEASURE_DELAY;

               if(mTask.control & FUNC_MASK_DEBUG){
                 osLog(LOG_INFO, "%s close 8M \n", __func__);
               }
             }
           }
           else
           {
             mTask.txBuf[0] = REG_MEASURE;
             mTask.txBuf[1] = 0x01; 
             i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);
             delay_time = AF6133E_BIST_DELAY;
             mTask.bist_status = 2;
           }
      break;
    case 3: // finish stay here
           delay_time = 0;
      break;
  }
  SetReadDataTimer(i2cCallBack, next_state, delay_time, REG_DATA);
  
  return 0;
}
#ifdef AF6133E_SELF_TEST
/*------------------------------------------------------------------------------------------------*/
static int af6133e_self_test_run(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                 void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                 void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s, next_state = %d \n",__func__, (int)next_state);
  }

  mTask.bist_status = 0;
  
  mTask.test2_idx = 0;
  mTask.test2_reg[0] = 0x04;
  mTask.test2_reg[1] = 0x02;
  mTask.test2_reg[2] = 0x01;

  mTask.control_8m_disable = 0;
  
  mTask.txBuf[0] = REG_CHOPPER;
  mTask.txBuf[1] = 0xF1; // open 8M
  i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);
  
  mTask.txBuf[0] = REG_MEASURE;
  mTask.txBuf[1] = 0x01;
  i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

  SetReadDataTimer(i2cCallBack, next_state, AF6133E_MEASURE_DELAY, REG_DATA);

  return 0;
}
/*------------------------------------------------------------------------------------------------*/
static int af6133e_self_test_flow(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                  void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                  void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{

  uint64_t delay_time = 0;
  int16_t idata[3];

  idata[0] = (int16_t)((mTask.rxBuf[1] << 8) | mTask.rxBuf[0]);
  idata[1] = (int16_t)((mTask.rxBuf[3] << 8) | mTask.rxBuf[2]);
  idata[2] = (int16_t)((mTask.rxBuf[5] << 8) | mTask.rxBuf[4]);

  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s REG_TEST2[%d]=0x%x, status:%d, idata: %d, %d, %d \n", __func__, mTask.test2_idx, mTask.test2_reg[mTask.test2_idx], mTask.bist_status, idata[0], idata[1], idata[2]);
  }
  
  switch(mTask.bist_status)
  {
    case 0: // normal field
           if(mTask.test2_idx == 0)
           {
             mTask.normal_field[0] = (float)(idata[0]);
             mTask.normal_field[1] = (float)(idata[1]);
             mTask.normal_field[2] = (float)(idata[2]);
           }
           else if(mTask.test2_idx == 1)
           {
             mTask.self_bist_x[0] = (float)(mTask.normal_field[0] - (float)(idata[0]));
             mTask.self_bist_x[1] = (float)(mTask.normal_field[1] - (float)(idata[1]));
             mTask.self_bist_x[2] = (float)(mTask.normal_field[2] - (float)(idata[2]));
           }
           else if(mTask.test2_idx == 2)
           {
             mTask.self_bist_y[0] = (float)(mTask.normal_field[0] - (float)(idata[0]));
             mTask.self_bist_y[1] = (float)(mTask.normal_field[1] - (float)(idata[1]));
             mTask.self_bist_y[2] = (float)(mTask.normal_field[2] - (float)(idata[2]));
           }
           else if(mTask.test2_idx == 3)
           {
             mTask.self_bist_z[0] = (float)(mTask.normal_field[0] - (float)(idata[0]));
             mTask.self_bist_z[1] = (float)(mTask.normal_field[1] - (float)(idata[1]));
             mTask.self_bist_z[2] = (float)(mTask.normal_field[2] - (float)(idata[2]));
           }
           
           if(mTask.test2_idx < 3)
           {
             mTask.txBuf[0] = REG_CHOPPER;
             mTask.txBuf[1] = 0xF1; // open 8M
             i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

             mTask.txBuf[0] = REG_TEST2;
             mTask.txBuf[1] = mTask.test2_reg[mTask.test2_idx];
             i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);
           
             delay_time = AF6133E_FIELD_DELAY;
           
             mTask.bist_status = 1;
           }
           else
           {
             mTask.txBuf[0] = REG_CHOPPER;  
             mTask.txBuf[1] = 0xC1; // close 8M
             i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);
           
             mTask.txBuf[0] = REG_TEST2;
             mTask.txBuf[1] = 0x00;
             i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);
           
             mTask.txBuf[0] = REG_MEASURE;
             mTask.txBuf[1] = 0x01; 
             i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);

             delay_time = AF6133E_MEASURE_DELAY;
             
             mTask.bist_status = 2;

             if(mTask.control & FUNC_MASK_DEBUG)
             {
               osLog(LOG_INFO, "%s self-bist: %d, %d, %d \n", __func__, (int)(mTask.self_bist_x[0]), (int)(mTask.self_bist_y[1]), (int)(mTask.self_bist_z[2]));
             }
           }
      break;
    case 1:
           mTask.txBuf[0] = REG_MEASURE;
           mTask.txBuf[1] = 0x01;
           i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2, NULL, NULL);
           
           delay_time = AF6133E_BIST_DELAY;
           mTask.test2_idx++;
           mTask.bist_status = 0;
           mTask.control_8m_disable = 1;
      break;
    case 2:
           delay_time = 0;
      break;
  }
  
  SetReadDataTimer(i2cCallBack, next_state, delay_time, REG_DATA);
  
  return 0;
}
/*------------------------------------------------------------------------------------------------*/
static int af6133e_self_test_done(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                  void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                  void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s, next_state = %d \n",__func__, (int)next_state);
    osLog(LOG_INFO, "%s self-bist: %d, %d, %d \n", __func__, (int)(mTask.self_bist_x[0]), (int)(mTask.self_bist_y[1]), (int)(mTask.self_bist_z[2]));
  }

  mTask.control_8m_disable = 0;

  int selftest_result = 1;

  if(vtc_fabs(mTask.self_bist_x[0]) < BIST_TEST_MIN || vtc_fabs(mTask.self_bist_x[0]) > BIST_TEST_MAX ||
     vtc_fabs(mTask.self_bist_y[1]) < BIST_TEST_MIN || vtc_fabs(mTask.self_bist_y[1]) > BIST_TEST_MAX ||
     vtc_fabs(mTask.self_bist_z[2]) < BIST_TEST_MIN || vtc_fabs(mTask.self_bist_z[2]) > BIST_TEST_MAX)
  {
    selftest_result = -1;
    osLog(LOG_INFO, "%s self-bist fail: %d, %d, %d \n", __func__, (int)(mTask.self_bist_x[0]), (int)(mTask.self_bist_y[1]), (int)(mTask.self_bist_z[2]));
  }

  magSendTestResult(selftest_result);
  sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
  return 0;
}
#endif
/*------------------------------------------------------------------------------------------------*/
static struct sensorFsm af6133eFsm[] = {
  /* VTC Sample */
  sensorFsmCmd(STATE_SAMPLE,                  STATE_READTEMP,                af6133eReadTemp),
  sensorFsmCmd(STATE_READTEMP,                STATE_CONVERT,                 af6133eSample),
  sensorFsmCmd(STATE_CONVERT,                 STATE_SAMPLE_DONE,             af6133eConvert),

  /* VTC Enable/Disable */
  sensorFsmCmd(STATE_ENABLE,                  STATE_ENABLE_DONE,             af6133eEnable),
  
  /* VTC Disable */
  sensorFsmCmd(STATE_DISABLE,                 STATE_DISABLE_DONE,            af6133eDisable),
  
  /* VTC Rate */
  sensorFsmCmd(STATE_RATECHG,                 STATE_RATECHG_DONE,            af6133eRate),
    
  /* VTC RESET */
  sensorFsmCmd(STATE_SENSOR_INIT_MEAS_CONG1,  STATE_SENSOR_INIT_MEAS_CONG2,  af6133e_init_measurement_conf1),
  sensorFsmCmd(STATE_SENSOR_INIT_MEAS_CONG2,  STATE_SENSOR_INIT_OFFSET,      af6133e_init_measurement_conf2),
  sensorFsmCmd(STATE_SENSOR_INIT_OFFSET,      STATE_SENSOR_INIT_TEMP,        af6133e_init_offset),
  sensorFsmCmd(STATE_SENSOR_INIT_TEMP,        STATE_SENSOR_BIST_INIT,        af6133e_init_temp),
  
  sensorFsmCmd(STATE_SENSOR_BIST_INIT,        STATE_SENSOR_BIST_XPOS,        af6133e_Bist_init),

  sensorFsmCmd(STATE_SENSOR_BIST_XPOS,        STATE_SENSOR_BIST_XPOS,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_XPOS,        STATE_SENSOR_BIST_XPOS,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_XPOS,        STATE_SENSOR_BIST_XPOS,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_XPOS,        STATE_SENSOR_BIST_XPOS,        af6133e_Bist_Flow),
#ifdef MAG_BIST_LOOP_5_TIMES
  sensorFsmCmd(STATE_SENSOR_BIST_XPOS,        STATE_SENSOR_BIST_XPOS,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_XPOS,        STATE_SENSOR_BIST_XPOS,        af6133e_Bist_Flow),
#endif
  sensorFsmCmd(STATE_SENSOR_BIST_XPOS,        STATE_SENSOR_BIST_XNEG,        af6133e_Bist_Flow),

  sensorFsmCmd(STATE_SENSOR_BIST_XNEG,        STATE_SENSOR_BIST_XNEG,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_XNEG,        STATE_SENSOR_BIST_XNEG,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_XNEG,        STATE_SENSOR_BIST_XNEG,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_XNEG,        STATE_SENSOR_BIST_XNEG,        af6133e_Bist_Flow),
#ifdef MAG_BIST_LOOP_5_TIMES
  sensorFsmCmd(STATE_SENSOR_BIST_XNEG,        STATE_SENSOR_BIST_XNEG,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_XNEG,        STATE_SENSOR_BIST_XNEG,        af6133e_Bist_Flow),
#endif
  sensorFsmCmd(STATE_SENSOR_BIST_XNEG,        STATE_SENSOR_BIST_YPOS,        af6133e_Bist_Flow),


  sensorFsmCmd(STATE_SENSOR_BIST_YPOS,        STATE_SENSOR_BIST_YPOS,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_YPOS,        STATE_SENSOR_BIST_YPOS,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_YPOS,        STATE_SENSOR_BIST_YPOS,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_YPOS,        STATE_SENSOR_BIST_YPOS,        af6133e_Bist_Flow),
#ifdef MAG_BIST_LOOP_5_TIMES
  sensorFsmCmd(STATE_SENSOR_BIST_YPOS,        STATE_SENSOR_BIST_YPOS,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_YPOS,        STATE_SENSOR_BIST_YPOS,        af6133e_Bist_Flow),
#endif
  sensorFsmCmd(STATE_SENSOR_BIST_YPOS,        STATE_SENSOR_BIST_YNEG,        af6133e_Bist_Flow),

  sensorFsmCmd(STATE_SENSOR_BIST_YNEG,        STATE_SENSOR_BIST_YNEG,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_YNEG,        STATE_SENSOR_BIST_YNEG,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_YNEG,        STATE_SENSOR_BIST_YNEG,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_YNEG,        STATE_SENSOR_BIST_YNEG,        af6133e_Bist_Flow),
#ifdef MAG_BIST_LOOP_5_TIMES
  sensorFsmCmd(STATE_SENSOR_BIST_YNEG,        STATE_SENSOR_BIST_YNEG,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_YNEG,        STATE_SENSOR_BIST_YNEG,        af6133e_Bist_Flow),
#endif
  sensorFsmCmd(STATE_SENSOR_BIST_YNEG,        STATE_SENSOR_BIST_ZPOS,        af6133e_Bist_Flow),

  sensorFsmCmd(STATE_SENSOR_BIST_ZPOS,        STATE_SENSOR_BIST_ZPOS,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_ZPOS,        STATE_SENSOR_BIST_ZPOS,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_ZPOS,        STATE_SENSOR_BIST_ZPOS,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_ZPOS,        STATE_SENSOR_BIST_ZPOS,        af6133e_Bist_Flow),
#ifdef MAG_BIST_LOOP_5_TIMES
  sensorFsmCmd(STATE_SENSOR_BIST_ZPOS,        STATE_SENSOR_BIST_ZPOS,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_ZPOS,        STATE_SENSOR_BIST_ZPOS,        af6133e_Bist_Flow),
#endif
  sensorFsmCmd(STATE_SENSOR_BIST_ZPOS,        STATE_SENSOR_BIST_ZNEG,        af6133e_Bist_Flow),
                                                                                                
  sensorFsmCmd(STATE_SENSOR_BIST_ZNEG,        STATE_SENSOR_BIST_ZNEG,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_ZNEG,        STATE_SENSOR_BIST_ZNEG,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_ZNEG,        STATE_SENSOR_BIST_ZNEG,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_ZNEG,        STATE_SENSOR_BIST_ZNEG,        af6133e_Bist_Flow),
#ifdef MAG_BIST_LOOP_5_TIMES
  sensorFsmCmd(STATE_SENSOR_BIST_ZNEG,        STATE_SENSOR_BIST_ZNEG,        af6133e_Bist_Flow),
  sensorFsmCmd(STATE_SENSOR_BIST_ZNEG,        STATE_SENSOR_BIST_ZNEG,        af6133e_Bist_Flow),
#endif
  sensorFsmCmd(STATE_SENSOR_BIST_ZNEG,        STATE_SENSOR_BIST_END,         af6133e_Bist_Flow),

  sensorFsmCmd(STATE_SENSOR_BIST_END,         STATE_CORE,                    af6133e_Bist_end),
  sensorFsmCmd(STATE_CORE,                    STATE_INIT_DONE,               af6133eRegisterCore),
  
  /* VTC SELF-TEST */
#ifdef AF6133E_SELF_TEST
  sensorFsmCmd(STATE_SELFTEST,                STATE_SELFTEST_FIELD,          af6133e_self_test_run),
  sensorFsmCmd(STATE_SELFTEST_FIELD,          STATE_SENSOR_BIST_XPOS,        af6133e_self_test_flow),
  sensorFsmCmd(STATE_SENSOR_BIST_XPOS,        STATE_SENSOR_BIST_XPOS,        af6133e_self_test_flow),
  sensorFsmCmd(STATE_SENSOR_BIST_XPOS,        STATE_SENSOR_BIST_YPOS,        af6133e_self_test_flow),
  sensorFsmCmd(STATE_SENSOR_BIST_YPOS,        STATE_SENSOR_BIST_YPOS,        af6133e_self_test_flow),
  sensorFsmCmd(STATE_SENSOR_BIST_YPOS,        STATE_SENSOR_BIST_ZPOS,        af6133e_self_test_flow),
  sensorFsmCmd(STATE_SENSOR_BIST_ZPOS,        STATE_SENSOR_BIST_ZPOS,        af6133e_self_test_flow),
  sensorFsmCmd(STATE_SENSOR_BIST_ZPOS,        STATE_SELFTEST_END,            af6133e_self_test_flow),
  sensorFsmCmd(STATE_SELFTEST_END,            STATE_SELFTEST_DONE,           af6133e_self_test_done),
#endif
};
#ifndef CFG_MAG_CALIBRATION_IN_AP
/*------------------------------------------------------------------------------------------------*/
static int af6133eCaliApiGetOffset(float offset[AXES_NUM])
{
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s \n", __func__);
  }

  return 0;
}
/*------------------------------------------------------------------------------------------------*/
static int af6133eCaliApiSetOffset(float offset[AXES_NUM])
{
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s \n", __func__);
  }

  return 0;
}
/*------------------------------------------------------------------------------------------------*/
static int af6133eCaliApiSetGyroData(struct magCaliDataInPut *inputData)
{
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s \n", __func__);
  }

#ifdef VTC_HUBALGOLIB_READY
  float gyr[3];
  gyr[0] = inputData->x;
  gyr[1] = inputData->y;
  gyr[2] = inputData->z;
  vtc_lib_gyr_enter(gyr, inputData->timeStamp);
#endif
  return 0;
}
/*------------------------------------------------------------------------------------------------*/
static int af6133eDoCaliAPI(struct magCaliDataInPut *inputData, struct magCaliDataOutPut *outputData)
{
  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_INFO, "%s \n", __func__);
  }

  float Mag_data[3];   
  int16_t err = 0;
  int16_t accuracy = 0;
  float data_cali[AXES_NUM];
  float data_offset[AXES_NUM];
 
  Mag_data[0] = inputData->x;
  Mag_data[1] = inputData->y;
  Mag_data[2] = inputData->z;

#ifdef VTC_HUBALGOLIB_READY
  vtc_lib_mag_enter(Mag_data, inputData->timeStamp);
  err = vtc_lib_run(inputData->timeStamp);
  vtc_lib_get_calimag(data_cali, data_offset, &accuracy);
#endif

  if (err == 0)
  {
    outputData->x = Mag_data[AXIS_X];
    outputData->y = Mag_data[AXIS_Y];
    outputData->z = Mag_data[AXIS_Z];
    outputData->x_bias = 0;
    outputData->y_bias = 0;
    outputData->z_bias = 0;

  if(mTask.control & FUNC_MASK_DEBUG)
  {
    osLog(LOG_ERROR, "vtc_set_mag_data fail\n\r");
  }
    return err;
  }
  else
  {
    outputData->x = data_cali[AXIS_X];
    outputData->y = data_cali[AXIS_Y];
    outputData->z = data_cali[AXIS_Z];
    outputData->x_bias = (float)data_offset[AXIS_X];
    outputData->y_bias = (float)data_offset[AXIS_Y];
    outputData->z_bias = (float)data_offset[AXIS_Z];
    outputData->status = accuracy;

    if(mTask.control & FUNC_MASK_DEBUG)
    {
      osLog(LOG_ERROR, "vtc_set_mag_data success\n\r");
    }

  }

  return 0;
}
/*------------------------------------------------------------------------------------------------*/
static struct magCalibrationLibAPI af6133eCaliAPI = {
    .caliApiGetOffset    = af6133eCaliApiGetOffset,
    .caliApiSetOffset    = af6133eCaliApiSetOffset,
    .caliApiSetGyroData  = af6133eCaliApiSetGyroData,
    /*.caliApiSetAccelData = af6133eCaliApiSetAccelData,*/
    .doCaliApi           = af6133eDoCaliAPI
};
#endif
/*------------------------------------------------------------------------------------------------*/
int af6133eInit(void)
{
  osLog(LOG_INFO, "%s \n", __func__);

  int ret = 0;

  af6133eDebugPoint = &mTask;
  insertMagicNum(&mTask.magPacket);
  mTask.hw = get_cust_mag("af6133e");

  if (NULL == mTask.hw) {
    osLog(LOG_ERROR, "af6133e get_cust_mag fail\n");
    return -1;
  }

  mTask.i2c_addr = mTask.hw->i2c_addr[0];
  osLog(LOG_ERROR, "mag i2c_num: %d, i2c_addr: 0x%x\n", mTask.hw->i2c_num, mTask.i2c_addr);

  if (0 != (ret = sensorDriverGetConvert(mTask.hw->direction, &mTask.cvt))) {
    osLog(LOG_ERROR, "invalid direction: %d\n", mTask.hw->direction);
  }

  osLog(LOG_ERROR, "mag map[0]:%d, map[1]:%d, map[2]:%d, sign[0]:%d, sign[1]:%d, sign[2]:%d\n\r",
        mTask.cvt.map[AXIS_X],  mTask.cvt.map[AXIS_Y],  mTask.cvt.map[AXIS_Z],
        mTask.cvt.sign[AXIS_X], mTask.cvt.sign[AXIS_Y], mTask.cvt.sign[AXIS_Z]);

  i2cMasterRequest(mTask.hw->i2c_num, I2C_SPEED);
    
  mTask.txBuf[0] = REG_PCODE;
  ret = i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr,
                       mTask.txBuf, 1,
                       mTask.rxBuf, 1,
                       NULL, NULL);

  mTask.control_8m_disable = 0;

  for (uint8_t i = 0; i < 3; i++)
  {
    ret = i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                            &mTask.deviceId, 1, NULL, NULL);
  
    if (ret >= 0 && mTask.deviceId == AF6133E_PID)
    {
      osLog(LOG_INFO, "AF6133E read product id successfully id:%x.\n", mTask.rxBuf[0]);
      goto success_out;
    }
    else
      ret = -1;
  }
  
  if (ret < 0)
  {
    ret = -1;
    osLog(LOG_ERROR, "AF6133E read id failed.id:%x\n", mTask.rxBuf[0]);
    i2cMasterRelease(mTask.hw->i2c_num);
    goto err_out;
  }

success_out:
  magSensorRegister();
  magRegisterInterruptMode(MAG_UNFIFO);
  registerMagDriverFsm(af6133eFsm, ARRAY_SIZE(af6133eFsm));
  registerMagTimerCbk(af6133eTimerCbkF);
#ifndef CFG_MAG_CALIBRATION_IN_AP
  registerMagCaliAPI(&af6133eCaliAPI);
#ifdef VTC_HUBALGOLIB_READY
  vtc_lib_init();
#endif
#endif
  mTask.mag_dev_info.layout = mTask.hw->direction;
  mTask.mag_dev_info.deviceid = AF6133E_PID;
  strncpy(mTask.mag_dev_info.libname, "vtclib", sizeof(mTask.mag_dev_info.libname));
  osLog(LOG_ERROR, "af6133e Ver:201909101000\n");
err_out:
  return ret;
}
/*------------------------------------------------------------------------------------------------*/
#ifndef CFG_OVERLAY_INIT_SUPPORT
MODULE_DECLARE(af6133e, SENS_TYPE_MAG, af6133eInit);
#else
#include "mtk_overlay_init.h"
OVERLAY_DECLARE(af6133e, OVERLAY_WORK_01, af6133eInit);
#endif
