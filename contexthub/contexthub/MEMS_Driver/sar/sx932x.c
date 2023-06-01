/* MediaTek Inc. (C) 2015. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <seos.h>
#include <util.h>
#include <plat/inc/rtc.h>
#include <sensors.h>

#include <sar.h>
#include <contexthub_core.h>
#include <cust_sar.h>
#include <sx932x.h>
#include <timer.h>

#define SAR_NAME                        "sx932x"
#define I2C_SPEED                       400000

enum SX932xState {
    STATE_CHECK_INT = CHIP_SAR_SAMPLING,
    STATE_CONVERT = CHIP_SAR_CONVERT,
    STATE_SAMPLE_DONE = CHIP_SAR_SAMPLING_DONE,
    STATE_ENABLE = CHIP_SAR_ENABLE,
    STATE_ENABLE_DONE = CHIP_SAR_ENABLE_DONE,
    STATE_DISABLE = CHIP_SAR_DISABLE,
    STATE_DISABLE_DONE = CHIP_SAR_DISABLE_DONE,
    STATE_RATECHG = CHIP_SAR_RATECHG,
    STATE_RATECHG_DONE = CHIP_SAR_RATECHG_DONE,
    STATE_INIT_DONE = CHIP_INIT_DONE,
    STATE_IDLE = CHIP_IDLE,
    STATE_RESET = CHIP_RESET,         //15

    STATE_CORE,//16
    STATE_DEALY_1,//17
    STATE_REG_INIT_1,//18
    STATE_REG_INIT_2,//19
    STATE_REG_INIT_3,//20
    STATE_REG_INIT_4,//21
    STATE_DEALY_4,//22
    STATE_SAMPLE,
};

static struct SX932xTask {
    /* txBuf for i2c operation, fill register and fill value */
    uint8_t txBuf[8];
    uint8_t rxBuf[8];
    uint8_t sarIntStatus;

    uint8_t i2c_addr;
    uint8_t chipId;
    struct sar_hw *hw;

    uint32_t timerHandle;
    /* rxBuf for i2c operation, receive rawdata */
    struct transferDataInfo dataInfo;
    struct SarData_t data;

    /* data for factory */
    struct TripleAxisDataPoint factoryData;
} mTask;
//static struct SX932xTask *sx932xDebugPoint;

static int sx932xCheckHwState(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    //osLog(LOG_ERROR, "%s, time:%lld\n", __func__, rtcGetTime());

    if (rxTransferDataInfo(&mTask.dataInfo, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize)) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "sar rx error\n");
        return -1;
    }
    mTask.txBuf[0] = SX932x_STAT0_REG;
    return i2cMasterTxRx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                         mTask.rxBuf, 1, i2cCallBack, next_state);
}

static int sx932xSample(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    //osLog(LOG_ERROR, "%s, time:%lld\n", __func__, rtcGetTime());

    mTask.sarIntStatus = mTask.rxBuf[0];
    //osLog(LOG_ERROR, "sar int status: %x\n", mTask.rxBuf[0]);

    for (uint8_t i = 0; i < 4; i++) {
        mTask.txBuf[0] = SX932x_CPSRD;
        mTask.txBuf[1] = 0x00 + i;

        i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
                NULL, 0, NULL, NULL);

        mTask.txBuf[0] = SX932x_USEMSB;
        i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
                mTask.rxBuf, 8, NULL, NULL);

        osLog(LOG_ERROR, "sar ch:%d useful data: %d avg: %d diff: %d offset: %d\n", i,
            (int16_t)((mTask.rxBuf[0] << 8) | mTask.rxBuf[1]),
            (int16_t)((mTask.rxBuf[2] << 8) | mTask.rxBuf[3]),
            (int16_t)((mTask.rxBuf[4] << 8) | mTask.rxBuf[5]),
            (int16_t)((mTask.rxBuf[6] << 8) | mTask.rxBuf[7]));
        if (i == 1) {
            mTask.factoryData.ix = (int32_t)((mTask.rxBuf[0] << 8) | mTask.rxBuf[1]); //useful value
            mTask.factoryData.iy = (int32_t)((mTask.rxBuf[2] << 8) | mTask.rxBuf[3]); //average value
            mTask.factoryData.iz = (int32_t)((mTask.rxBuf[4] << 8) | mTask.rxBuf[5]); //diff value
        }
    }

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int sx932xConvert(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    //osLog(LOG_ERROR, "%s, time:%lld\n", __func__, rtcGetTime());

    if (mTask.sarIntStatus)
        mTask.data.sarState = SAR_STATE_NEAR;
    else
        mTask.data.sarState = SAR_STATE_FAR;

    mTask.data.sarData[0] = mTask.sarIntStatus;
    mTask.data.sensType = SENS_TYPE_SAR;

    txTransferDataInfo(&mTask.dataInfo, 1, &mTask.data);
    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}


static int sx932xEnable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    //osLog(LOG_ERROR, "%s, time:%lld\n", __func__, rtcGetTime());

    mTask.txBuf[0] = SX932x_CTRL1_REG;
    mTask.txBuf[1] = 0x20 | 0x07;//Enable phase0/phase1/phase2
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
        i2cCallBack, next_state);
}

static int sx932xDisable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    //osLog(LOG_ERROR, "%s, time:%lld\n", __func__, rtcGetTime());

    mTask.txBuf[0] = SX932x_CTRL1_REG;
    mTask.txBuf[1] = 0x20 & (~0x07);//Disable all phases
    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
        i2cCallBack, next_state);
}

static int sx932xRate(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    //osLog(LOG_ERROR, "%s, time:%lld\n", __func__, rtcGetTime());

    int ret = 0;
    struct SarCntlPacket cntlPacket = {0};

    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "sx932xRate, rx inSize and elemSize error\n");
        return -1;
    }

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int sx932xReset(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    //osLog(LOG_ERROR, "%s, time:%lld\n", __func__, rtcGetTime());

    mTask.txBuf[0] = SX932x_SOFTRESET_REG;
    mTask.txBuf[1] = SX932x_SOFTRESET;

    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
        i2cCallBack, next_state);
}
static int sx932xInit_1(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    //osLog(LOG_ERROR, "%s, time:%lld\n", __func__, rtcGetTime());

    mTask.txBuf[0] = SX932x_IRQ_ENABLE_REG;
    mTask.txBuf[1] = 0x60;//REG-0X05 Enable FAR/CLOSE interrupt
    mTask.txBuf[2] = 0x00;//REG-0X06
    mTask.txBuf[3] = 0x00;//REG-0X07
    mTask.txBuf[4] = 0x00;//REG-0X08

    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 5,
        NULL, NULL);

	mTask.txBuf[0] = SX932x_CTRL0_REG;
    mTask.txBuf[1] = 0x16;//REG-0X10

    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
        NULL, NULL);

	mTask.txBuf[0] = SX932x_CLKSPRD;
    mTask.txBuf[1] = 0x00;//REG-0X15
	return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 2,
        i2cCallBack, next_state);
	
}

static int sx932xInit_2(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    //osLog(LOG_ERROR, "%s, time:%lld\n", __func__, rtcGetTime());

    mTask.txBuf[0] = SX932x_AFE_CTRL0_REG;
    mTask.txBuf[1] = 0x80;//REG-0X20 //Compensation resistor
    mTask.txBuf[2] = 0x10;//REG-0X21 //Reserved
    mTask.txBuf[3] = 0x00;//REG-0X22 //Reserved
    mTask.txBuf[4] = 0x00;//REG-0X23 //Range-phase0/1,0-small, 1-big
    mTask.txBuf[5] = 0x47;//REG-0X24 //Frequency/83khz
    mTask.txBuf[6] = 0x00;//REG-0X25 //Reserved
    mTask.txBuf[7] = 0x01;//REG-0X26 //Range-phase2/3,0-small, 1-big

    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 8,
        NULL, NULL);
	mTask.txBuf[0] = SX932x_AFE_CTRL7_REG;
    mTask.txBuf[1] = 0x47;//REG-0X27 //Frequency/83khz
    mTask.txBuf[2] = 0x04;//REG-0X28 //Phase0 config/CS1-input,other-HZ
    mTask.txBuf[3] = 0x01;//REG-0X29 //Phase1 config/CS0-input,other-HZ
    mTask.txBuf[4] = 0x10;//REG-0X2A //Phase2 config/CS2-input,other-HZ
    mTask.txBuf[5] = 0x00;//REG-0X2B //Phase3 config/Not used - all HZ
    mTask.txBuf[6] = 0x12;//REG-0X2C //input resistor
    mTask.txBuf[7] = 0x08;//REG-0X2D //Analog gain/1x

    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 8,
        i2cCallBack, next_state);
}

static int sx932xInit_3(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    //osLog(LOG_ERROR, "%s, time:%lld\n", __func__, rtcGetTime());

    mTask.txBuf[0] = SX932x_PROX_CTRL0_REG;
    mTask.txBuf[1] = 0x0B;//REG-0X30/Phase0/1-Digital-Gain
    mTask.txBuf[2] = 0x0B;//REG-0X31/Phase2/3-Digital-Gain
    mTask.txBuf[3] = 0x20;//REG-0X32
    mTask.txBuf[4] = 0x20;//REG-0X33
    mTask.txBuf[5] = 0x0C;//REG-0X34/Avg-NegFilter-PosFilter
    mTask.txBuf[6] = 0x10;//REG-0X35/HYST

    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 7,
        NULL, NULL);

	mTask.txBuf[0] = SX932x_PROX_CTRL6_REG;
    mTask.txBuf[1] = 0x1B;//REG-0X30
    mTask.txBuf[2] = 0x1B;//REG-0X31

    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 3,
        i2cCallBack, next_state);
}

static int sx932xInit_4(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    //osLog(LOG_ERROR, "%s, time:%lld\n", __func__, rtcGetTime());

    mTask.txBuf[0] = SX932x_ADV_CTRL0_REG;
    mTask.txBuf[1] = 0x00;//REG-0X40
	mTask.txBuf[2] = 0x00;//REG-0X41
	mTask.txBuf[3] = 0x00;//REG-0X42/Reference setting
	mTask.txBuf[4] = 0x00;//REG-0X43/Reference parameter - phase0/1
	mTask.txBuf[5] = 0x00;//REG-0X44/Reference parameter - phase2/3

    i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 6,
            NULL, NULL);

    mTask.txBuf[6] = SX932x_CTRL1_REG;
    mTask.txBuf[7] = 0x27;

    return i2cMasterTx(mTask.hw->i2c_num, mTask.i2c_addr, &mTask.txBuf[6], 2,
        i2cCallBack, next_state);
}

static int sx932xDelay(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
    void *inBuf, uint8_t inSize, uint8_t elemInSize,
    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    uint64_t timerDelay = 1000000000ull / 100;
   // osLog(LOG_ERROR, "%s, time:%lld\n", __func__, rtcGetTime());

    if (mTask.timerHandle)
        timTimerCancel(mTask.timerHandle);
    mTask.timerHandle = timTimerSet(timerDelay, 0, 50, delayCallback, next_state, true);
    if (!mTask.timerHandle)
        configASSERT(0);

    return 0;
}

static void sarGetSensorInfo(struct sensorInfo_t *data)
{
    strncpy(data->name, SAR_NAME, sizeof(data->name));
}

static void sarGetData(void *sample) {
    struct TripleAxisDataPoint *tripleSample = (struct TripleAxisDataPoint *)sample;
    tripleSample->ix = mTask.factoryData.ix;
    tripleSample->iy = mTask.factoryData.iy;
    tripleSample->iz = mTask.factoryData.iz;
}

static int sx932xRegisterCore(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    //osLog(LOG_ERROR, "%s, time:%lld\n", __func__, rtcGetTime());

    struct sensorCoreInfo mInfo;
    memset(&mInfo, 0x00, sizeof(struct sensorCoreInfo));
    /* Register sensor Core */
    mInfo.sensType = SENS_TYPE_SAR;
    mInfo.getData = sarGetData;
    mInfo.getSensorInfo = sarGetSensorInfo;
    sensorCoreRegister(&mInfo);

    sensorFsmEnqueueFakeI2cEvt(i2cCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static struct sensorFsm sx932xFsm[] = {
    sensorFsmCmd(STATE_CHECK_INT, STATE_SAMPLE, sx932xCheckHwState),
    sensorFsmCmd(STATE_SAMPLE, STATE_CONVERT, sx932xSample),
    sensorFsmCmd(STATE_CONVERT, STATE_SAMPLE_DONE, sx932xConvert),

    sensorFsmCmd(STATE_ENABLE, STATE_ENABLE_DONE, sx932xEnable),
    sensorFsmCmd(STATE_DISABLE, STATE_DISABLE_DONE, sx932xDisable),

    sensorFsmCmd(STATE_RATECHG, STATE_RATECHG_DONE, sx932xRate),

    sensorFsmCmd(STATE_RESET, STATE_DEALY_1, sx932xReset),
    sensorFsmCmd(STATE_DEALY_1, STATE_REG_INIT_1, sx932xDelay),
    sensorFsmCmd(STATE_REG_INIT_1, STATE_REG_INIT_2, sx932xInit_1),
    sensorFsmCmd(STATE_REG_INIT_2, STATE_REG_INIT_3, sx932xInit_2),
    sensorFsmCmd(STATE_REG_INIT_3, STATE_REG_INIT_4, sx932xInit_3),
    sensorFsmCmd(STATE_REG_INIT_4, STATE_DEALY_4, sx932xInit_4),
    sensorFsmCmd(STATE_DEALY_4, STATE_CORE, sx932xDelay),
    sensorFsmCmd(STATE_CORE, STATE_INIT_DONE, sx932xRegisterCore),
};





static int sx932xInit(void)
{
    int ret = 0;

    mTask.hw = get_cust_sar("sx932x");
    if (NULL == mTask.hw) {
        osLog(LOG_ERROR, "sx932x get_cust_sar fail\n");
        ret = -1;
        goto err_out;
    }
    mTask.i2c_addr = mTask.hw->i2c_addr[0];
    osLog(LOG_ERROR, "sar i2c_num: %d, i2c_addr: 0x%x\n", mTask.hw->i2c_num, mTask.i2c_addr);

    i2cMasterRequest(mTask.hw->i2c_num, I2C_SPEED);
    mTask.txBuf[0] = SX932x_WHOAMI_REG;


    for (uint8_t i = 0; i < 3; i++) {
        ret = i2cMasterTxRxSync(mTask.hw->i2c_num, mTask.i2c_addr, mTask.txBuf, 1,
            &mTask.chipId, 1, NULL, NULL);

        if (ret >= 0 && mTask.chipId == SX932x_WHOAMI_VALUE) {
            osLog(LOG_INFO, "sx932x auto detect success %x\n", mTask.chipId);
            goto success_out;
        } else
            ret = -1;
    }

    if (ret < 0) {
        ret = -1;
        osLog(LOG_INFO, "sx932x id fail: %x\n", mTask.chipId);
        i2cMasterRelease(mTask.hw->i2c_num);
        goto err_out;
    }

success_out:
    sarSensorRegister();
    registerSarDriverFsm(sx932xFsm, ARRAY_SIZE(sx932xFsm));
err_out:
    return ret;
}

#ifndef CFG_OVERLAY_INIT_SUPPORT
MODULE_DECLARE(sx932x, SENS_TYPE_SAR, sx932xInit);
#else
#include "mtk_overlay_init.h"
OVERLAY_DECLARE(sx932x, OVERLAY_ID_SAR, sx932xInit);
#endif
