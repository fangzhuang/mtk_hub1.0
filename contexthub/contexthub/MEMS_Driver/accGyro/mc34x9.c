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

#include <algos/time_sync.h>
#include <atomic.h>
#include <cpu/inc/cpuMath.h>
#include <gpio.h>
#include <heap.h>
#include <hostIntf.h>
/* #include <isr.h> */
#include <nanohub_math.h>
#include <nanohubPacket.h>
/* #include <plat/inc/exti.h> */
/* #include <plat/inc/gpio.h> */
/* #include <plat/inc/syscfg.h> */
#include <plat/inc/rtc.h>
#include <sensors.h>
#include <seos.h>
#include <slab.h>
#include <spi.h>
#include <plat/inc/spichre.h>
#include <spichre-plat.h>
#include <timer.h>
/* #include <variant/inc/sensType.h> */
#include <variant/inc/variant.h>
#include <util.h>
#include <accGyro.h>
#include <cust_accGyro.h>
#include "hwsen.h"
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <contexthub_core.h>
#include "eint.h"
#include <performance.h>
#include <API_sensor_calibration.h>


/***********************************************
 *** REGISTER MAP FOR 36XX
 ***********************************************/
#define MC34X9_REG_XOUT_8BIT        0x00
#define MC34X9_REG_YOUT_8BIT        0x01
#define MC34X9_REG_ZOUT_8BIT        0x02
#define MC34X9_REG_STATUS_1         0x03
#define MC34X9_REG_INTR_S           0x04
#define MC34X9_REG_DEV_S            0x05
#define MC34X9_REG_INTR_EN          0x06
#define MC34X9_REG_MODE_C           0x07
#define MC34X9_REG_ODR              0x08
#define MC34X9_REG_MOTION_C         0x09
#define MC34X9_REG_FIFO_S           0x0A
#define MC34X9_REG_FIFO_RP          0x0B
#define MC34X9_REG_FIFO_WP          0x0C
#define MC34X9_REG_XOUT_LSB         0x0D
#define MC34X9_REG_XOUT_MSB         0x0E
#define MC34X9_REG_YOUT_LSB         0x0F
#define MC34X9_REG_YOUT_MSB         0x10
#define MC34X9_REG_ZOUT_LSB         0x11
#define MC34X9_REG_ZOUT_MSB         0x12

#define MC34X9_REG_CHIP_ID          0x18
#define MC34X9_REG_RESET            0x1C

#define MC34X9_REG_RANGE_LPF        0x20
#define MC34X9_REG_FIFO_C           0x2D
#define MC34X9_REG_THR              0x2E

#define MC34X9_REG_FIFO_C_S         0x30
#define MC34X9_REG_COM_C            0x31
#define MC34X9_REG_RTIG_PAD_C       0x32
#define MC34X9_REG_INTR_C           0x33
#define MC34X9_REG_SOR              0x3B

#define MC34X9_REG_AMD_LSB          0x43
#define MC34X9_REG_AMD_MSB          0x44
#define MC34X9_REG_AMD_DB           0x45
#define MC34X9_REG_RSC              0x4B

/***********************************************
 *** MODE  REG(0X10) BIT[2:0]
 ***********************************************/
#define MC34X9_MODE_SLEEP           0x00
#define MC34X9_MODE_WAKE            0x01
#define MC34X9_MODE_STANDBY         0x03
#define MC34X9_MODE_STANDBY_RESET   0x08

/***********************************************
 *** POCDE
 ***********************************************/
#define MC34X9_WAI_VALUE                (0xA4)

/***********************************************
 *** SAMPLE IDR
 ***********************************************/
#define MC34X9_2MHZ_INR                 (0x17)//(OSR=64/RATE=7/MCLK=625k)==>1K INR
#define MC34X9_8MHZ_INR                 (0x4f)//(OSR=128/RATE=7/MCLK=1.25M)==>1K INR

/***********************************************
 *** SAMPLE RATE
 ***********************************************/
#define MC34X9_ODR_1HZ_REG_VALUE        (0x0F)
#define MC34X9_ODR_5HZ_REG_VALUE        (0x0C)
#define MC34X9_ODR_10HZ_REG_VALUE       (0x0B)
#define MC34X9_ODR_15HZ_REG_VALUE       (0x09)
#define MC34X9_ODR_25HZ_REG_VALUE       (0x08)
#define MC34X9_ODR_50HZ_REG_VALUE       (0x07)
#define MC34X9_ODR_100HZ_REG_VALUE      (0x05)
#define MC34X9_ODR_200HZ_REG_VALUE      (0x03)
#define MC34X9_ODR_500HZ_REG_VALUE      (0x01)

/***********************************************
 *** ODR RATE RELATED
 ***********************************************/
#define MC34X9_ODR_1HZ_ACCEL_STD          1
#define MC34X9_ODR_5HZ_ACCEL_STD          1
#define MC34X9_ODR_10HZ_ACCEL_STD         1
#define MC34X9_ODR_15HZ_ACCEL_STD         1
#define MC34X9_ODR_25HZ_ACCEL_STD         1
#define MC34X9_ODR_50HZ_ACCEL_STD         1
#define MC34X9_ODR_100HZ_ACCEL_STD        1
#define MC34X9_ODR_200HZ_ACCEL_STD        1
#define MC34X9_ODR_500HZ_ACCEL_STD        1

/***********************************************
 *** INT STATES
 ***********************************************/
 //reg 0x04 or 0x14
#define MC34X9_INT_AMD_S               (0x04)
#define MC34X9_INT_FIFO_S              (0x20)
#define MC34X9_INT_ACQ                 (0x80)
//reg 0x0a or 0x2f
#define MC34X9_INT_FIFO_EMPTY          (0x01)
#define MC34X9_INT_FIFO_FULL           (0x02)
#define MC34X9_INT_FIFO_THR_S          (0x04)
/***********************************************/

/* One sample of triaxial sensor is expressed on 6 byte */
#define MC34X9_ONE_SAMPLE_BYTE                6
#define MC34X9_CFG1_XYZ_FRAME_MAX             30
#define MC34XX_LRF_SUPPORT 1
#define DATANUM 3

uint8_t dataBuffer[256];

int16_t pre_data[AXES_NUM];
int16_t diff[3]={0};
int startflag=0;

#if 0
#define MC_ALOGF()                 do{printf("[mcube@f@%04d]-@%s(): entry2!\n", __LINE__, __FUNCTION__);}while(0)
#define MC_ALOGD(fmt, args...)     do{printf("[mcube@d@%04d]-@%s(): " fmt "\n", __LINE__, __FUNCTION__, ##args);}while(0)
#define MC_ALOGE(fmt, args...)     do{printf("[mcube@e@%04d]-@%s(): " fmt "\n", __LINE__, __FUNCTION__, ##args);}while(0)
// key log, keep with final code.
#define MC_ALOGK(fmt, args...)     do{printf("[mcube@k@%04d]-@%s(): " fmt "\n", __LINE__, __FUNCTION__, ##args);}while(0)
#else
#define MC_ALOGF()                 
#define MC_ALOGD(fmt, args...)     
#define MC_ALOGE(fmt, args...)     do{printf("[mcube@e@%04d]-@%s(): " fmt "\n", __LINE__, __FUNCTION__, ##args);}while(0)
// key log, keep with final code.
#define MC_ALOGK(fmt, args...)     do{printf("[mcube@k@%04d]-@%s(): " fmt "\n", __LINE__, __FUNCTION__, ##args);}while(0)
#endif

#define max(x, y)   (x > y ? x : y)
#define ABS( x ) ( ((x) < 0) ? -(x) : (x) )

#define SPI_PACKET_SIZE  30
#define SPI_BUF_SIZE    (1024 + 4)
#define SPI_WRITE_0(addr, data) spiQueueWrite(addr, data, 2)
#define SPI_WRITE_1(addr, data, delay) spiQueueWrite(addr, data, delay)
#define GET_SPI_WRITE_MACRO(_1, _2, _3, NAME, ...) NAME
#define SPI_WRITE(...) GET_SPI_WRITE_MACRO(__VA_ARGS__, SPI_WRITE_1, SPI_WRITE_0)(__VA_ARGS__)
#define SPI_READ_0(addr, size, buf) spiQueueRead(addr, size, buf, 0)
#define SPI_READ_1(addr, size, buf, delay) spiQueueRead(addr, size, buf, delay)
#define GET_SPI_READ_MACRO(_1, _2, _3, _4, NAME, ...) NAME
#define SPI_READ(...) GET_SPI_READ_MACRO(__VA_ARGS__, SPI_READ_1, SPI_READ_0)(__VA_ARGS__)
#define EVT_SENSOR_ANY_MOTION       sensorGetMyEventType(SENS_TYPE_ANY_MOTION)

enum SensorEvents {
    NO_EVT = -1,
    EVT_SPI_DONE = EVT_APP_START + 1,
    EVT_SENSOR_INTERRUPT_1,
    EVT_SENSOR_INTERRUPT_2,
    EVT_TIME_SYNC,
};

static uint8_t MC34X9ImuRatesRegValue[] = {
    MC34X9_ODR_1HZ_REG_VALUE,      /* 1Hz */
    MC34X9_ODR_5HZ_REG_VALUE,     /* 5Hz */
    MC34X9_ODR_10HZ_REG_VALUE,     /* 10Hz */
    MC34X9_ODR_15HZ_REG_VALUE,     /* 15Hz */
    MC34X9_ODR_25HZ_REG_VALUE,     /* 25Hz */
    MC34X9_ODR_50HZ_REG_VALUE,    /* 50Hz */
    MC34X9_ODR_100HZ_REG_VALUE,    /* 100Hz */
    MC34X9_ODR_200HZ_REG_VALUE,    /* 200Hz */
    MC34X9_ODR_500HZ_REG_VALUE,    /* 500Hz */
};

static uint32_t MC34X9ImuRates[] = {
    SENSOR_HZ(1.0f),               /* 1Hz */
    SENSOR_HZ(5.0f),               /* 5Hz */
    SENSOR_HZ(10.0f),              /* 10Hz */
    SENSOR_HZ(15.0f),              /* 15Hz */
    SENSOR_HZ(25.0f),              /* 25Hz */
    SENSOR_HZ(50.0f),              /* 50Hz */
    SENSOR_HZ(100.0f),             /* 100Hz */
    SENSOR_HZ(200.0f),             /* 200Hz */
    SENSOR_HZ(500.0f),             /* 500Hz */
    0,
};

static uint8_t MC34X9AccelRatesSamplesToDiscard[] = {
    MC34X9_ODR_1HZ_ACCEL_STD,      /* 1Hz*/
    MC34X9_ODR_5HZ_ACCEL_STD,      /* 5Hz */
    MC34X9_ODR_10HZ_ACCEL_STD,     /* 10Hz */
    MC34X9_ODR_15HZ_ACCEL_STD,     /* 15Hz */
    MC34X9_ODR_25HZ_ACCEL_STD,     /* 25Hz */
    MC34X9_ODR_50HZ_ACCEL_STD,     /* 50Hz */
    MC34X9_ODR_100HZ_ACCEL_STD,    /* 100Hz */
    MC34X9_ODR_200HZ_ACCEL_STD,    /* 200Hz */
    MC34X9_ODR_500HZ_ACCEL_STD,    /* 200Hz */
};

static int listStkDrop[] =
{
    0,      /* 1Hz*/
    0,      /* 5Hz */
    0,     /* 10Hz */
    0,     /* 15Hz */
    0,     /* 25Hz */
    0,     /* 50Hz */
    0,    /* 100Hz */
    0,    /* 200Hz */
    0,    /* 200Hz */
};

enum MC34X9State {
    STATE_SAMPLE = CHIP_SAMPLING,
    STATE_FIFO = CHIP_FIFO,
    STATE_CONVERT = CHIP_CONVERT,
    STATE_SAMPLE_DONE = CHIP_SAMPLING_DONE,
    STATE_ACC_ENABLE = CHIP_ACC_ENABLE,
    STATE_ACC_ENABLE_DONE = CHIP_ACC_ENABLE_DONE,
    STATE_ACC_DISABLE = CHIP_ACC_DISABLE,
    STATE_ACC_DISABLE_DONE = CHIP_ACC_DISABLE_DONE,
    STATE_ACC_RATECHG = CHIP_ACC_RATECHG,
    STATE_ACC_CONFIG,
    STATE_ACC_RATECHG_DONE = CHIP_ACC_RATECHG_DONE,
    STATE_ACC_CALI = CHIP_ACC_CALI,
    STATE_ACC_CALI_DONE = CHIP_ACC_CALI_DONE,
    STATE_ACC_CFG = CHIP_ACC_CFG,
    STATE_ACC_CFG_DONE = CHIP_ACC_CFG_DONE,
    STATE_ANYMO_ENABLE = CHIP_ANYMO_ENABLE,
    STATE_ANYMO_ENABLE_DONE = CHIP_ANYMO_ENABLE_DONE,
    STATE_ANYMO_DISABLE = CHIP_ANYMO_DISABLE,
    STATE_ANYMO_DISABLE_DONE = CHIP_ANYMO_DISABLE_DONE,
    STATE_HW_INT_STATUS_CHECK = CHIP_HW_INT_STATUS_CHECK,
    STATE_HW_INT_HANDLING = CHIP_HW_INT_HANDLING,
    STATE_HW_INT_HANDLING_DONE = CHIP_HW_INT_HANDLING_DONE,
    STATE_INIT_DONE = CHIP_INIT_DONE,
    STATE_IDLE = CHIP_IDLE,
    STATE_SW_RESET = CHIP_RESET,
    MC_INITIALIZE_01,
    MC_INITIALIZE_02,
    MC_INITIALIZE_03,
    MC_INITIALIZE_04,
    MC_INITIALIZE_05,
    MC_INITIALIZE_06,
    MC_INITIALIZE_07,
    MC_INITIALIZE_08,
    MC_INITIALIZE_09,
    MC_INITIALIZE_10,
    STATE_SENSOR_REGISTRATION,
    STATE_EINT_REGISTRATION,
    STATE_ANYMO_ENABLE_01,
    STATE_ANYMO_ENABLE_02,
    STATE_ANYMO_DISABLE_01,
    STATE_ANYMO_DISABLE_02,
    CLAER_HW_INT,
    FIFO_DATA_LENGTH,
};

enum SensorIndex {
    ACC = 0,
    ANYMO,
    NUM_OF_SENSOR,
};

struct mc34x9Sensor {
    bool powered;
    bool configed;
    bool startCali;
    float staticCali[AXES_NUM];
    int32_t accuracy;
    uint32_t rate;
    uint64_t latency;
    uint32_t hwRate;  // rate set in hw
    uint8_t samplesToDiscard;
    float sensitivity;
};

static struct MC34X9Task {
    struct mc34x9Sensor sensors[NUM_OF_SENSOR];

    uint16_t watermark;
    uint32_t fifoDataToRead;
    uint64_t hwSampleTime;
    uint64_t swSampleTime;
    uint64_t lastSampleTime;
    uint8_t *regBuffer;
    uint8_t *statusBuffer;
    uint8_t *wakeupBuffer;
    uint8_t *tempStatusBuffer;
    uint8_t *tempBuffer;
    uint8_t *dummyBuffer;
    uint8_t *dataBuffer;
    float temperature;
    bool tempReady;

    SpiCbkF spiCallBack;
    struct transferDataInfo dataInfo;
    struct accGyroDataPacket accGyroPacket;
    struct accGyro_hw *hw;
    struct sensorDriverConvert cvt;
    spi_cs_t cs;
    struct SpiMode mode;
    struct SpiPacket packets[SPI_PACKET_SIZE];
    struct SpiDevice *spiDev;
    uint8_t txrxBuffer[SPI_BUF_SIZE];
    uint16_t mWbufCnt;
    uint8_t mRegCnt;
    uint8_t mRetryLeft;
    /* data for factory */
    struct TripleAxisDataPoint accFactoryData;
    int32_t accSwCali[AXES_NUM];
    int32_t debug_trace;

    int latch_time_id;
    uint16_t accSampleCntAfterPwrOn;
    int drop;
    int dropCnt;
} mTask;

static void spiQueueWrite(uint8_t addr, uint8_t data, uint32_t delay) {
    mTask.packets[mTask.mRegCnt].size = 2;
    mTask.packets[mTask.mRegCnt].txBuf = &mTask.txrxBuffer[mTask.mWbufCnt];
    mTask.packets[mTask.mRegCnt].rxBuf = &mTask.txrxBuffer[mTask.mWbufCnt];
    mTask.packets[mTask.mRegCnt].delay = delay * 1000;
    mTask.txrxBuffer[mTask.mWbufCnt++] = addr;
    mTask.txrxBuffer[mTask.mWbufCnt++] = data;
    mTask.mWbufCnt = (mTask.mWbufCnt + 3) & 0xFFFC;
    mTask.mRegCnt++;
}

static void spiQueueRead(uint8_t addr, size_t size, uint8_t **buf, uint32_t delay) {
    memset(mTask.txrxBuffer, 0, SPI_BUF_SIZE);
    *buf = &mTask.txrxBuffer[mTask.mWbufCnt];
    mTask.packets[mTask.mRegCnt].size = size + 2;  // first byte will not contain valid data
    mTask.packets[mTask.mRegCnt].txBuf = &mTask.txrxBuffer[mTask.mWbufCnt];
    mTask.packets[mTask.mRegCnt].rxBuf = *buf;
    mTask.packets[mTask.mRegCnt].delay = delay * 1000;
    mTask.txrxBuffer[mTask.mWbufCnt++] = 0x80 | addr ;
    mTask.mWbufCnt = (mTask.mWbufCnt + size + 3) & 0xFFFC;
    mTask.mRegCnt++;
}

static int spiBatchTxRx(struct SpiMode *mode,
        SpiCbkF callback, void *cookie, const char *src) {
    int err = 0;
    if (mTask.mWbufCnt > SPI_BUF_SIZE) {
        MC_ALOGD("NO enough SPI buffer space, dropping transaction.\n");
        return -1;
    }
    if (mTask.mRegCnt > SPI_PACKET_SIZE) {
        MC_ALOGD("spiBatchTxRx too many packets!\n");
        return -1;
    }
    err = spiMasterRxTx(mTask.spiDev, mTask.cs, mTask.packets, mTask.mRegCnt, mode, callback, cookie);
    mTask.mRegCnt = 0;
    mTask.mWbufCnt = 0;
    return err;
}

static void accGetCalibration(int32_t *cali, int32_t size) {
    cali[AXIS_X] = mTask.accSwCali[AXIS_X];
    cali[AXIS_Y] = mTask.accSwCali[AXIS_Y];
    cali[AXIS_Z] = mTask.accSwCali[AXIS_Z];
    MC_ALOGD("accGetCalibration cali x:%d, y:%d, z:%d\n", cali[AXIS_X], cali[AXIS_Y], cali[AXIS_Z]);
}

static void accSetCalibration(int32_t *cali, int32_t size) {
    mTask.accSwCali[AXIS_X] = cali[AXIS_X];
    mTask.accSwCali[AXIS_Y] = cali[AXIS_Y];
    mTask.accSwCali[AXIS_Z] = cali[AXIS_Z];
    MC_ALOGD("accSetCalibration cali x:%d, y:%d, z:%d\n", mTask.accSwCali[AXIS_X],
        mTask.accSwCali[AXIS_Y], mTask.accSwCali[AXIS_Z]); 
}

static void accGetData(void *sample) {
    struct TripleAxisDataPoint *tripleSample = (struct TripleAxisDataPoint *)sample;
    tripleSample->ix = mTask.accFactoryData.ix;
    tripleSample->iy = mTask.accFactoryData.iy;
    tripleSample->iz = mTask.accFactoryData.iz;
}

static int mc34x9ModeStandBy(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    MC_ALOGF();
    SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_STANDBY, 100);//standy mode W reg
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int mc34x9Reset(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    MC_ALOGF();
    SPI_WRITE(MC34X9_REG_RESET, 0x40, 10000); //software reset chip
    SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_STANDBY, 100); 
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int mc34x9SetBusSpi(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    MC_ALOGF();
    SPI_WRITE(MC34X9_REG_COM_C, 0x00, 100); //3 wire spi disable,swap INT1 INT2
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int mc34x9SPISpeed(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    MC_ALOGF();
    SPI_READ(MC34X9_REG_SOR, 1, &mTask.regBuffer,50);
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int mc34x9InitReg(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    //sample rate
    MC_ALOGE(" 0x3b---->:a:%02x,b:%02x,c:%02x,d:%02x\n", mTask.regBuffer[0],mTask.regBuffer[1],mTask.regBuffer[2],mTask.regBuffer[3]);
    if((mTask.regBuffer[2] & 0x30) == 0x10){//1.25MHz
        mTask.mode.speed = 8000000;    //8Mhz
       SPI_WRITE(MC34X9_REG_ODR,MC34X9_8MHZ_INR, 50);
    }else if((mTask.regBuffer[2] & 0x30) == 0x20){
        mTask.mode.speed = 2000000;    //2Mhz
        SPI_WRITE(MC34X9_REG_ODR,MC34X9_2MHZ_INR, 50);
    }else{
        MC_ALOGD("mc34x9 set spi speed failded\n");
    }
    SPI_WRITE(MC34X9_REG_FIFO_C_S,0x87, 50);//burst mode 50Hz odr

    //set anymotion
    SPI_WRITE(MC34X9_REG_MOTION_C,0x04, 50);//set amd Threshold LSB
    SPI_WRITE(MC34X9_REG_AMD_LSB,0x50, 50);//set amd Threshold LSB
    SPI_WRITE(MC34X9_REG_AMD_MSB,0x00, 50);//set amd Threshold MSB
    SPI_WRITE(MC34X9_REG_AMD_DB,0xFF, 50);//set amd Debounce
    //set rang/resolution/LFP
    SPI_WRITE(MC34X9_REG_RANGE_LPF,  0x2B, 100);//+/-8g/16bit/LPF 333
    //set int
    SPI_WRITE(MC34X9_REG_RTIG_PAD_C, 0x00, 100);//PAD[1,2] for INT
    SPI_WRITE(MC34X9_REG_INTR_C, 0xCC, 100);//push-pull/active high[int1,int2]
    //SPI_WRITE(MC34X9_REG_INTR_EN, 0x84, 100);//ACQ/AMD enable
    /*set fifo mode*/
    SPI_WRITE(MC34X9_REG_FIFO_C, 0x10, 200);//fifo reset
    SPI_WRITE(MC34X9_REG_FIFO_C, 0x6C, 50);//threshold,enbale,All interrupts are routed to INT1
    //SPI_WRITE(MC34X9_REG_FIFO_C_S, 0x80, 50);//burst mode
    SPI_WRITE(MC34X9_REG_THR,0x01, 50);//set waternark=1 and enable fifo
    SPI_WRITE(MC34X9_REG_RSC,0x01, 50);//set waternark=1 and enable fifo
    //wake
    //SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_WAKE, 100); //wake mode

    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static void mc34x9SetDebugTrace(int32_t trace) {
    mTask.debug_trace = trace;
    MC_ALOGD("%s ==> trace:%d\n", __func__, mTask.debug_trace);
}

static void mc34x9GetSensorInfo(struct sensorInfo_t *data)
{
    strncpy(data->name, "mc34x9", sizeof(data->name));
}

static void sensorCoreRegistration(void) {
    struct sensorCoreInfo mInfo;
    MC_ALOGD("mc34x9RegisterCore\n");
    memset(&mInfo, 0x00, sizeof(struct sensorCoreInfo));
    /* Register sensor Core */
    mInfo.sensType = SENS_TYPE_ACCEL;
    mInfo.gain = GRAVITY_EARTH_1000;
    mInfo.sensitivity = mTask.sensors[ACC].sensitivity;
    mInfo.cvt = mTask.cvt;
    mInfo.getCalibration = accGetCalibration;
    mInfo.setCalibration = accSetCalibration;
    mInfo.getData = accGetData;
    mInfo.setDebugTrace = mc34x9SetDebugTrace;
    mInfo.getSensorInfo = mc34x9GetSensorInfo;
    sensorCoreRegister(&mInfo);
}

static int mc34x9SensorRegistration(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    MC_ALOGF();
    sensorCoreRegistration();
    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static void mc34x9Isr1(int arg) {
    MC_ALOGD("mc34x9Isr1,AA=%x\n", mTask.latch_time_id);
    if (mTask.latch_time_id < 0) {
        mTask.hwSampleTime = rtcGetTime();
        MC_ALOGE("mc34x9Isr1, real=%lld\n", mTask.hwSampleTime);
    } else {
        mTask.hwSampleTime = get_latch_time_timestamp(mTask.latch_time_id);
        MC_ALOGD("mc34x9Isr1, fake=%lld, real=%lld\n",
            rtcGetTime(), mTask.hwSampleTime);
    }
    accGyroHwIntCheckStatus();
}

static int mc34x9EintRegistration(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    MC_ALOGF();
    mt_eint_dis_hw_debounce(mTask.hw->eint_num);
    mt_eint_registration(mTask.hw->eint_num, LEVEL_SENSITIVE, HIGH_LEVEL_TRIGGER, mc34x9Isr1, EINT_INT_UNMASK,
        EINT_INT_AUTO_UNMASK_OFF);
    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int mc34x9CalcuOdr(uint32_t *rate, uint32_t *report_rate) {
    int i;
    MC_ALOGF();
    for (i = 0; i < (ARRAY_SIZE(MC34X9ImuRates) - 1); i++) {
        if (*rate <= MC34X9ImuRates[i]) {
            *report_rate = MC34X9ImuRates[i];
            break;
        }
    }
    if (*rate > MC34X9ImuRates[(ARRAY_SIZE(MC34X9ImuRates) - 2)]) {
        i = (ARRAY_SIZE(MC34X9ImuRates) - 2);
        *report_rate = MC34X9ImuRates[i];
    }
    if (i == (ARRAY_SIZE(MC34X9ImuRates) - 1 )) {
        MC_ALOGE("ODR not valid! Selected smallest ODR available\n");
        return -1;
    }
    return i;
}

static uint16_t mc34x9CalcuWm(void)
{
    uint64_t min_latency = SENSOR_LATENCY_NODATA;
    uint8_t min_watermark = 1;
    uint8_t max_watermark = MAX_RECV_PACKET;
    uint16_t watermark = 0;
    uint32_t temp_delay = 0, temp_cnt = 0, total_cnt = 0;
    MC_ALOGF();
    if (mTask.sensors[ACC].powered && (SENSOR_LATENCY_NODATA != mTask.sensors[ACC].latency))
    {
        min_latency =
            (SENSOR_LATENCY_NODATA > mTask.sensors[ACC].latency) ? mTask.sensors[ACC].latency : SENSOR_LATENCY_NODATA;
    }

    /* If acc OFF, or latency = SENSOR_LATENCY_NODATA, watermark = 0, 1, or 2 */
    if (SENSOR_LATENCY_NODATA == min_latency)
    {
        watermark = 1;
    }
    else
    {
        if (mTask.sensors[ACC].powered)
        {
            temp_delay = (1000000000ULL / mTask.sensors[ACC].rate) << 10;
            temp_cnt = min_latency / temp_delay;
            min_watermark = mTask.sensors[ACC].rate / SENSOR_HZ(400.0f);
            total_cnt = (temp_cnt > min_watermark) ? temp_cnt : min_watermark;
            MC_ALOGD("%s: delay=%d, latency:%lld, min_wm=%d, total_cnt=%d\n",
                __func__, temp_delay, min_latency, min_watermark, total_cnt);
        }
        watermark = total_cnt;
        watermark = (watermark < min_watermark) ? min_watermark : watermark;
        watermark = (watermark > max_watermark) ? max_watermark : watermark;
    }

    watermark = (MC34X9_CFG1_XYZ_FRAME_MAX < watermark) ? MC34X9_CFG1_XYZ_FRAME_MAX : watermark;
    MC_ALOGD("%s: real watermark=%d \n", __func__, watermark);

    return watermark;
}

static void mc34x9ConfigFifo(bool odr_change) {
    uint8_t regValue;
    uint16_t watermarkReg;
    watermarkReg = mTask.watermark;
    regValue = *((uint8_t *)&watermarkReg);
    MC_ALOGF();
    SPI_WRITE(MC34X9_REG_INTR_S, 0x00, 100);//clear INT
    SPI_WRITE(MC34X9_REG_FIFO_C, 0x10, 500);//fifo reset
    SPI_WRITE(MC34X9_REG_FIFO_C, 0x6C, 100);//threshold,enbale,All interrupts are routed to INT1
    SPI_WRITE(MC34X9_REG_THR,regValue, 100);//set waternark=1 and enable fifo
    SPI_WRITE(MC34X9_REG_RSC,regValue, 100);//set burst count

    MC_ALOGD("mc34x9ConfigFifo watermark:%d\n", regValue);
}

static int mc34x9AccRate(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    int ret = 0;
    int odr = 0;
    uint8_t regValue = 0x00;
    //uint8_t watermark = 0;
    uint32_t sampleRate = 0;
    bool accelOdrChanged = false;
    struct accGyroCntlPacket cntlPacket;
    MC_ALOGF();
    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
        MC_ALOGE("mc34x9AccRate, rx inSize and elemSize error\n");
        return -1;
    }
    mTask.sensors[ACC].rate = cntlPacket.rate;
    mTask.sensors[ACC].latency = cntlPacket.latency;
    mTask.watermark = cntlPacket.waterMark;
    mTask.accSampleCntAfterPwrOn = 0;

    MC_ALOGD(" acc rate:%d, latency:%lld, watermark:%d\n",
        mTask.sensors[ACC].rate, mTask.sensors[ACC].latency,  mTask.watermark);

    odr = mc34x9CalcuOdr(&mTask.sensors[ACC].rate, &sampleRate);
    mTask.drop = listStkDrop[odr];
    mTask.dropCnt = 0;
    if (odr < 0) {
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
        MC_ALOGE("mc34x9AccRate, calcu odr error\n");
        return -1;
    }
    mTask.drop = listStkDrop[odr];
    mTask.dropCnt = 0;

    MC_ALOGD("mc34x9AccRate acc rate:%d, latency:%lld, watermark:%d,odr=%d,sampleRate=%d,cntlPacket.rate=%d\n",mTask.sensors[ACC].rate, mTask.sensors[ACC].latency,  mTask.watermark, odr,sampleRate,cntlPacket.rate);

    if ((sampleRate != mTask.sensors[ACC].hwRate))
    {
        mTask.sensors[ACC].hwRate = sampleRate;
        regValue = 0x80 | MC34X9ImuRatesRegValue[odr];//busrt mode

        mTask.sensors[ACC].samplesToDiscard = MC34X9AccelRatesSamplesToDiscard[odr];

        MC_ALOGD("acc odr:%d,mTask.sampleRate=%d,drop=%d,dropCnt=%d,cntlPacket.rate=%d,regValue=0x%x\n",  odr, mTask.sensors[ACC].hwRate, mTask.drop, mTask.dropCnt, cntlPacket.rate,regValue);
        SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_STANDBY, 50); //standy
        SPI_WRITE(MC34X9_REG_FIFO_C_S,  regValue, 50);
        //workaround only for interrupt delay 1s
        SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_STANDBY | MC34X9_MODE_STANDBY_RESET, 50); //standy&standy_reset
        SPI_WRITE(MC34X9_REG_INTR_S, 0x00, 50);//clear INT
        SPI_WRITE(MC34X9_REG_FIFO_C,0x00, 50); //disable fifo,threshold int and watermark mode
        SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_WAKE, 50*1000); //delay 11ms
        SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_STANDBY, 50); //standy
        accelOdrChanged = true;
    } else {
        SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_STANDBY, 50); //standy
        accelOdrChanged = false;
    }

    registerAccGyroFifoInfo((mTask.sensors[ACC].hwRate == 0) ? 0 : 1024000000000 / mTask.sensors[ACC].hwRate, 0);
    mTask.sensors[ACC].configed = true;

    mTask.watermark = mc34x9CalcuWm();
    mc34x9ConfigFifo(accelOdrChanged);

    startflag=1;//workaround for first data change odr or fifo

    MC_ALOGE("acc rate:%d, latency:%lld, watermark:%d, regValue=0x%02x\n",
    mTask.sensors[ACC].rate, mTask.sensors[ACC].latency,  mTask.watermark,regValue);
   
    SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_WAKE, 50); //wake mode
   

    mt_eint_unmask(mTask.hw->eint_num);
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int mc34x9IntStatusCheck_01(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                            void *inBuf, uint8_t inSize, uint8_t elemInSize,
                            void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    MC_ALOGF();
    SPI_READ(MC34X9_REG_INTR_S, 1, &mTask.statusBuffer);
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int mc34x9FsmCheckFifo(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack,
        void *next_state, void *inBuf, uint8_t inSize, uint8_t elemInSize,
        void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    union EmbeddedDataPoint trigger_axies;
    MC_ALOGD("interrupt Power any :%d, acc:%d,int_status:0x%02x\n",
        mTask.sensors[ANYMO].powered, mTask.sensors[ACC].powered,mTask.statusBuffer[2]);
    if (mTask.sensors[ANYMO].powered){
         if(MC34X9_INT_AMD_S & mTask.statusBuffer[2]){
            trigger_axies.idata = 0x80;
            MC_ALOGE("Detected any motion,INT_WAKE=0x%x,INT_WAKE=0x%0x\n",(MC34X9_INT_AMD_S & mTask.statusBuffer[2]),(MC34X9_INT_FIFO_S & mTask.statusBuffer[2]));
            // as other vendor do , we do not enable software mask here
            osEnqueueEvt(EVT_SENSOR_ANY_MOTION, trigger_axies.vptr, NULL);
       }
   }
	//becuse only fifo threshold,fifo int status==fifo threshold status
   if ((MC34X9_INT_FIFO_S & mTask.statusBuffer[2]) && (mTask.sensors[ACC].powered == true))
   {
        MC_ALOGD("Detected FIFO int \n");
        accGyroInterruptOccur();
   }else{
        /* If not enter accGyroInterruptOccur by INT_FIFO_WM, need unmask eint here */
        mt_eint_unmask(mTask.hw->eint_num); // for other interrupt		
   }

    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int mc34x9clearint(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    MC_ALOGF();
    memcpy(dataBuffer,&mTask.dataBuffer[2],mTask.fifoDataToRead);
    SPI_WRITE(MC34X9_REG_INTR_S, 0x00, 100);
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static void spiIsrCallBack(void *cookie, int err) {
    if (err != 0) {
        MC_ALOGE("mc34x9: spiIsrCallBack err\n");
        sensorFsmEnqueueFakeSpiEvt(mTask.spiCallBack, cookie, ERROR_EVT);
    } else {
        mTask.swSampleTime = rtcGetTime();
        sensorFsmEnqueueFakeSpiEvt(mTask.spiCallBack, cookie, SUCCESS_EVT);
    }
}

static int mc34x9Sample(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    int ret = 0;
    MC_ALOGF();

    ret = rxTransferDataInfo(&mTask.dataInfo, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
        MC_ALOGE("mc34x9Sample, rx dataInfo error\n");
        return -1;
    }

    SPI_READ(MC34X9_REG_FIFO_S, 1, &mTask.statusBuffer);

    mTask.spiCallBack = spiCallBack;
    return spiBatchTxRx(&mTask.mode, spiIsrCallBack, next_state, __FUNCTION__);
}

static int mc34x9ReadFifo(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    MC_ALOGD("MC34X9_REG_INTR_S=status:0x%02x\n",mTask.statusBuffer[2]);
    mTask.fifoDataToRead = mTask.watermark * MC34X9_ONE_SAMPLE_BYTE;
    SPI_READ(MC34X9_REG_XOUT_LSB, mTask.fifoDataToRead, &mTask.dataBuffer);

    MC_ALOGD("mc34x9ReadFifo1 length:%d\n", mTask.fifoDataToRead);

    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static void mc34x9ParseRawData(struct accGyroData *data, uint8_t *buf, uint8_t sensorType) {
    int16_t raw_data[AXES_NUM] = {0};
    int16_t remap_data[AXES_NUM] = {0};
    int32_t SwCali[AXES_NUM] = {0};
    int32_t caliResult[AXES_NUM] = {0};
    float temp_data[AXES_NUM] = {0};
    float calibrated_data_output[AXES_NUM] = {0};
    int32_t delta_time = 0;
    int16_t status = 0;

    if (sensorType == SENS_TYPE_ACCEL)
        accGetCalibration(SwCali, 0);
    raw_data[AXIS_X] = (int16_t)(buf[0] | (buf[1] << 8));
    raw_data[AXIS_Y] = (int16_t)(buf[2] | (buf[3] << 8));
    raw_data[AXIS_Z] = (int16_t)(buf[4] | (buf[5] << 8));

    raw_data[AXIS_X] = raw_data[AXIS_X] + SwCali[AXIS_X];
    raw_data[AXIS_Y] = raw_data[AXIS_Y] + SwCali[AXIS_Y];
    raw_data[AXIS_Z] = raw_data[AXIS_Z] + SwCali[AXIS_Z];

    MC_ALOGD("ACCEL:raw_data_x=%d, raw_data_y=%d, raw_data_z=%d,pwrcnt=%d\n",
        raw_data[0], raw_data[1], raw_data[2],mTask.accSampleCntAfterPwrOn);

    if((raw_data[AXIS_X] == 0)&&(raw_data[AXIS_Y] == 0)&&(raw_data[AXIS_Z] == 0)){
        raw_data[AXIS_X]=pre_data[AXIS_X] + pre_data[AXIS_X]%10;
        raw_data[AXIS_Y]=pre_data[AXIS_Y] + pre_data[AXIS_X]%10;
        raw_data[AXIS_Z]=pre_data[AXIS_Z] + pre_data[AXIS_X]%10;
    }

    if((ABS(raw_data[0]) < 2000) && (ABS(raw_data[1]) < 2000) && (ABS(raw_data[2]) < 2000)){
    MC_ALOGE(" error accel:raw_data_x=%d, raw_data_y=%d, raw_data_z=%d,pwrcnt=%d\n",
        raw_data[0], raw_data[1], raw_data[2],mTask.accSampleCntAfterPwrOn);
    }

#if MC34XX_LRF_SUPPORT
    if(mTask.sensors[ACC].hwRate <= SENSOR_HZ(500.0f)){
        if(startflag==1){
            startflag=0;
            pre_data[AXIS_X]=raw_data[AXIS_X];
            pre_data[AXIS_Y]=raw_data[AXIS_Y];
            pre_data[AXIS_Z]=raw_data[AXIS_Z];
        }else{
            diff[0] = raw_data[AXIS_X] - pre_data[AXIS_X];
            diff[1] = raw_data[AXIS_Y] - pre_data[AXIS_Y];
            diff[2] = raw_data[AXIS_Z] - pre_data[AXIS_Z];
            
            if(ABS(diff[0]) < 100){
                raw_data[AXIS_X] = pre_data[AXIS_X] + raw_data[AXIS_X]%10;
            }else{
                pre_data[AXIS_X] = raw_data[AXIS_X];
            }
            if(ABS(diff[1]) < 100){
                raw_data[AXIS_Y] = pre_data[AXIS_Y] + raw_data[AXIS_X]%10;
            }else{
                pre_data[AXIS_Y] = raw_data[AXIS_Y];
            }
            if(ABS(diff[2]) < 100){
                raw_data[AXIS_Z] = pre_data[AXIS_Z] + raw_data[AXIS_X]%10;
            }else{
                pre_data[AXIS_Z] = raw_data[AXIS_Z];
            } 
       }
    }
#endif

    remap_data[mTask.cvt.map[AXIS_X]] = mTask.cvt.sign[AXIS_X] * raw_data[AXIS_X];
    remap_data[mTask.cvt.map[AXIS_Y]] = mTask.cvt.sign[AXIS_Y] * raw_data[AXIS_Y];
    remap_data[mTask.cvt.map[AXIS_Z]] = mTask.cvt.sign[AXIS_Z] * raw_data[AXIS_Z];

    MC_ALOGD("ACCEL:remap_data_x=%d, remap_data_y=%d, remap_data_z=%d\n",
        remap_data[0], remap_data[1], remap_data[2]);

    if (sensorType == SENS_TYPE_ACCEL) {
        temp_data[AXIS_X] = (float)remap_data[AXIS_X] * GRAVITY_EARTH_SCALAR / mTask.sensors[ACC].sensitivity;
        temp_data[AXIS_Y] = (float)remap_data[AXIS_Y] * GRAVITY_EARTH_SCALAR / mTask.sensors[ACC].sensitivity;
        temp_data[AXIS_Z] = (float)remap_data[AXIS_Z] * GRAVITY_EARTH_SCALAR / mTask.sensors[ACC].sensitivity;

        if (UNLIKELY(mTask.sensors[ACC].startCali)) {
            status = Acc_run_factory_calibration_timeout(delta_time,
                temp_data, calibrated_data_output, (int *)&mTask.sensors[ACC].accuracy, rtcGetTime());
            if (status != 0) {
                mTask.sensors[ACC].startCali = false;
                if (status > 0) {
                    MC_ALOGD("ACC cali detect shake\n");
                    caliResult[AXIS_X] = (int32_t)(mTask.sensors[ACC].staticCali[AXIS_X] * 1000);
                    caliResult[AXIS_Y] = (int32_t)(mTask.sensors[ACC].staticCali[AXIS_Y] * 1000);
                    caliResult[AXIS_Z] = (int32_t)(mTask.sensors[ACC].staticCali[AXIS_Z] * 1000);
                    accGyroSendCalibrationResult(SENS_TYPE_ACCEL, (int32_t *)&caliResult[0], (uint8_t)status);
                } else
                    MC_ALOGD("ACC cali time out\n");
            } else if (mTask.sensors[ACC].accuracy == 3) {
                mTask.sensors[ACC].startCali = false;
                mTask.sensors[ACC].staticCali[AXIS_X] = calibrated_data_output[AXIS_X] - temp_data[AXIS_X];
                mTask.sensors[ACC].staticCali[AXIS_Y] = calibrated_data_output[AXIS_Y] - temp_data[AXIS_Y];
                mTask.sensors[ACC].staticCali[AXIS_Z] = calibrated_data_output[AXIS_Z] - temp_data[AXIS_Z];
                caliResult[AXIS_X] = (int32_t)(mTask.sensors[ACC].staticCali[AXIS_X] * 1000);
                caliResult[AXIS_Y] = (int32_t)(mTask.sensors[ACC].staticCali[AXIS_Y] * 1000);
                caliResult[AXIS_Z] = (int32_t)(mTask.sensors[ACC].staticCali[AXIS_Z] * 1000);
                accGyroSendCalibrationResult(SENS_TYPE_ACCEL, (int32_t *)&caliResult[0], (uint8_t)status);
                MC_ALOGD("ACC cali done:caliResult[0]:%d, caliResult[1]:%d, caliResult[2]:%d, offset[0]:%f, offset[1]:%f, offset[2]:%f\n",
                    caliResult[AXIS_X], caliResult[AXIS_Y], caliResult[AXIS_Z],
                    (double)mTask.sensors[ACC].staticCali[AXIS_X],
                    (double)mTask.sensors[ACC].staticCali[AXIS_Y],
                    (double)mTask.sensors[ACC].staticCali[AXIS_Z]);
            }
        }

        data->sensType = sensorType;

        data->x = temp_data[AXIS_X] + mTask.sensors[ACC].staticCali[AXIS_X];
        data->y = temp_data[AXIS_Y] + mTask.sensors[ACC].staticCali[AXIS_Y];
        data->z = temp_data[AXIS_Z] + mTask.sensors[ACC].staticCali[AXIS_Z];

        mTask.accFactoryData.ix = (int32_t)(data->x * ACCELEROMETER_INCREASE_NUM_AP);
        mTask.accFactoryData.iy = (int32_t)(data->y * ACCELEROMETER_INCREASE_NUM_AP);
        mTask.accFactoryData.iz = (int32_t)(data->z * ACCELEROMETER_INCREASE_NUM_AP);

        MC_ALOGD("double  ACCEL:raw_data_x=%f, raw_data_y=%f, raw_data_z=%f\n",
                    (double)data->x, (double)data->y, (double)data->z);
    }

    if (mTask.debug_trace) {
        switch (sensorType) {
            case SENS_TYPE_ACCEL:
                MC_ALOGD("ACCEL:raw_data_x=%f, raw_data_y=%f, raw_data_z=%f\n",
                    (double)data->x, (double)data->y, (double)data->z);
                break;
            default:
                break;
        }
    }
}

static int mc34x9Convert(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    struct accGyroData *data = mTask.accGyroPacket.outBuf;
    uint8_t accEventSize = 0;
    uint64_t swSampleTime = 0, realSampleTime = 0;
    uint8_t *fifo_data = dataBuffer;
    uint16_t fifo_offset = 0;//fifo raw data counter
    
    MC_ALOGF();

    mTask.fifoDataToRead = mTask.watermark  * MC34X9_ONE_SAMPLE_BYTE;
    while (fifo_offset < mTask.fifoDataToRead) {
            if (0 == mTask.drop || mTask.dropCnt >= (mTask.drop - 1))
            {
                if (accEventSize < MAX_RECV_PACKET)
                {
                    mc34x9ParseRawData(&data[accEventSize], &fifo_data[fifo_offset], SENS_TYPE_ACCEL);
                    accEventSize++;
                    mTask.dropCnt = 0;
                }
                else
                {
                    MC_ALOGE("outBuf full, accEventSize = %d\n", accEventSize);
                }
            }
            else
            {
                mTask.dropCnt++;
            }
        fifo_offset += MC34X9_ONE_SAMPLE_BYTE;
    }
    /*if startcali true , can't send to runtime cali in parseRawData to accGyro*/
    if (mTask.sensors[ACC].startCali) {
        accEventSize = 0;
    }

    swSampleTime = mTask.swSampleTime;

    realSampleTime = calcFakeInterruptTime(swSampleTime, mTask.hwSampleTime, mTask.lastSampleTime,
        mTask.sensors[ACC].hwRate, mTask.sensors[ACC].configed, accEventSize, 0, 0, 0);
    {
           MC_ALOGD(" mc_rate [%lld,%lld,%lld,%lld,%d,%d,%d] \n", realSampleTime, swSampleTime, mTask.hwSampleTime,mTask.lastSampleTime,mTask.sensors[ACC].hwRate, mTask.sensors[ACC].configed, accEventSize);
    }

    mTask.hwSampleTime = realSampleTime;
    mTask.lastSampleTime = realSampleTime;
    txTransferDataInfo(&mTask.dataInfo, accEventSize, 0, realSampleTime, data, 0);

    mt_eint_unmask(mTask.hw->eint_num);
    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);

    return 0;
}

static int mc34x9AccCali(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                            void *inBuf, uint8_t inSize, uint8_t elemInSize,
                            void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    float bias[AXES_NUM] = {0};
    mTask.sensors[ACC].startCali = true;

    MC_ALOGF();
    Acc_init_calibration(bias);

    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int mc34x9AccCfgCali(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                            void *inBuf, uint8_t inSize, uint8_t elemInSize,
                            void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    struct accGyroCaliCfgPacket caliCfgPacket;
    MC_ALOGF();
    ret = rxCaliCfgInfo(&caliCfgPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);

    if (ret < 0) {
       sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
       MC_ALOGE("mc34x9AccCfgCali, rx inSize and elemSize error\n");
       return -1;
    }
    MC_ALOGD("mc34x9AccCfgCali: cfgData[0]:%d, cfgData[1]:%d, cfgData[2]:%d\n",
       caliCfgPacket.caliCfgData[0], caliCfgPacket.caliCfgData[1], caliCfgPacket.caliCfgData[2]);

    mTask.sensors[ACC].staticCali[0] = (float)caliCfgPacket.caliCfgData[0] / 1000;
    mTask.sensors[ACC].staticCali[1] = (float)caliCfgPacket.caliCfgData[1] / 1000;
    mTask.sensors[ACC].staticCali[2] = (float)caliCfgPacket.caliCfgData[2] / 1000;

    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}


static int mc34x9AccPowerOn(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    MC_ALOGK("mc34x9AccPowerOn any :%d, acc:%d\n",
        mTask.sensors[ANYMO].powered, mTask.sensors[ACC].powered);
   
    mTask.sensors[ACC].powered = true;
    mTask.accSampleCntAfterPwrOn = 0;
    /*enable fifo interrupt*/
    SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_STANDBY, 100); //standy mode
    SPI_WRITE(MC34X9_REG_FIFO_C, 0x10, 100);//fifo reset
    SPI_WRITE(MC34X9_REG_FIFO_C, 0x6C, 100);//threshold enbale,All interrupts are routed to INT1
    //SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_WAKE, 100); //wake mode
    startflag=1;
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int mc34x9AccPowerOff(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    int ret = 0;
    struct accGyroCntlPacket cntlPacket;

    MC_ALOGK("mc34x9AccPowerOff any :%d, acc:%d\n",
        mTask.sensors[ANYMO].powered, mTask.sensors[ACC].powered);
    
    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
        MC_ALOGE("mc34x9AccPowerOff, rx inSize and elemSize error\n");
        return -1;
    }
    MC_ALOGD("mc34x9AccPowerOff rate:%d,hwRate:%d,latency:%lld,samplesToDiscard:%d\n", \
        mTask.sensors[ACC].rate,mTask.sensors[ACC].hwRate,mTask.sensors[ACC].latency,mTask.sensors[ACC].samplesToDiscard);
    mTask.sensors[ACC].rate = 0;
    mTask.sensors[ACC].hwRate = 0;
    mTask.sensors[ACC].latency = SENSOR_LATENCY_NODATA;
    mTask.sensors[ACC].samplesToDiscard = 0;

    SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_STANDBY, 100); //standy mode
    SPI_WRITE(MC34X9_REG_FIFO_C, 0x10, 100);//fifo reset
    SPI_WRITE(MC34X9_REG_FIFO_C, 0x68, 100);//threshold disable,All interrupts are routed to INT1
    SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_WAKE, 100); //wake mode	
    // TODO: how to set?
    registerAccGyroFifoInfo(0, 0);

    mTask.sensors[ACC].powered = false;
    mTask.sensors[ACC].configed = false;
    mTask.accSampleCntAfterPwrOn = 0;

    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

int mc34x9AmdPowerOn(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                            void *inBuf, uint8_t inSize, uint8_t elemInSize,
                            void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    MC_ALOGK(" mc34x9AmdPowerOn any :%d, acc:%d\n",
        mTask.sensors[ANYMO].powered, mTask.sensors[ACC].powered);

    SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_STANDBY, 100); //standy mode W reg,reg10=>{MCTRL[2:0]:1,standby}
    SPI_WRITE(MC34X9_REG_INTR_EN,  0x44, 100);//AMD
    SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_WAKE, 100); //wake mode
    mTask.sensors[ANYMO].powered = true;

    return  spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

int mc34x9AmdPowerOff(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                             void *inBuf, uint8_t inSize, uint8_t elemInSize,
                             void *outBuf, uint8_t *outSize, uint8_t *elemOutSize) {
    MC_ALOGK(" mc34x9AmdPowerOff any :%d, acc:%d\n",mTask.sensors[ANYMO].powered, mTask.sensors[ACC].powered);
    
    SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_STANDBY, 100);
    SPI_WRITE(MC34X9_REG_INTR_EN,  0x40, 100);//AMD
    SPI_WRITE(MC34X9_REG_MODE_C, MC34X9_MODE_WAKE, 100); //wake mode

    mTask.sensors[ANYMO].configed = false;
    mTask.sensors[ANYMO].powered = false;

    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static struct sensorFsm mc34x9Fsm[] = {
    /* Initialize: STATE_SW_RESET to STATE_INIT_DONE */
    sensorFsmCmd(STATE_SW_RESET, MC_INITIALIZE_01, mc34x9ModeStandBy),
    sensorFsmCmd(MC_INITIALIZE_01, MC_INITIALIZE_02, mc34x9Reset),
    sensorFsmCmd(MC_INITIALIZE_02, MC_INITIALIZE_03, mc34x9SetBusSpi),
    sensorFsmCmd(MC_INITIALIZE_03, MC_INITIALIZE_04, mc34x9SPISpeed),
    sensorFsmCmd(MC_INITIALIZE_04, STATE_SENSOR_REGISTRATION, mc34x9InitReg),
    sensorFsmCmd(STATE_SENSOR_REGISTRATION, STATE_EINT_REGISTRATION, mc34x9SensorRegistration),
    sensorFsmCmd(STATE_EINT_REGISTRATION, STATE_INIT_DONE, mc34x9EintRegistration),

    /* Enable accel: STATE_ACC_ENABLE to STATE_ACC_ENABLE_DONE */
    sensorFsmCmd(STATE_ACC_ENABLE, STATE_ACC_ENABLE_DONE, mc34x9AccPowerOn),

    /* Disable accel: STATE_ACC_DISABLE to STATE_ACC_DISABLE_DONE */
    sensorFsmCmd(STATE_ACC_DISABLE, STATE_ACC_DISABLE_DONE, mc34x9AccPowerOff),

     /* Change rate and fifo config: STATE_ACC_RATECHG to STATE_ACC_RATECHG_DONE */
    sensorFsmCmd(STATE_ACC_RATECHG, STATE_ACC_RATECHG_DONE, mc34x9AccRate),

    /*Int handling:STATE_HW_INT_STATUS_CHECK -> STATE_HW_INT_HANDLING -> STATE_HW_INT_HANDLING_DONE*/
    sensorFsmCmd(STATE_HW_INT_STATUS_CHECK, STATE_HW_INT_HANDLING, mc34x9IntStatusCheck_01),
    sensorFsmCmd(STATE_HW_INT_HANDLING, STATE_FIFO,mc34x9FsmCheckFifo),
    sensorFsmCmd(STATE_FIFO, STATE_CONVERT, mc34x9ReadFifo),
    sensorFsmCmd(STATE_CONVERT, STATE_HW_INT_HANDLING_DONE,mc34x9clearint),
    

    /*Get fifo data:STATE_SAMPLE -> STATE_FIFO -> STATE_CONVERT -> STATE_SAMPLE_DONE*/
    sensorFsmCmd(STATE_SAMPLE, MC_INITIALIZE_05, mc34x9Sample),
    sensorFsmCmd(MC_INITIALIZE_05, STATE_SAMPLE_DONE, mc34x9Convert),

    /* Init calibration: STATE_ACC_CALI to STATE_ACC_CALI_DONE */
    sensorFsmCmd(STATE_ACC_CALI, STATE_ACC_CALI_DONE, mc34x9AccCali),
    sensorFsmCmd(STATE_ACC_CFG, STATE_ACC_CFG_DONE, mc34x9AccCfgCali),

    /* For Anymotion */
    sensorFsmCmd(STATE_ANYMO_ENABLE, STATE_ANYMO_ENABLE_DONE, mc34x9AmdPowerOn),
    sensorFsmCmd(STATE_ANYMO_DISABLE, STATE_ANYMO_DISABLE_DONE, mc34x9AmdPowerOff),
};

static void initSensorStruct(struct mc34x9Sensor *sensor, enum SensorIndex idx) {
    sensor->powered = false;
    sensor->configed = false;
    sensor->rate = 0;
    sensor->latency = SENSOR_LATENCY_NODATA;
    sensor->hwRate = 0;  // rate set in hw

    sensor->startCali = false;
    sensor->staticCali[AXIS_X] = 0;
    sensor->staticCali[AXIS_Y] = 0;
    sensor->staticCali[AXIS_Z] = 0;
    sensor->accuracy = 0;
}
#if 0
int ret_flag = 0;
static void  mc34x9Detect(void *cookie, int err)
{
   int ret = 0;
   MC_ALOGE(" mc34x9Detect:a:%02x,b:%02x,c:%02x,d:%02x\n", mTask.regBuffer[0],mTask.regBuffer[1],mTask.regBuffer[2],mTask.regBuffer[3]);
    if (ret < 0 || (mTask.regBuffer[2] != 0xA4)) {
        MC_ALOGE("failed id match: %02x, ret: %d\n", mTask.regBuffer[2], ret);
        ret_flag = -1;
        spiMasterRelease(mTask.spiDev);
        disable_latch_time(mTask.latch_time_id);
        free_latch_time(mTask.latch_time_id);
      //  goto err_out;
    }
    if(mTask.regBuffer[2] == MC34X9_WAI_VALUE)
    {
        MC_ALOGD("mc34x9: auto detect success: %02x\n", mTask.regBuffer[2]);
        accSensorRegister();
        anyMotionSensorRegister();
        registerAccGyroInterruptMode(ACC_GYRO_FIFO_INTERRUPTIBLE);
        registerAccGyroDriverFsm(mc34x9Fsm, ARRAY_SIZE(mc34x9Fsm));
    }
}
#endif
int mc34x9Init(void) {
    int ret = 0;
    uint8_t txData[2] = {0}, rxData[4] = {0};
    enum SensorIndex i;
    insertMagicNum(&mTask.accGyroPacket);
    mTask.hw = get_cust_accGyro("mc34x9");
    if (NULL == mTask.hw) {
        MC_ALOGE("get_cust_acc_hw fail\n");
        ret = -1;
     //   goto err_out;
    }

    MC_ALOGD("start  acc spi_num: %d\n", mTask.hw->i2c_num);
    if (0 != (ret = sensorDriverGetConvert(mTask.hw->direction, &mTask.cvt))) {
        MC_ALOGE("invalid direction: %d\n", mTask.hw->direction);
    }
    MC_ALOGD("acc map[0]:%d, map[1]:%d, map[2]:%d, sign[0]:%d, sign[1]:%d, sign[2]:%d\n\r",
        mTask.cvt.map[AXIS_X], mTask.cvt.map[AXIS_Y], mTask.cvt.map[AXIS_Z],
        mTask.cvt.sign[AXIS_X], mTask.cvt.sign[AXIS_Y], mTask.cvt.sign[AXIS_Z]);
    
    mTask.sensors[ACC].sensitivity =  4096;
    mTask.watermark = 0;
   // mTask.temperature = 25.0f;  // init with 25 Celsius
   // mTask.tempReady = false;
    mTask.fifoDataToRead = 0;

    mTask.latch_time_id = alloc_latch_time();
    enable_latch_time(mTask.latch_time_id, mTask.hw->eint_num);

    for (i = ACC; i < NUM_OF_SENSOR; i++) {
        initSensorStruct(&mTask.sensors[i], i);
    }
    mTask.mode.speed = 8000000;//8Mhz
    mTask.mode.bitsPerWord = 8;
    mTask.mode.cpol = SPI_CPOL_IDLE_HI;//SPI_CPOL_IDLE_LO;
    mTask.mode.cpha = SPI_CPHA_TRAILING_EDGE;//SPI_CPHA_LEADING_EDGE;
    mTask.mode.nssChange = true;
    mTask.mode.format = SPI_FORMAT_MSB_FIRST;
    mTask.mWbufCnt = 0;
    mTask.mRegCnt = 0;
    mTask.drop = 0;
    mTask.dropCnt = 0;
    spiMasterRequest(mTask.hw->i2c_num, &mTask.spiDev);
#if 0
    SPI_READ(MC34X9_REG_CHIP_ID, 1, &mTask.regBuffer,50000);
    ret = spiBatchTxRx(&mTask.mode, mc34x9Detect, &mTask, __FUNCTION__);
#endif
    /* Check PID */
    txData[0] = MC34X9_REG_CHIP_ID | 0x80;
    ret = spiMasterRxTxSync(mTask.spiDev, rxData, txData, 4);

    if (0 > ret)
    {
	MC_ALOGE("fail in spiMasterRxTxSync, ret=%d \n", ret);
        goto exit_error;
    }
    else if (MC34X9_WAI_VALUE != rxData[2])
    {
        MC_ALOGE("failed id match:0x%02X \n", rxData[2]);
        goto exit_error;
    }

    MC_ALOGD("mc34x9: auto detect success: %02x\n", rxData[2]);
    accSensorRegister();
    anyMotionSensorRegister();
    registerAccGyroInterruptMode(ACC_GYRO_FIFO_INTERRUPTIBLE);
    registerAccGyroDriverFsm(mc34x9Fsm, ARRAY_SIZE(mc34x9Fsm));

    return ret;
exit_error:
    spiMasterRelease(mTask.spiDev);
    disable_latch_time(mTask.latch_time_id);
    free_latch_time(mTask.latch_time_id);
    return -1;
#if 0    
    if(ret_flag == 0){
        return 0;
    }else{
        return -1;
    }
#endif
}

#ifndef CFG_OVERLAY_INIT_SUPPORT
MODULE_DECLARE(mc34x9, SENS_TYPE_ACCEL, mc34x9Init);
#else
#include "mtk_overlay_init.h"
OVERLAY_DECLARE(mc34x9, OVERLAY_WORK_00, mc34x9Init);
#endif

