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
#include <atomic.h>
#include <cpu/inc/cpuMath.h>
#include <gpio.h>
#include <heap.h>
#include <slab.h>
#include <timer.h>
#include <variant/inc/variant.h>
#include <limits.h>
#include <performance.h>
#include <sensors.h>
#include <plat/inc/rtc.h>
#include <plat/inc/spichre.h>
#include <spichre-plat.h>
#include <hostIntf.h>
#include <nanohub_math.h>
#include <nanohubPacket.h>
#include <plat/inc/rtc.h>
#include <spi.h>
#include <contexthub_core.h>
#include <accGyro.h>
#include <contexthub_core.h>
#include <cust_accGyro.h>
#include <mt_gpt.h>
#include <API_sensor_calibration.h>
#include <algos/time_sync.h>
#include "hwsen.h"
#include "eint.h"

#define ACC_NAME     "icm20607_acc"
#define GYRO_NAME     "icm20607_gyro"

#define SUPPORT_ANYMO       1
#define DEBUG_PHASE         0
#define DEBUG_FIFO_DATA     0

#define SAMPLE_TO_DISCARD 0 //3
#define ODR2SMPLRT_DIV(odr)         (1000/(odr) - 1)

/* ICM2060X register  */
#define AXIS_X              0
#define AXIS_Y              1
#define AXIS_Z              2
#define AXES_NUM            3

#define ICM20608G_WHOAMI            0xAF
#define ICM20608D_WHOAMI            0xAE
#define ICM20609_WHOAMI             0xA6
#define ICM20600_WHOAMI             0x11
#define ICM20602_WHOAMI             0x12
#define ICM20607_WHOAMI             0x05

#define ICM2060X_REG_LP_CONFIG          0x1e
#define ICM2060X_PWR_MGMT_1             0x6b
#define ICM2060X_PWR_MGMT_2             0x6c
#define ICM2060X_REG_FIFO_R_W           0x74
#define ICM2060X_REG_FIFO_COUNTH        0x72
#define ICM2060X_REG_FIFO_COUNTL        0x73
#define ICM2060X_REG_FIFO_EN_2          0x23
#define ICM2060X_REG_FIFO_RST           0x6a
#define ICM2060X_REG_USER_CTRL          0x6a
#define ICM2060X_REG_INT_ENABLE         0x38
#define ICM2060X_REG_FIFO_WM_TH1        0x60
#define ICM2060X_REG_FIFO_WM_TH2        0x61
#define ICM2060X_REG_FIFO_WM_STATUS     0x39
#define ICM2060X_REG_INT_STATUS         0x3a
#define ICM2060X_REG_I2C_IF             0x70

#define ICM2060X_REG_ACCEL_INTEL_CTRL   0x69
#define ICM2060X_REG_WOM_THR_X          0x20
#define ICM2060X_REG_WOM_THR_Y          0x21
#define ICM2060X_REG_WOM_THR_Z          0x22
#define ICM2060X_REG_WOM_THR            0x1f

#define BIT_WOM_EN                      0xE0

//REG_PWR_MGMT_1
#define BIT_DEVICE_RESET            0x80
#define BIT_SLEEP                   0x40
#define BIT_CYCLE                   0x20
#define BIT_CLKSEL                  0x07

#define BIT_GYRO_LP_EN                  0x80
//#define BIT_ACC_LP_EN                   0x20
//#define BIT_G_AVGCFG                    0x00
//#define BIT_ACC_I2C_MST                 0x40

#define BIT_LP_EN                       0x20
#define BIT_CLK_PLL                     0x01
#define BIT_TEMP_DIS                    (1<<3)

#define BIT_PWR_ACCEL_STBY              0x38
#define BIT_PWR_GYRO_STBY               0x07

#define ICM2060X_TIMEBASE_PLL           0x28

/* gyro */
#define ICM2060X_REG_SAMRT_DIV          0x19
#define ICM2060X_REG_GYRO_CONFIG        0x1a
#define ICM2060X_REG_GYRO_CONFIG_1      0x1b


#define SHIFT_GYRO_FS_SEL                  3
#define GYRO_FS_SEL_250                     (0x00 << SHIFT_GYRO_FS_SEL)
#define GYRO_FS_SEL_500                     (0x01 << SHIFT_GYRO_FS_SEL)
#define GYRO_FS_SEL_1000                    (0x02 << SHIFT_GYRO_FS_SEL)
#define GYRO_FS_SEL_2000                    (0x03 << SHIFT_GYRO_FS_SEL)

#define ICM2060X_FS_250_LSB             131.0f
#define ICM2060X_FS_500_LSB             65.5f
#define ICM2060X_FS_1000_LSB            32.8f
#define ICM2060X_FS_2000_LSB            16.4f

#define GYRO_AVGCFG_1X                     0
#define GYRO_AVGCFG_2X                     1
#define GYRO_AVGCFG_4X                     2
#define GYRO_AVGCFG_8X                     3
#define GYRO_AVGCFG_16X                    4
#define GYRO_AVGCFG_32X                    5
#define GYRO_AVGCFG_64X                    6
#define GYRO_AVGCFG_128X                   7
#define GYRO_DLPFCFG                    (0x04)      //dlpf_cfg for gyro @4
#define GYRO_FCHOICE                    (0x00)
/* gyro */

/* acc */
//#define ICM2060X_REG_SAMRT_DIV1         0x10
//#define ICM2060X_REG_SAMRT_DIV2         0x11
#define ICM2060X_REG_ACC_CONFIG         0x1c
#define ICM2060X_REG_ACC_CONFIG_2       0x1d


#define ICM2060X_RANGE_2G               (0x00 << 3)
#define ICM2060X_RANGE_4G               (0x01 << 3)
#define ICM2060X_RANGE_8G               (0x02 << 3)
#define ICM2060X_RANGE_16G              (0x03 << 3)

#define ACCEL_AVGCFG_1_4X               (0x00 << 4)
#define ACCEL_AVGCFG_8X                 (0x01 << 4)
#define ACCEL_AVGCFG_16X                (0x02 << 4)
#define ACCEL_AVGCFG_32X                (0x03 << 4)
#define ACCEL_FCHOICE                      0 << 3
#define ACCEL_DLPFCFG                      4

/* acc */
#define ACCEL_AVG_SAMPLE                (0x3)

#define MAX_FIFO_SIZE_1024BYTES         0xc0

#define ICM2060X_REG_DATAX0             0x2d
#define ICM2060X_REG_DATAY0             0x2f
#define ICM2060X_REG_DATAZ0             0x31


#define ICM2060X_SLEEP                  0x40    //enable low power sleep mode
#define ICM2060X_DEV_RESET              0x80

#define FIFO_MODE                       0x40    //fifo mode for fifo
#define FIFO_EN                         0x40
#define FIFO_DIS                        0x00
#define FIFO_RST                        0x04
#define FIFO_BYTE_MODE                  14      //A 6 , T 2,G 6
#define BIT_FIFO_OFLOW_EN               0x10

#define BIT_WOM_EN                      0xE0 //0x40?
//REG_FIFO_WM_INT_STATUS
#define BIT_FIFO_WM_INT                 0x40
//REG_INT_STATUS
#define BIT_DRI_INT                     0x01
#define BIT_WOM_X_INT                   0x80
#define BIT_WOM_Y_INT                   0x40
#define BIT_WOM_Z_INT                   0x20
#define BIT_FIFO_OFLOW_INT              0x10

//REG_ACC_INTEL_CTRL
#define BIT_ACC_INTEL_EN            0x80
#define BIT_ACC_INTEL_MODE          0x40
#define BIT_ACCEL_FCHOICE_OIS_B     0x30
#define BIT_WOM_INT_MODE            0x01

#define DEF_WOM_THRESHOLD           (100 >> 2) //default:100mg, max=1020mg, LSB = 4mg.

/* ICM2060X Register Map  (Please refer to ICM2060X Specifications) */
#define ICM2060X_REG_DEVID              0x75
#define remapFifoDelay(delay_ns, pll)     ((delay_ns) * (1270ULL)/(1270ULL + pll))

#define ICM2060X_MAX_ODR_500HZ (1)

//#define ICM2060X_FIFO_SIZE                       2080
#define G                                        9.80665
#define PI                                       3.141592
#define KSCALE_ACC_8G_RANGE                      0.002394202f  //sensitivity m/s^2   // ACC_range * 9.81f / 65536.0f;
#define KSCALE_GYRO_2000_RANGE                   0.001065264f  // sesitivity rps // GYR_range * M_PI / (180.0f * 65536.0f);

#define ICM2060X_ODR_2HZ_REG_VALUE                    (0xe)
#define ICM2060X_ODR_3HZ_REG_VALUE                    (0xd)
#define ICM2060X_ODR_6HZ_REG_VALUE                    (0xc)
#define ICM2060X_ODR_13HZ_REG_VALUE                   (0xb)
#define ICM2060X_ODR_25HZ_REG_VALUE                   (0xa)
#define ICM2060X_ODR_50HZ_REG_VALUE                   (0x9)
#define ICM2060X_ODR_100HZ_REG_VALUE                  (0x8)
#define ICM2060X_ODR_200HZ_REG_VALUE                  (0x7)
#define ICM2060X_ODR_500HZ_REG_VALUE                  (0xf)
#define ICM2060X_ODR_1000HZ_REG_VALUE                 (0x6)
#define ICM2060X_ODR_2000HZ_REG_VALUE                 (0x5)
#define ICM2060X_ODR_4000HZ_REG_VALUE                 (0x4)
#define ICM2060X_ODR_8000HZ_REG_VALUE                 (0x3)

#define max(x, y)   (x > y ? x : y)

#define AXIS_X              0
#define AXIS_Y              1
#define AXIS_Z              2
#define AXES_NUM            3

/* Support odr range 25HZ - 200HZ */
static uint32_t ICM2060XHWRates[] = {
    SENSOR_HZ(5.0f),
    SENSOR_HZ(10.0f),
    SENSOR_HZ(15.0f),
    SENSOR_HZ(20.0f),
    SENSOR_HZ(25.0f),
    SENSOR_HZ(40.0f),
    SENSOR_HZ(50.0f),
    SENSOR_HZ(100.0f),
    SENSOR_HZ(200.0f),
    SENSOR_HZ(250.0f),
#if ICM2060X_MAX_ODR_500HZ
    SENSOR_HZ(500.0f),
#endif
    0,
};

#define SPI_PACKET_SIZE 30
#define ICM2060X_MAX_FIFO_SIZE  (73 * FIFO_BYTE_MODE)// 73 * 14 < 1024(hw), 73 < 100 (accGyro common data buffer size)
#define SPI_BUF_SIZE    ((ICM2060X_MAX_FIFO_SIZE + 4) + 4)
#define EVT_SENSOR_ANY_MOTION       sensorGetMyEventType(SENS_TYPE_ANY_MOTION)

#define SPI_WRITE_0(addr, data) spiQueueWrite(addr, data, 2)
#define SPI_WRITE_1(addr, data, delay) spiQueueWrite(addr, data, delay)
#define GET_SPI_WRITE_MACRO(_1, _2, _3, NAME, ...) NAME
#define SPI_WRITE(...) GET_SPI_WRITE_MACRO(__VA_ARGS__, SPI_WRITE_1, SPI_WRITE_0)(__VA_ARGS__)

#define SPI_READ_0(addr, size, buf) spiQueueRead(addr, size, buf, 0)
#define SPI_READ_1(addr, size, buf, delay) spiQueueRead(addr, size, buf, delay)
#define GET_SPI_READ_MACRO(_1, _2, _3, _4, NAME, ...) NAME
#define SPI_READ(...) GET_SPI_READ_MACRO(__VA_ARGS__, SPI_READ_1, SPI_READ_0)(__VA_ARGS__)


enum ICM2060XState {
    STATE_SAMPLE = CHIP_SAMPLING,
    STATE_FIFO = CHIP_FIFO,
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
    STATE_GYRO_ENABLE = CHIP_GYRO_ENABLE,
    STATE_GYRO_ENABLE_DONE = CHIP_GYRO_ENABLE_DONE,
    STATE_GYRO_DISABLE = CHIP_GYRO_DISABLE,
    STATE_GYRO_DISABLE_DONE = CHIP_GYRO_DISABLE_DONE,
    STATE_GYRO_RATECHG = CHIP_GYRO_RATECHG,
    STATE_GYRO_RATECHG_DONE = CHIP_GYRO_RATECHG_DONE,
    STATE_GYRO_CALI = CHIP_GYRO_CALI,
    STATE_GYRO_CALI_DONE = CHIP_GYRO_CALI_DONE,
    STATE_GYRO_CFG = CHIP_GYRO_CFG,
    STATE_GYRO_CFG_DONE = CHIP_GYRO_CFG_DONE,
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
    STATE_SW_RESET_W,
    STATE_RESET_CHECK,
    STATE_INT_STATUS,
    STATE_CLKSEL_REG_READ,
    STATE_CLKSEL_REG_WRITE,
    STATE_POWER_R,
    STATE_ENPOWER_W,
    STATE_POWER2_R,
    STATE_INIT_REG,
    STATE_SENSOR_REGISTRATION,
    STATE_EINT_REGISTRATION,
    STATE_REGDUMP_READ,       //for debug
    STATE_REGDUMP_RESULT,     //for debug
};

enum SensorIndex {
    ACC = 0,
    GYR,
    NUM_OF_SENSOR,
};

enum SensorHandleIndex {
    ANYMO = 0,
    NUM_OF_HANDLE,
};

struct scale_factor {
    unsigned char  whole;
    unsigned char  fraction;
};

struct acc_data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
struct gyro_data_resolution {
    float   sensitivity;
};

struct ICM2060XSensor {
    float staticCali[AXES_NUM];
    uint64_t latency;
    int32_t accuracy;
    uint32_t rate;
    uint32_t hwRate;
    uint32_t preRealRate;
    uint32_t handle;
    float sensitivity;
    bool powered;
    bool configed;
    bool startCali;
    bool needDiscardSample;     //Terry
    uint32_t samplesToDiscard;  //Terry
    enum SensorHandleIndex idx_handle;
};

static struct ICM2060XTask {
    struct ICM2060XSensor sensors[NUM_OF_SENSOR];
    struct ICM2060XSensor sensors_handle[NUM_OF_HANDLE];

    uint64_t hwSampleTime;
    uint64_t swSampleTime;
    uint64_t unmask_eint_time;

    uint64_t lastSampleTime;
    uint8_t *regBuffer;
    uint8_t *statusBuffer;
    uint8_t *int_statusBuffer;
#if DEBUG_PHASE
    uint8_t *debugintBuffer;
#endif

    SpiCbkF spiCallBack;
    struct transferDataInfo dataInfo;
    struct accGyroDataPacket accGyroPacket;
    /* data for factory */
    struct TripleAxisDataPoint accFactoryData;
    struct TripleAxisDataPoint gyroFactoryData;

    int32_t accHwCali[AXES_NUM];
    int32_t gyroHwCali[AXES_NUM];
    struct acc_data_resolution *accReso;
    struct gyro_data_resolution *gyroReso;
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
    int32_t debug_trace;
    uint32_t fifoDataToRead;

    int latch_time_id;
    uint16_t watermark;
    /* For save reg status */
    uint8_t int_src0;
    uint8_t smd_cfg_reg69;
    uint8_t acc_cfg0;
    uint8_t gyro_cfg0;
    //uint8_t fifo_cfg1;
    uint8_t pwr_mgmr_config;
    uint8_t pwr_mgmr_config2;
} mTask;
static struct ICM2060XTask *ICM2060XDebugPoint;

static void spiQueueWrite(uint8_t addr, uint8_t data, uint32_t delay)
{
    mTask.packets[mTask.mRegCnt].size = 2;
    mTask.packets[mTask.mRegCnt].txBuf = &mTask.txrxBuffer[mTask.mWbufCnt];
    mTask.packets[mTask.mRegCnt].rxBuf = &mTask.txrxBuffer[mTask.mWbufCnt];
    mTask.packets[mTask.mRegCnt].delay = delay * 1000;
    mTask.txrxBuffer[mTask.mWbufCnt++] = addr;
    mTask.txrxBuffer[mTask.mWbufCnt++] = data;
    mTask.mWbufCnt = (mTask.mWbufCnt + 3) & 0xFFFC;
    mTask.mRegCnt++;
}

static void spiQueueRead(uint8_t addr, size_t size, uint8_t **buf, uint32_t delay)
{
    *buf = &mTask.txrxBuffer[mTask.mWbufCnt];
    mTask.packets[mTask.mRegCnt].size = size + 1;  // first byte will not contain valid data
    mTask.packets[mTask.mRegCnt].txBuf = &mTask.txrxBuffer[mTask.mWbufCnt];
    mTask.packets[mTask.mRegCnt].rxBuf = *buf;
    mTask.packets[mTask.mRegCnt].delay = delay * 1000;
    mTask.txrxBuffer[mTask.mWbufCnt++] = 0x80 | addr;
    mTask.mWbufCnt = (mTask.mWbufCnt + size + 3) & 0xFFFC;
    mTask.mRegCnt++;
}

static int spiBatchTxRx(struct SpiMode *mode,
                        SpiCbkF callback, void *cookie, const char *src)
{
    int err = 0;

    if (mTask.mWbufCnt > SPI_BUF_SIZE) {
        osLog(LOG_INFO, "NO enough SPI buffer space, dropping transaction.\n");
        return -1;
    }
    if (mTask.mRegCnt > SPI_PACKET_SIZE) {
        osLog(LOG_INFO, "spiBatchTxRx too many packets!\n");
        return -1;
    }

    err = spiMasterRxTx(mTask.spiDev, mTask.cs, mTask.packets, mTask.mRegCnt, mode, callback, cookie);
    mTask.mRegCnt = 0;
    mTask.mWbufCnt = 0;
    return err;
}

static int ICM2060XCalcuOdr(uint32_t *rate, uint32_t *report_rate)
{
    int i;

    for (i = 0; i < (ARRAY_SIZE(ICM2060XHWRates) - 1); i++) {
        if (*rate <= ICM2060XHWRates[i]) {
            *report_rate = ICM2060XHWRates[i];
            break;
        }
    }

    if (*rate > ICM2060XHWRates[(ARRAY_SIZE(ICM2060XHWRates) - 2)]) {
        i = (ARRAY_SIZE(ICM2060XHWRates) - 2);
        *report_rate = ICM2060XHWRates[i];
    }

    return (*report_rate) / 1024;
}

void ICM2060XTimerCbkF(void)
{
    mTask.hwSampleTime = rtcGetTime();
}

static int ICM2060XResetWrite(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                              void *inBuf, uint8_t inSize, uint8_t elemInSize,
                              void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    uint8_t val = 0;

    osLog(LOG_INFO, "ICM2060XResetWrite 0x%x\n", mTask.regBuffer[1]);
    val = mTask.regBuffer[1] | ICM2060X_DEV_RESET;
    /* Wait 200ms to check reset status, then check again */
    SPI_WRITE(ICM2060X_PWR_MGMT_1, val, 200000);
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int ICM2060XResetCheck(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                              void *inBuf, uint8_t inSize, uint8_t elemInSize,
                              void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    if (!(BIT_DEVICE_RESET & mTask.regBuffer[1]))
        osLog(LOG_INFO, "ICM2060XResetCheck Done\n");
    else
        osLog(LOG_ERROR, "ICM2060XResetCheck Fail 0x%x\n", mTask.regBuffer[1]);
    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int ICM2060XResetRead(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                             void *inBuf, uint8_t inSize, uint8_t elemInSize,
                             void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    SPI_READ(ICM2060X_PWR_MGMT_1, 1, &mTask.regBuffer);
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}


static int ICM2060XClkSelWrite(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                               void *inBuf, uint8_t inSize, uint8_t elemInSize,
                               void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    uint8_t val = 0;

    osLog(LOG_INFO, "ICM2060XClkSelWrite 0x%x\n", mTask.regBuffer[1]);
    val = mTask.regBuffer[1] | BIT_CLK_PLL;
    SPI_WRITE(ICM2060X_PWR_MGMT_1, val);
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int ICM2060X_PWRMGMT_Read(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                 void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                 void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    SPI_READ(ICM2060X_PWR_MGMT_1, 1, &mTask.regBuffer);
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int ICM2060X_PWRMGMT2_Read(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                  void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                  void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    SPI_READ(ICM2060X_PWR_MGMT_2, 1, &mTask.statusBuffer);
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int ICM2060XPowerEnableWrite(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                    void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{

    osLog(LOG_INFO, "ICM2060XPowerEnableWrite 0x%x\n", mTask.regBuffer[1]);
    mTask.pwr_mgmr_config = mTask.regBuffer[1] & ~ICM2060X_SLEEP;
    SPI_WRITE(ICM2060X_PWR_MGMT_1, mTask.pwr_mgmr_config);
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}
static int ICM2060XInitConfig(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                              void *inBuf, uint8_t inSize, uint8_t elemInSize,
                              void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "ICM2060XInitConfig\n");

    SPI_WRITE(ICM2060X_REG_I2C_IF, 0x40);                            // disable i2c module
    /* gyro  fs */
    mTask.pwr_mgmr_config2 = mTask.statusBuffer[1];
    mTask.gyro_cfg0 =  FIFO_MODE | GYRO_DLPFCFG;                      //SET fifo mode
    SPI_WRITE(ICM2060X_REG_GYRO_CONFIG_1, GYRO_FS_SEL_2000 | GYRO_FCHOICE);
    SPI_WRITE(ICM2060X_REG_GYRO_CONFIG, FIFO_MODE | GYRO_DLPFCFG);    // also write bit 7 for 0

    mTask.acc_cfg0 =  ICM2060X_RANGE_8G;     // BIT_ACC_FS_4G -> 8G
    SPI_WRITE(ICM2060X_REG_ACC_CONFIG, ICM2060X_RANGE_8G);
    // fifo size @ 1024bytes,acc DLPCFG@4
    SPI_WRITE(ICM2060X_REG_ACC_CONFIG_2, MAX_FIFO_SIZE_1024BYTES | ACCEL_AVGCFG_1_4X | ACCEL_DLPFCFG | ACCEL_FCHOICE);

    /* always set A T G into fifo, format the fifo package @14bytes  */
    SPI_WRITE(ICM2060X_REG_FIFO_EN_2, 0xF8);

    /* acc & gyro  stand by , sleep mode in idle */
    SPI_WRITE(ICM2060X_PWR_MGMT_2, mTask.pwr_mgmr_config2 | BIT_PWR_GYRO_STBY | BIT_PWR_ACCEL_STBY);
    SPI_WRITE(ICM2060X_PWR_MGMT_1, mTask.pwr_mgmr_config | ICM2060X_SLEEP);

    /*update globle status pwr_mgmr_config2, pwr_mgmr_config2 */

    mTask.pwr_mgmr_config2 |= BIT_PWR_GYRO_STBY | BIT_PWR_ACCEL_STBY;
    mTask.pwr_mgmr_config  |= ICM2060X_SLEEP;

    /* get interrupt reg status */
    SPI_READ(ICM2060X_REG_INT_ENABLE, 1, &mTask.regBuffer);

    /*get ACCEL_INTEL_CTRL  */
    SPI_READ(ICM2060X_REG_ACCEL_INTEL_CTRL, 1, &mTask.statusBuffer);
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static void ICM2060XFIFO_Restart(void)
{
    osLog(LOG_INFO, "ICM2060XFIFO_Restart\n");
    SPI_WRITE(ICM2060X_REG_USER_CTRL, FIFO_DIS);
    SPI_WRITE(ICM2060X_REG_USER_CTRL, FIFO_RST);
    SPI_WRITE(ICM2060X_REG_FIFO_EN_2, 0xF8);
    SPI_WRITE(ICM2060X_REG_USER_CTRL, FIFO_EN);
}

static void ICM2060XFIFO_Restart_Read(void)
{
    osLog(LOG_INFO, "ICM2060XFIFO_Restart read\n");
    SPI_WRITE(ICM2060X_REG_USER_CTRL, FIFO_DIS);
    mTask.fifoDataToRead = (mTask.statusBuffer[1] << 8) | mTask.statusBuffer[2];
    SPI_READ(ICM2060X_REG_FIFO_R_W, mTask.fifoDataToRead, &mTask.regBuffer);
    SPI_WRITE(ICM2060X_REG_USER_CTRL, FIFO_RST);
    SPI_WRITE(ICM2060X_REG_FIFO_EN_2, 0xF8);
    SPI_WRITE(ICM2060X_REG_USER_CTRL, FIFO_EN);
}

static void ICM2060XConfigFifo(bool odr_change)
{
    uint8_t  buffer[2];
    uint16_t watermarkReg;

    watermarkReg = mTask.watermark;

#if 1//byte mode
    if (watermarkReg < FIFO_BYTE_MODE)
        watermarkReg = FIFO_BYTE_MODE;
#else
    if (watermarkReg == 0)
        watermarkReg = 1;
#endif

    buffer[0] = watermarkReg & 0x00ff;
    buffer[1] = (watermarkReg & 0xff00) >> 8;

    if (odr_change) {
        /* reset & restart fifo first if odr change */
        ICM2060XFIFO_Restart();
        /* set threshold */
        SPI_WRITE(ICM2060X_REG_FIFO_WM_TH2 , buffer[0]);
        SPI_WRITE(ICM2060X_REG_FIFO_WM_TH1, buffer[1]);

        osLog(LOG_INFO, "ICM2060XConfigFifo: Reset, TH_L:0x%x, TH_H:0x%x\n",
              buffer[0], buffer[1]);
        //osLog(LOG_INFO, "fifo config rtc %llu\n",rtcGetTime());
    } else {
        SPI_WRITE(ICM2060X_REG_FIFO_WM_TH2 , buffer[0]);
        SPI_WRITE(ICM2060X_REG_FIFO_WM_TH1, buffer[1]);
        if (mTask.sensors[ACC].configed || mTask.sensors[GYR].configed) {
            SPI_WRITE(ICM2060X_REG_USER_CTRL, FIFO_EN);
        } else {
            osLog(LOG_INFO, "ICM2060XConfigFifo disable fifo\n");
            SPI_WRITE(ICM2060X_REG_USER_CTRL, FIFO_DIS);        // if acc and gyro all off, fifo disable
        }
        osLog(LOG_INFO, "ICM2060XConfigFifo, TH_L:0x%x, TH_H:0x%x\n",
              buffer[0], buffer[1]);
    }
}

static uint16_t ICM2060XCalcuWm(void)
{
    uint8_t handle;
    uint64_t min_latency = SENSOR_LATENCY_NODATA;
    uint8_t min_watermark = 1;
    /* FIFO length is 1024 and a record is 16Bytes, so max_watermark set 60 */
    uint8_t max_watermark = /*60*/50;
    uint16_t watermark = 0;
    uint32_t temp_cnt, total_cnt = 0;
    uint32_t temp_delay = 0;

    for (handle = ACC; handle < NUM_OF_SENSOR; handle++) {
        if (mTask.sensors[handle].hwRate && mTask.sensors[handle].latency != SENSOR_LATENCY_NODATA) {
            min_latency =
                mTask.sensors[handle].latency < min_latency ? mTask.sensors[handle].latency : min_latency;
        }
    }
    /*if acc and gyr off or acc and gyr latency = SENSOR_LATENCY_NODATA, watermark = 0 or 1 or 2*/
    if (min_latency == SENSOR_LATENCY_NODATA) {
        for (handle = ACC; handle < NUM_OF_SENSOR; handle++) {
            if (mTask.sensors[handle].hwRate) {
                watermark++;
            }
        }
    } else {
        for (handle = ACC; handle < NUM_OF_SENSOR; handle++) {
            if (mTask.sensors[handle].hwRate) {
                temp_delay = (1000000000ULL / mTask.sensors[handle].hwRate) << 10;
                temp_cnt = min_latency / temp_delay;
                min_watermark = mTask.sensors[handle].hwRate / SENSOR_HZ(400.0f);
                total_cnt = temp_cnt > min_watermark ? temp_cnt : min_watermark;
                osLog(LOG_INFO, "ICM2060XCalcuWm, delay=%d, latency:%lld, min_wm=%d, total_cnt=%d\n",
                      temp_delay, min_latency, min_watermark, total_cnt);
            }
        }

        watermark = total_cnt;
        watermark = watermark < min_watermark ? min_watermark : watermark;
        watermark = watermark > max_watermark ? max_watermark : watermark;
    }

    return watermark * FIFO_BYTE_MODE;/* byte mode */
}

static int ICM2060XgEnable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;

    osLog(LOG_INFO, "ICM2060XgEnable power mode=%d, time:%lld\n", mTask.sensors[ACC].powered, rtcGetTime());

    //Go back from sleep mode to normal mode if necessary
    if (false == mTask.sensors[ACC].powered && false == mTask.sensors[GYR].powered) {
        osLog(LOG_INFO, "ICM2060XgEnable: acc go out sleep\n");
        mTask.pwr_mgmr_config &= ~ICM2060X_SLEEP;
        SPI_WRITE(ICM2060X_PWR_MGMT_1, mTask.pwr_mgmr_config);
    }
    mTask.sensors[ACC].powered = true;
    mTask.pwr_mgmr_config2 &= ~BIT_PWR_ACCEL_STBY;
    SPI_WRITE(ICM2060X_PWR_MGMT_2, mTask.pwr_mgmr_config2,
              30000); //set 30ms for accel start-up time, note:at least 200us here to sync fly changes

    ret = spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
    return ret;
}

static int ICM2060XgDisable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                            void *inBuf, uint8_t inSize, uint8_t elemInSize,
                            void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    int odr = 0;
    uint8_t regValue = 0x00;
    uint32_t sampleRate = 0;
    struct accGyroCntlPacket cntlPacket;

    osLog(LOG_INFO, "ICM2060XAccPowerOff\n");

    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "ICM2060XAccPowerOff, rx inSize and elemSize error\n");
        return -1;
    }

    mTask.sensors[ACC].rate = 0;
    mTask.sensors[ACC].preRealRate = 0;
    mTask.sensors[ACC].hwRate = 0;
    mTask.sensors[ACC].latency = SENSOR_LATENCY_NODATA;
    mTask.sensors[ACC].needDiscardSample = false;
    mTask.sensors[ACC].samplesToDiscard = 0;

    if ((mTask.sensors[GYR].powered == false) && (mTask.sensors_handle[ANYMO].powered == false)) {
        mt_eint_mask(mTask.hw->eint_num);
        /* acc & gyro  stand by , sleep mode in idle */
        SPI_WRITE(ICM2060X_PWR_MGMT_2, mTask.pwr_mgmr_config2 | BIT_PWR_GYRO_STBY | BIT_PWR_ACCEL_STBY);
        SPI_WRITE(ICM2060X_PWR_MGMT_1, mTask.pwr_mgmr_config | ICM2060X_SLEEP);

        /*update globle status pwr_mgmr_config2, pwr_mgmr_config2 */

        mTask.pwr_mgmr_config2 |= BIT_PWR_GYRO_STBY | BIT_PWR_ACCEL_STBY;
        mTask.pwr_mgmr_config  |= ICM2060X_SLEEP;

    } else {  //  update gyro old ord
        if (mTask.sensors[GYR].hwRate != mTask.sensors[GYR].preRealRate
                && mTask.sensors[GYR].powered == true) {
            mTask.sensors[GYR].hwRate = mTask.sensors[GYR].preRealRate;
            //terry odr=CalcuOdr ?
            odr = ICM2060XCalcuOdr(&mTask.sensors[GYR].hwRate, &sampleRate);

            if (15 == odr)
                regValue = 66;
            else
                regValue = ODR2SMPLRT_DIV(odr);

            SPI_WRITE(ICM2060X_REG_SAMRT_DIV, regValue);
            osLog(LOG_INFO, "DIV %d,gyro change rate to preRealRate:%d\n", regValue, mTask.sensors[GYR].hwRate);
        }
        /* turn off acc if ANYMO OFF*/
        if (mTask.sensors_handle[ANYMO].powered == false) {
            SPI_WRITE(ICM2060X_PWR_MGMT_2, mTask.pwr_mgmr_config2 | BIT_PWR_ACCEL_STBY);

            /*update globle status , pwr_mgmr_config2 */
            mTask.pwr_mgmr_config2 |=  BIT_PWR_ACCEL_STBY;
        }

    }

    registerAccGyroFifoInfo((mTask.sensors[ACC].hwRate == 0) ? 0 : 1024000000000 / mTask.sensors[ACC].hwRate,
                            (mTask.sensors[GYR].hwRate == 0) ? 0 : 1024000000000 / mTask.sensors[GYR].hwRate);

    mTask.watermark = ICM2060XCalcuWm();
    mTask.sensors[ACC].powered = false;
    mTask.sensors[ACC].configed = false;   //Lomen

    ICM2060XConfigFifo(false);
#if 0
    if (mTask.sensors[GYR].powered == true) {
        mTask.fifo_cfg1 |= 0x2F;    //Terry
        SPI_WRITE(ICM2060X_REG_FIFO_CONFIG_1, mTask.fifo_cfg1);
    }
#endif

    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

#if DEBUG_PHASE
static int ICM2060Xregdump(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    SPI_READ(ICM2060X_REG_SAMRT_DIV, 91, &mTask.debugintBuffer);
    //SPI_READ(0x31, 60, &mTask.regdumpBuffer);
    //SPI_READ(ICM2060X_REG_DEVID, 1, &mTask.whoami);
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int ICM2060Xregresult(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                             void *inBuf, uint8_t inSize, uint8_t elemInSize,
                             void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    uint8_t *reg_data = &mTask.debugintBuffer[1];
    uint8_t regaddr = ICM2060X_REG_SAMRT_DIV ;
    uint8_t i = 0;
    while (regaddr <= ICM2060X_REG_FIFO_COUNTL) {
        osLog(LOG_INFO, "regaddr 0x%x regvalue 0x%x\n", regaddr, reg_data[i]);
        regaddr++;
        i++;
    }

    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}
#endif
static int ICM2060XgRate(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                         void *inBuf, uint8_t inSize, uint8_t elemInSize,
                         void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    int odr = 0;
    uint8_t regValue = 0x00;
    uint32_t sampleRate = 0;
    uint32_t maxRate = 0;
    bool accelOdrChanged = false;
    struct accGyroCntlPacket cntlPacket;

    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "ICM2060XgRate, rx inSize and elemSize error\n");
        return -1;
    }

    mTask.sensors[ACC].rate = cntlPacket.rate;
    mTask.sensors[ACC].latency = cntlPacket.latency;

    if (0 == mTask.sensors[ACC].preRealRate) { //Terry
        mTask.sensors[ACC].needDiscardSample = true;
        mTask.sensors[ACC].samplesToDiscard = SAMPLE_TO_DISCARD;
    }

    odr = ICM2060XCalcuOdr(&mTask.sensors[ACC].rate, &sampleRate);
    mTask.sensors[ACC].preRealRate = sampleRate;

    /*if gyr configed ,compare maxRate with acc and gyr rate*/
    if (mTask.sensors[GYR].configed) {
        maxRate = max(sampleRate, mTask.sensors[GYR].preRealRate);   // choose with preRealRate
        if ((maxRate != mTask.sensors[ACC].hwRate) || (maxRate != mTask.sensors[GYR].hwRate)) {
            mTask.sensors[ACC].hwRate = maxRate;
            mTask.sensors[GYR].hwRate = maxRate;
            odr = ICM2060XCalcuOdr(&maxRate, &sampleRate);
            if (15 == odr)
                regValue = 66;
            else
                regValue = ODR2SMPLRT_DIV(odr);

            SPI_WRITE(ICM2060X_REG_SAMRT_DIV, regValue);

            accelOdrChanged = true;
        } else {
            accelOdrChanged = false;
        }
    } else {
        if ((sampleRate != mTask.sensors[ACC].hwRate)) {
            mTask.sensors[ACC].hwRate = sampleRate;
            if (15 == odr)
                regValue = 66;
            else
                regValue = ODR2SMPLRT_DIV(odr);

            SPI_WRITE(ICM2060X_REG_SAMRT_DIV, regValue);
            accelOdrChanged = true;
        } else {
            accelOdrChanged = false;
        }
    }
    osLog(LOG_INFO, "set odr %d,mTask.sensors[ACC].hwRate %d,rlatency %d\n", odr, mTask.sensors[ACC].hwRate,
          mTask.sensors[ACC].latency);
    registerAccGyroFifoInfo((mTask.sensors[ACC].hwRate == 0) ? 0 : 1024000000000 / mTask.sensors[ACC].hwRate,
                            (mTask.sensors[GYR].hwRate == 0) ? 0 : 1024000000000 / mTask.sensors[GYR].hwRate);
    mTask.sensors[ACC].configed = true;

    mTask.watermark = ICM2060XCalcuWm();

    ICM2060XConfigFifo(accelOdrChanged);
    mt_eint_unmask(mTask.hw->eint_num);
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int ICM2060XgyEnable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                            void *inBuf, uint8_t inSize, uint8_t elemInSize,
                            void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;

    osLog(LOG_INFO, "ICM2060XgyEnable power mode=%d\n", mTask.sensors[GYR].powered);

    //Go back from sleep mode to normal mode if necessary
    if (false == mTask.sensors[ACC].powered && false == mTask.sensors[GYR].powered) {
        osLog(LOG_INFO, "ICM2060XgyEnable: gyro go out sleep\n");
        mTask.pwr_mgmr_config &= ~ICM2060X_SLEEP;
        SPI_WRITE(ICM2060X_PWR_MGMT_1, mTask.pwr_mgmr_config);
    }
    mTask.sensors[GYR].powered = true;
    mTask.pwr_mgmr_config2 &= ~BIT_PWR_GYRO_STBY;
    SPI_WRITE(ICM2060X_PWR_MGMT_2, mTask.pwr_mgmr_config2,
              90000); //set 90ms for gyro start-up time, note:at least 200us here to sync fly changes
    ret = spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
    return ret;
}

static int ICM2060XgyDisable(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                             void *inBuf, uint8_t inSize, uint8_t elemInSize,
                             void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    int odr = 0;
    uint8_t regValue = 0x00;
    uint32_t sampleRate = 0;
    struct accGyroCntlPacket cntlPacket;
    osLog(LOG_INFO, "ICM2060XgyDisable\n");

    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "ICM2060XAccPowerOff, rx inSize and elemSize error\n");
        return -1;
    }

    mTask.sensors[GYR].rate = 0;
    mTask.sensors[GYR].preRealRate = 0;
    mTask.sensors[GYR].hwRate = 0;
    mTask.sensors[GYR].latency = SENSOR_LATENCY_NODATA;
    mTask.sensors[GYR].needDiscardSample = false;  //Terry
    mTask.sensors[GYR].samplesToDiscard = 0;       //Terry

    /* enter off mode */
    if ((mTask.sensors[ACC].powered == false) && (mTask.sensors_handle[ANYMO].powered == false)) {
        mt_eint_mask(mTask.hw->eint_num);
        /* acc & gyro  stand by , sleep mode in idle */
        SPI_WRITE(ICM2060X_PWR_MGMT_2, mTask.pwr_mgmr_config2 | BIT_PWR_GYRO_STBY | BIT_PWR_ACCEL_STBY);
        SPI_WRITE(ICM2060X_PWR_MGMT_1, mTask.pwr_mgmr_config | ICM2060X_SLEEP);

        /*update globle status pwr_mgmr_config2, pwr_mgmr_config2 */

        mTask.pwr_mgmr_config2 |= BIT_PWR_GYRO_STBY | BIT_PWR_ACCEL_STBY;
        mTask.pwr_mgmr_config  |= ICM2060X_SLEEP;
    } else if (mTask.sensors[ACC].powered == true) {  //  update ACC old ord
        if (mTask.sensors[ACC].hwRate != mTask.sensors[ACC].preRealRate) {
            mTask.sensors[ACC].hwRate = mTask.sensors[ACC].preRealRate;
            //terry odr=CalcuOdr ?
            odr = ICM2060XCalcuOdr(&mTask.sensors[ACC].hwRate, &sampleRate);
            if (15 == odr)
                regValue = 66;
            else
                regValue = ODR2SMPLRT_DIV(odr);

            SPI_WRITE(ICM2060X_REG_SAMRT_DIV, regValue);
            osLog(LOG_INFO, "acc change rate to preRealRate:%d\n", mTask.sensors[ACC].hwRate);
        }
        // GYRO off
        SPI_WRITE(ICM2060X_PWR_MGMT_2, mTask.pwr_mgmr_config2 | BIT_PWR_GYRO_STBY);
        /*update globle status pwr_mgmr_config2, pwr_mgmr_config2 */
        mTask.pwr_mgmr_config2 |= BIT_PWR_GYRO_STBY ;

    } else if (mTask.sensors_handle[ANYMO].powered == true) {
        // GYRO off
        SPI_WRITE(ICM2060X_PWR_MGMT_2, mTask.pwr_mgmr_config2 | BIT_PWR_GYRO_STBY);
        /*update globle status pwr_mgmr_config2, pwr_mgmr_config2 */
        mTask.pwr_mgmr_config2 |= BIT_PWR_GYRO_STBY ;
    }

    registerAccGyroFifoInfo((mTask.sensors[ACC].hwRate == 0) ? 0 : 1024000000000 / mTask.sensors[ACC].hwRate,
                            (mTask.sensors[GYR].hwRate == 0) ? 0 : 1024000000000 / mTask.sensors[GYR].hwRate);

    mTask.watermark = ICM2060XCalcuWm();
    mTask.sensors[GYR].powered = false;
    mTask.sensors[GYR].configed = false;

    ICM2060XConfigFifo(false);
#if 0
    if (mTask.sensors[ACC].powered == true) {
        /* enable fifo INT */
        mTask.fifo_cfg1 |= 0x2F;    //Terry
        SPI_WRITE(ICM2060X_REG_FIFO_CONFIG_1, mTask.fifo_cfg1);
    }
#endif

    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int ICM2060XgyRate(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                          void *inBuf, uint8_t inSize, uint8_t elemInSize,
                          void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    int odr = 0;
    uint8_t regValue = 0x00;
    uint32_t sampleRate = 0;
    uint32_t maxRate = 0;
    bool gyroOdrChanged = false;
    struct accGyroCntlPacket cntlPacket;

    ret = rxControlInfo(&cntlPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "ICM2060XgRate, rx inSize and elemSize error\n");
        return -1;
    }

    mTask.sensors[GYR].rate = cntlPacket.rate;
    mTask.sensors[GYR].latency = cntlPacket.latency;

    if (0 == mTask.sensors[GYR].preRealRate) {   //Terry
        mTask.sensors[GYR].needDiscardSample = true;
        mTask.sensors[GYR].samplesToDiscard = SAMPLE_TO_DISCARD;
    }

    /*  get hw sample rate */
    odr = ICM2060XCalcuOdr(&mTask.sensors[GYR].rate, &sampleRate);

    mTask.sensors[GYR].preRealRate = sampleRate;

    /*if gyr configed ,compare maxRate with acc and gyr rate*/
    if (mTask.sensors[ACC].configed) {
        maxRate = max(sampleRate, mTask.sensors[ACC].preRealRate);
        if ((maxRate != mTask.sensors[ACC].hwRate) || (maxRate != mTask.sensors[GYR].hwRate)) {
            mTask.sensors[ACC].hwRate = maxRate;
            mTask.sensors[GYR].hwRate = maxRate;
            /* update new odr */
            odr = ICM2060XCalcuOdr(&maxRate, &sampleRate);
            if (15 == odr)
                regValue = 66;
            else
                regValue = ODR2SMPLRT_DIV(odr);

            SPI_WRITE(ICM2060X_REG_SAMRT_DIV, regValue);
            osLog(LOG_INFO, "ICM2060XgyRate: gyro config %d\n", regValue);
            gyroOdrChanged = true;
        } else {
            gyroOdrChanged = false;
        }
    } else {
        if ((sampleRate != mTask.sensors[GYR].hwRate)) {
            mTask.sensors[GYR].hwRate = sampleRate;
            if (15 == odr)
                regValue = 66;
            else
                regValue = ODR2SMPLRT_DIV(odr);
            SPI_WRITE(ICM2060X_REG_SAMRT_DIV, regValue);
            osLog(LOG_INFO, "ICM2060XgyRate-S: gyro config %d\n", regValue);
            gyroOdrChanged = true;
        } else {
            gyroOdrChanged = false;
        }
    }

    registerAccGyroFifoInfo((mTask.sensors[ACC].hwRate == 0) ? 0 : 1024000000000 / mTask.sensors[ACC].hwRate,
                            (mTask.sensors[GYR].hwRate == 0) ? 0 : 1024000000000 / mTask.sensors[GYR].hwRate);
    mTask.sensors[GYR].configed = true;

    /* watermark update */
    mTask.watermark = ICM2060XCalcuWm();
    ICM2060XConfigFifo(gyroOdrChanged);

    mt_eint_unmask(mTask.hw->eint_num);
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int ICM2060XAccCali(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    float bias[AXES_NUM] = {0};
    mTask.sensors[ACC].startCali = true;

    osLog(LOG_INFO, "ICM2060XAccCali %d\n", mTask.sensors[ACC].startCali);
    Acc_init_calibration(bias);

    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int ICM2060XGyroCali(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                            void *inBuf, uint8_t inSize, uint8_t elemInSize,
                            void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    float slope[AXES_NUM] = {0};
    float intercept[AXES_NUM] = {0};
    mTask.sensors[GYR].startCali = true;

    osLog(LOG_INFO, "ICM2060XGyroCali %d\n", mTask.sensors[GYR].startCali);
    Gyro_init_calibration(slope, intercept);

    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int ICM2060XAccCfgCali(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                              void *inBuf, uint8_t inSize, uint8_t elemInSize,
                              void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    struct accGyroCaliCfgPacket caliCfgPacket;

    ret = rxCaliCfgInfo(&caliCfgPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);

    if (ret < 0) {
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "accHwCaliCheck, rx inSize and elemSize error\n");
        return -1;
    }
    osLog(LOG_INFO, "ICM2060XAccCfgCali: cfgData[0]:%d, cfgData[1]:%d, cfgData[2]:%d\n",
          caliCfgPacket.caliCfgData[0], caliCfgPacket.caliCfgData[1], caliCfgPacket.caliCfgData[2]);

    mTask.sensors[ACC].staticCali[0] = (float)caliCfgPacket.caliCfgData[0] / 1000;
    mTask.sensors[ACC].staticCali[1] = (float)caliCfgPacket.caliCfgData[1] / 1000;
    mTask.sensors[ACC].staticCali[2] = (float)caliCfgPacket.caliCfgData[2] / 1000;

    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static int ICM2060XGyroCfgCali(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                               void *inBuf, uint8_t inSize, uint8_t elemInSize,
                               void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;
    struct accGyroCaliCfgPacket caliCfgPacket;

    ret = rxCaliCfgInfo(&caliCfgPacket, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);

    if (ret < 0) {
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "gyroHwCaliCheck, rx inSize and elemSize error\n");
        return -1;
    }

    osLog(LOG_INFO, "ICM2060XGyroCfgCali: cfgData[0]:%d, cfgData[1]:%d, cfgData[2]:%d\n",
          caliCfgPacket.caliCfgData[0], caliCfgPacket.caliCfgData[1], caliCfgPacket.caliCfgData[2]);

    mTask.sensors[GYR].staticCali[0] = (float)caliCfgPacket.caliCfgData[0] / 1000;
    mTask.sensors[GYR].staticCali[1] = (float)caliCfgPacket.caliCfgData[1] / 1000;
    mTask.sensors[GYR].staticCali[2] = (float)caliCfgPacket.caliCfgData[2] / 1000;

    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static void accGetCalibration(int32_t *cali, int32_t size)
{
    cali[AXIS_X]  = mTask.accHwCali[AXIS_X];
    cali[AXIS_Y]  = mTask.accHwCali[AXIS_Y];
    cali[AXIS_Z]  = mTask.accHwCali[AXIS_Z];
    //  osLog(LOG_INFO, "accGetCalibration cali x:%d, y:%d, z:%d\n", cali[AXIS_X], cali[AXIS_Y], cali[AXIS_Z]);
}
static void accSetCalibration(int32_t *cali, int32_t size)
{
    mTask.accHwCali[AXIS_X] = cali[AXIS_X];
    mTask.accHwCali[AXIS_Y] = cali[AXIS_Y];
    mTask.accHwCali[AXIS_Z] = cali[AXIS_Z];
    osLog(LOG_INFO, "accSetCalibration cali x:%d, y:%d, z:%d\n", mTask.accHwCali[AXIS_X],
          mTask.accHwCali[AXIS_Y], mTask.accHwCali[AXIS_Z]);
}
static void accGetData(void *sample)
{
    struct TripleAxisDataPoint *tripleSample = (struct TripleAxisDataPoint *)sample;
    tripleSample->ix = mTask.accFactoryData.ix;
    tripleSample->iy = mTask.accFactoryData.iy;
    tripleSample->iz = mTask.accFactoryData.iz;

    osLog(LOG_INFO, "accGetData x:%d, y:%d, z:%d\n", tripleSample->ix, tripleSample->iy, tripleSample->iz);
}
static void gyroGetCalibration(int32_t *cali, int32_t size)
{
    cali[AXIS_X] = mTask.gyroHwCali[AXIS_X];
    cali[AXIS_Y] = mTask.gyroHwCali[AXIS_Y];
    cali[AXIS_Z] = mTask.gyroHwCali[AXIS_Z];
    //  osLog(LOG_INFO, "gyroGetCalibration cali x:%d, y:%d, z:%d\n", cali[AXIS_X], cali[AXIS_Y], cali[AXIS_Z]);
}
static void gyroSetCalibration(int32_t *cali, int32_t size)
{
    mTask.gyroHwCali[AXIS_X] = cali[AXIS_X];
    mTask.gyroHwCali[AXIS_Y] = cali[AXIS_Y];
    mTask.gyroHwCali[AXIS_Z] = cali[AXIS_Z];
    osLog(LOG_INFO, "gyroSetCalibration cali x:%d, y:%d, z:%d\n", mTask.gyroHwCali[AXIS_X],
          mTask.gyroHwCali[AXIS_Y], mTask.gyroHwCali[AXIS_Z]);
}
static void gyroGetData(void *sample)
{
    struct TripleAxisDataPoint *tripleSample = (struct TripleAxisDataPoint *)sample;
    tripleSample->ix = mTask.gyroFactoryData.ix;
    tripleSample->iy = mTask.gyroFactoryData.iy;
    tripleSample->iz = mTask.gyroFactoryData.iz;

    osLog(LOG_INFO, "gyroGetData x:%d, y:%d, z:%d\n", tripleSample->ix, tripleSample->iy, tripleSample->iz);
}

static void spiIsrCallBack(void *cookie, int err)
{
    if (err != 0) {
        osLog(LOG_ERROR, "lsm6dsm: spiIsrCallBack err\n");
        sensorFsmEnqueueFakeSpiEvt(mTask.spiCallBack, cookie, ERROR_EVT);
    } else {
        mTask.swSampleTime = rtcGetTime();
        sensorFsmEnqueueFakeSpiEvt(mTask.spiCallBack, cookie, SUCCESS_EVT);
    }
}

static int ICM2060XSample(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                          void *inBuf, uint8_t inSize, uint8_t elemInSize,
                          void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    int ret = 0;

    ret = rxTransferDataInfo(&mTask.dataInfo, inBuf, inSize, elemInSize, outBuf, outSize, elemOutSize);
    if (ret < 0) {
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, ERROR_EVT);
        osLog(LOG_ERROR, "ICM2060XSample, rx dataInfo error\n");
        return -1;
    }

    SPI_READ(ICM2060X_REG_FIFO_COUNTH, 2, &mTask.statusBuffer);
    mTask.spiCallBack = spiCallBack;

    return spiBatchTxRx(&mTask.mode, spiIsrCallBack, next_state, __FUNCTION__);
}

static int ICM2060XReadFifo(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                            void *inBuf, uint8_t inSize, uint8_t elemInSize,
                            void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    uint16_t tmp = 0;
    uint32_t maxRate = 0;

    /* use record mode: one record equal to 16Byte */
    mTask.fifoDataToRead = (mTask.statusBuffer[1] << 8) | mTask.statusBuffer[2];
    /* covert record num to bytes */
    //mTask.fifoDataToRead *= 16;//N/A for byte mode
#if DEBUG_FIFO_DATA
    osLog(LOG_INFO, "ICM2060XReadFifo :mTask.fifoDataToRead = %d,H %x, L %x\n", mTask.fifoDataToRead, mTask.statusBuffer[1],
          mTask.statusBuffer[2]);
#endif
    if (mTask.fifoDataToRead <= 0) {
        sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
        osLog(LOG_ERROR, "ICM2060XReadFifo error mTask.fifoDataToRead = %d\n", mTask.fifoDataToRead);
        return 0;
    }
//    mTask.fifoDataToRead += 1;   //Terry, LiKang, read one more byte to eliminate double interrupt
    if (mTask.fifoDataToRead > ICM2060X_MAX_FIFO_SIZE) {
        tmp = mTask.fifoDataToRead - ICM2060X_MAX_FIFO_SIZE;
        mTask.fifoDataToRead = ICM2060X_MAX_FIFO_SIZE;
        tmp = tmp / FIFO_BYTE_MODE;
        maxRate = max(mTask.sensors[ACC].hwRate, mTask.sensors[GYR].hwRate);
        if (maxRate) {
            mTask.swSampleTime -= (1024000000000ULL / maxRate) * tmp;
            osLog(LOG_INFO, "out size tmp:%d fifo:%d\n", tmp, mTask.fifoDataToRead);
        }
    }

    SPI_READ(ICM2060X_REG_FIFO_R_W, mTask.fifoDataToRead, &mTask.regBuffer);
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static void parseRawData(struct accGyroData *data, uint8_t *buf, uint8_t sensorType)
{
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
    else if (sensorType == SENS_TYPE_GYRO)
        gyroGetCalibration(SwCali, 0);
    /* Use big endian mode */
    raw_data[AXIS_X] = (buf[1] | buf[0] << 8);
    raw_data[AXIS_Y] = (buf[3] | buf[2] << 8);
    raw_data[AXIS_Z] = (buf[5] | buf[4] << 8);

//    if (sensorType == SENS_TYPE_GYRO) {
//        osLog(LOG_ERROR, "GYRO: %d %d %d\n", raw_data[AXIS_X], raw_data[AXIS_Y], raw_data[AXIS_Z]);
//    }

    raw_data[AXIS_X] = raw_data[AXIS_X] + SwCali[AXIS_X];
    raw_data[AXIS_Y] = raw_data[AXIS_Y] + SwCali[AXIS_Y];
    raw_data[AXIS_Z] = raw_data[AXIS_Z] + SwCali[AXIS_Z];

    remap_data[mTask.cvt.map[AXIS_X]] = mTask.cvt.sign[AXIS_X] * raw_data[AXIS_X];
    remap_data[mTask.cvt.map[AXIS_Y]] = mTask.cvt.sign[AXIS_Y] * raw_data[AXIS_Y];
    remap_data[mTask.cvt.map[AXIS_Z]] = mTask.cvt.sign[AXIS_Z] * raw_data[AXIS_Z];

    if (sensorType == SENS_TYPE_ACCEL) {
        temp_data[AXIS_X] = (float)remap_data[AXIS_X] * KSCALE_ACC_8G_RANGE;
        temp_data[AXIS_Y] = (float)remap_data[AXIS_Y] * KSCALE_ACC_8G_RANGE;
        temp_data[AXIS_Z] = (float)remap_data[AXIS_Z] * KSCALE_ACC_8G_RANGE;

#if DEBUG_FIFO_DATA
        osLog(LOG_ERROR, "ACC: %f %f %f\n", (double)temp_data[AXIS_X], (double)temp_data[AXIS_Y], (double)temp_data[AXIS_Z]);
#endif
        if (UNLIKELY(mTask.sensors[ACC].startCali)) {
            status = Acc_run_factory_calibration_timeout(delta_time,
                     temp_data, calibrated_data_output, (int *)&mTask.sensors[ACC].accuracy, rtcGetTime());
            osLog(LOG_INFO, "ACC accuracy %d\n", (int *)&mTask.sensors[ACC].accuracy);
            if (status != 0) {
                mTask.sensors[ACC].startCali = false;
                if (status > 0) {
                    osLog(LOG_INFO, "ACC cali detect shake\n");
                    caliResult[AXIS_X] = (int32_t)(mTask.sensors[ACC].staticCali[AXIS_X] * 1000);
                    caliResult[AXIS_Y] = (int32_t)(mTask.sensors[ACC].staticCali[AXIS_Y] * 1000);
                    caliResult[AXIS_Z] = (int32_t)(mTask.sensors[ACC].staticCali[AXIS_Z] * 1000);
                    accGyroSendCalibrationResult(SENS_TYPE_ACCEL, (int32_t *)&caliResult[0], (uint8_t)status);
                } else
                    osLog(LOG_INFO, "ACC cali time out\n");
            } else if (mTask.sensors[ACC].accuracy == 3) {
                mTask.sensors[ACC].startCali = false;
                mTask.sensors[ACC].staticCali[AXIS_X] = calibrated_data_output[AXIS_X] - temp_data[AXIS_X];
                mTask.sensors[ACC].staticCali[AXIS_Y] = calibrated_data_output[AXIS_Y] - temp_data[AXIS_Y];
                mTask.sensors[ACC].staticCali[AXIS_Z] = calibrated_data_output[AXIS_Z] - temp_data[AXIS_Z];
                caliResult[AXIS_X] = (int32_t)(mTask.sensors[ACC].staticCali[AXIS_X] * 1000);
                caliResult[AXIS_Y] = (int32_t)(mTask.sensors[ACC].staticCali[AXIS_Y] * 1000);
                caliResult[AXIS_Z] = (int32_t)(mTask.sensors[ACC].staticCali[AXIS_Z] * 1000);
                accGyroSendCalibrationResult(SENS_TYPE_ACCEL, (int32_t *)&caliResult[0], (uint8_t)status);
                osLog(LOG_INFO,
                      "ACCEL cali done:caliResult[0]:%d, caliResult[1]:%d, caliResult[2]:%d,offset[0]:%f, offset[1]:%f, offset[2]:%f\n",
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

        //Need "m/s^2 * 1000" for factory calibration program
        //Need LSB in cali program = mTask.accFactoryData.ix * mInfo.sensitivity / mInfo.gain;
        mTask.accFactoryData.ix = (int32_t)(data->x * ACCELEROMETER_INCREASE_NUM_AP/*1000*/);
        mTask.accFactoryData.iy = (int32_t)(data->y * ACCELEROMETER_INCREASE_NUM_AP);
        mTask.accFactoryData.iz = (int32_t)(data->z * ACCELEROMETER_INCREASE_NUM_AP);

    } else if (sensorType == SENS_TYPE_GYRO) {
        temp_data[AXIS_X] = (float)remap_data[AXIS_X] * KSCALE_GYRO_2000_RANGE;
        temp_data[AXIS_Y] = (float)remap_data[AXIS_Y] * KSCALE_GYRO_2000_RANGE;
        temp_data[AXIS_Z] = (float)remap_data[AXIS_Z] * KSCALE_GYRO_2000_RANGE;

#if DEBUG_FIFO_DATA
        osLog(LOG_ERROR, "GYRO: %f %f %f\n", (double)temp_data[AXIS_X], (double)temp_data[AXIS_Y], (double)temp_data[AXIS_Z]);
#endif
        if (UNLIKELY(mTask.sensors[GYR].startCali)) {
            status = Gyro_run_factory_calibration_timeout(delta_time,
                     temp_data, calibrated_data_output, (int *)&mTask.sensors[GYR].accuracy, 0, rtcGetTime());
            osLog(LOG_INFO, "ACC gyro %d\n", (int *)&mTask.sensors[GYR].accuracy);
            if (status != 0) {
                mTask.sensors[GYR].startCali = false;
                if (status > 0) {
                    osLog(LOG_INFO, "GYRO cali detect shake\n");
                    caliResult[AXIS_X] = (int32_t)(mTask.sensors[GYR].staticCali[AXIS_X] * 1000);
                    caliResult[AXIS_Y] = (int32_t)(mTask.sensors[GYR].staticCali[AXIS_Y] * 1000);
                    caliResult[AXIS_Z] = (int32_t)(mTask.sensors[GYR].staticCali[AXIS_Z] * 1000);
                    accGyroSendCalibrationResult(SENS_TYPE_GYRO, (int32_t *)&caliResult[0], (uint8_t)status);
                } else
                    osLog(LOG_INFO, "GYRO cali time out\n");
            } else if (mTask.sensors[GYR].accuracy == 3) {
                mTask.sensors[GYR].startCali = false;
                mTask.sensors[GYR].staticCali[AXIS_X] = calibrated_data_output[AXIS_X] - temp_data[AXIS_X];
                mTask.sensors[GYR].staticCali[AXIS_Y] = calibrated_data_output[AXIS_Y] - temp_data[AXIS_Y];
                mTask.sensors[GYR].staticCali[AXIS_Z] = calibrated_data_output[AXIS_Z] - temp_data[AXIS_Z];
                caliResult[AXIS_X] = (int32_t)(mTask.sensors[GYR].staticCali[AXIS_X] * 1000);
                caliResult[AXIS_Y] = (int32_t)(mTask.sensors[GYR].staticCali[AXIS_Y] * 1000);
                caliResult[AXIS_Z] = (int32_t)(mTask.sensors[GYR].staticCali[AXIS_Z] * 1000);
                accGyroSendCalibrationResult(SENS_TYPE_GYRO, (int32_t *)&caliResult[0], (uint8_t)status);
                osLog(LOG_INFO,
                      "GYRO cali done: caliResult[0]:%d, caliResult[1]:%d, caliResult[2]:%d, offset[0]:%f, offset[1]:%f, offset[2]:%f\n",
                      caliResult[AXIS_X], caliResult[AXIS_Y], caliResult[AXIS_Z],
                      (double)mTask.sensors[GYR].staticCali[AXIS_X],
                      (double)mTask.sensors[GYR].staticCali[AXIS_Y],
                      (double)mTask.sensors[GYR].staticCali[AXIS_Z]);
            }
        }

        data->sensType = sensorType;

        data->x = temp_data[AXIS_X] + mTask.sensors[GYR].staticCali[AXIS_X];
        data->y = temp_data[AXIS_Y] + mTask.sensors[GYR].staticCali[AXIS_Y];
        data->z = temp_data[AXIS_Z] + mTask.sensors[GYR].staticCali[AXIS_Z];

        //Need "dps*GYROSCOPE_INCREASE_NUM_AP" for factory calibration program
        //Need LSB in cali program = mTask.accFactoryData.ix * mInfo.sensitivity / mInfo.gain;
        mTask.gyroFactoryData.ix =
            (int32_t)((float)data->x * GYROSCOPE_INCREASE_NUM_AP/*131*1000*/ / DEGREE_TO_RADIRAN_SCALAR/*pi/180*/);
        mTask.gyroFactoryData.iy =
            (int32_t)((float)data->y * GYROSCOPE_INCREASE_NUM_AP / DEGREE_TO_RADIRAN_SCALAR);
        mTask.gyroFactoryData.iz =
            (int32_t)((float)data->z * GYROSCOPE_INCREASE_NUM_AP / DEGREE_TO_RADIRAN_SCALAR);
    }

//    if (sensorType == SENS_TYPE_GYRO) {
//                osLog(LOG_ERROR, "GYRO_report: %f %f %f\n",
//                    (double)data->x, (double)data->y, (double)data->z);
//    }

    if (mTask.debug_trace) {
        switch (sensorType) {
            case SENS_TYPE_ACCEL:
                osLog(LOG_ERROR, "ACCEL:raw_data_x=%f, raw_data_y=%f, raw_data_z=%f\n",
                      (double)data->x, (double)data->y, (double)data->z);
                break;
            case SENS_TYPE_GYRO:
                osLog(LOG_ERROR, "GYRO:raw_data_x=%f, raw_data_y=%f, raw_data_z=%f\n",
                      (double)data->x, (double)data->y, (double)data->z);
                break;
            default:
                break;
        }
    }
}

static int ICM2060XConvert(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                           void *inBuf, uint8_t inSize, uint8_t elemInSize,
                           void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    struct accGyroData *data = mTask.accGyroPacket.outBuf;
    uint32_t i = 0;
    uint8_t accEventSize = 0;
    uint8_t gyroEventSize = 0;
    uint64_t swSampleTime = 0, realSampleTime = 0;
    uint8_t *fifo_data = &mTask.regBuffer[1];
    //uint32_t max_recv_packet = 60;   //Terry no need, sample

    for (i = 0; i < mTask.fifoDataToRead; i += FIFO_BYTE_MODE) {
        if ((accEventSize + gyroEventSize) < MAX_RECV_PACKET) {
            if (mTask.sensors[ACC].configed && mTask.sensors[ACC].powered) {
                if (mTask.sensors[ACC].samplesToDiscard)
                    mTask.sensors[ACC].samplesToDiscard--;
                else {
                    parseRawData(&data[accEventSize + gyroEventSize], &fifo_data[i], SENS_TYPE_ACCEL);
                    accEventSize++;
                }
            }
            if (mTask.sensors[GYR].configed && mTask.sensors[GYR].powered) {
                if (mTask.sensors[GYR].samplesToDiscard)
                    mTask.sensors[GYR].samplesToDiscard--;
                else {
                    parseRawData(&data[accEventSize + gyroEventSize], &fifo_data[i + 8], SENS_TYPE_GYRO);
                    gyroEventSize++;
                }
            }
        } else
            osLog(LOG_ERROR, "outBuf full, accEventSize = %d, gyroEventSize = %d\n", accEventSize, gyroEventSize);
    }

    /*if factory true , can't send to runtime cali in parseRawData in accGyro*/
    if (mTask.sensors[ACC].startCali || mTask.sensors[GYR].startCali) {
        accEventSize = 0;
        gyroEventSize = 0;
    }
    //osLog(LOG_INFO, "ICM2060XConvert, fifoDataToRead:%d, accEventSize:%d, gyroEventSize:%d\n",
    //mTask.fifoDataToRead, accEventSize, gyroEventSize);

    swSampleTime = mTask.swSampleTime;

    /* End Pre Process lastSampleTime and hwSampleTime */
    if (mTask.hwSampleTime != mTask.lastSampleTime) {
        realSampleTime = calcFakeInterruptTime(swSampleTime, mTask.hwSampleTime, mTask.lastSampleTime,
                                               mTask.sensors[ACC].hwRate, mTask.sensors[ACC].configed, accEventSize,
                                               mTask.sensors[GYR].hwRate, mTask.sensors[GYR].configed, gyroEventSize);
    } else {
        // if hwSampleTime equal to lastSample time, it does not need to calculate the Fake Interrupt time(because hwsample is not updated because of no current Interrupt comming)
        realSampleTime = swSampleTime;
    }
    //osLog(LOG_INFO, "ICM2060XConvert, swSampleTime=%llu, hwSampleTime=%llu, realSampleTime=%llu, lastSampleTime=%llu, now=%llu, unmask_eint_time=%llu\n",
    //swSampleTime, mTask.hwSampleTime, realSampleTime, mTask.lastSampleTime, rtcGetTime(), mTask.unmask_eint_time);

    //osLog(LOG_INFO, "hwT %llu, SwT %llu,rT %llu,accS %d,gyroS %d", mTask.hwSampleTime,swSampleTime,realSampleTime,accEventSize,gyroEventSize);
    mTask.hwSampleTime = realSampleTime;
    mTask.lastSampleTime = realSampleTime;

    txTransferDataInfo(&mTask.dataInfo, accEventSize, gyroEventSize, realSampleTime, data, 0);
    //accGyro CHIP_HW_INT_STATUS_CHECK pending by nowState:0
    mt_eint_unmask(mTask.hw->eint_num);
    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static void ICM2060XIsr1(int arg)
{
    if (mTask.latch_time_id < 0) {
        mTask.hwSampleTime = rtcGetTime();
        //osLog(LOG_ERROR, "ICM2060XIsr1, mTask.latch_time_id < 0, mTask.hwSampleTime=%llu\n", mTask.hwSampleTime);
    } else {
        mTask.hwSampleTime = get_latch_time_timestamp(mTask.latch_time_id);
        //osLog(LOG_ERROR, "ICM2060XIsr1, mTask.hwSampleTime=%llu, now_rtc=%llu\n", mTask.hwSampleTime, rtcGetTime());
    }
    accGyroHwIntCheckStatus();
}

static int ICM2060XIntStatusCheck(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                  void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                  void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    SPI_READ(ICM2060X_REG_FIFO_WM_STATUS, 1, &mTask.statusBuffer);
    SPI_READ(ICM2060X_REG_INT_STATUS, 1, &mTask.int_statusBuffer);
    SPI_READ(ICM2060X_REG_FIFO_COUNTH, 2, &mTask.regBuffer);
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int ICM2060XIntHandling(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                               void *inBuf, uint8_t inSize, uint8_t elemInSize,
                               void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    uint8_t wm_status = mTask.statusBuffer[1];
    uint8_t int_status = mTask.int_statusBuffer[1];

#if DEBUG_FIFO_DATA
    osLog(LOG_INFO, "ICM2060XIntHandling,  WM INT:0x%x, int_status:0x%x\n", wm_status, int_status);
    //osLog(LOG_INFO, "fifo H %x  L %x \n", mTask.regBuffer[2], mTask.regBuffer[1]);
#endif

#if SUPPORT_ANYMO
    union EmbeddedDataPoint trigger_axies;
    if (mTask.sensors_handle[ANYMO].powered) {
        if (int_status & (BIT_WOM_X_INT | BIT_WOM_Y_INT | BIT_WOM_Z_INT)) {
            trigger_axies.idata = (int_status & (BIT_WOM_X_INT | BIT_WOM_Y_INT | BIT_WOM_Z_INT));
            osLog(LOG_INFO, "Detected any motion\n");
            osEnqueueEvt(EVT_SENSOR_ANY_MOTION, trigger_axies.vptr, NULL);
        }
    }
#endif

    if (wm_status & BIT_FIFO_WM_INT) {
#if DEBUG_FIFO_DATA
        osLog(LOG_INFO, "Detected fifo watermark\n");
#endif
        /*when no app require data for acc & gyro, but happen WM INT, there seems something conflict logic when happen */
        if (!mTask.sensors[ACC].configed && !mTask.sensors[GYR].configed) {
            osLog(LOG_INFO, "Unexpected FIFO WM INTR fired\n");
            // reset fifo
            ICM2060XFIFO_Restart();
            mt_eint_unmask(mTask.hw->eint_num);
            return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
        } else {
            accGyroInterruptOccur();
        }
    } else if (int_status & BIT_FIFO_OFLOW_INT) {
        // reset fifo
        osLog(LOG_INFO, "fifo over flow\n");
        ICM2060XFIFO_Restart_Read();
        mt_eint_unmask(mTask.hw->eint_num);
        return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
    } else
        mt_eint_unmask(mTask.hw->eint_num);

    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);

    return 0;
}

static int ICM2060XEintRegistration(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                    void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                    void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    mt_eint_dis_hw_debounce(mTask.hw->eint_num);
    mt_eint_registration(mTask.hw->eint_num, EDGE_SENSITIVE, HIGH_LEVEL_TRIGGER, ICM2060XIsr1, EINT_INT_UNMASK,
                         EINT_INT_AUTO_UNMASK_OFF);
    sensorFsmEnqueueFakeSpiEvt(spiCallBack, next_state, SUCCESS_EVT);
    return 0;
}

static void ICM2060XSetDebugTrace(int32_t trace)
{
    mTask.debug_trace = trace;
    osLog(LOG_ERROR, "%s ==> trace:%d\n", __func__, mTask.debug_trace);
}

#if SUPPORT_ANYMO
static int anyMotionPowerOn(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                            void *inBuf, uint8_t inSize, uint8_t elemInSize,
                            void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "Enter anyMotionPowerOn\n");

    if (false == mTask.sensors[ACC].powered && false == mTask.sensors[GYR].powered) {
        osLog(LOG_INFO, "ICM2060XgyEnable: gyro go out sleep\n");
        mTask.pwr_mgmr_config &= ~ICM2060X_SLEEP;
        SPI_WRITE(ICM2060X_PWR_MGMT_1, mTask.pwr_mgmr_config);
        // if acc & gyro are all off, set sample rate to 10hz , otherwise do no change sample dive
        SPI_WRITE(ICM2060X_REG_SAMRT_DIV, 99);
    }

    if (false == mTask.sensors[ACC].configed) {
        // enable acc
        mTask.sensors[ACC].powered = true;
        mTask.pwr_mgmr_config2 &= ~BIT_PWR_ACCEL_STBY;
        SPI_WRITE(ICM2060X_PWR_MGMT_2, mTask.pwr_mgmr_config2,
                  20000); //set 20ms for accel start-up time, note:at least 200us here to sync fly changes
    }

    /*config wom mode to enable */
    mTask.smd_cfg_reg69 |= (BIT_ACC_INTEL_EN | BIT_ACC_INTEL_MODE | BIT_WOM_INT_MODE);
    SPI_WRITE(ICM2060X_REG_ACCEL_INTEL_CTRL, mTask.smd_cfg_reg69);

    /*config WOM threshold */
    SPI_WRITE(ICM2060X_REG_WOM_THR_X, DEF_WOM_THRESHOLD);
    SPI_WRITE(ICM2060X_REG_WOM_THR_Y, DEF_WOM_THRESHOLD);
    SPI_WRITE(ICM2060X_REG_WOM_THR_Z, DEF_WOM_THRESHOLD);
    SPI_WRITE(ICM2060X_REG_WOM_THR, DEF_WOM_THRESHOLD);

    /* enable interrupt of wom  */
    mTask.int_src0 |= BIT_WOM_EN;
    SPI_WRITE(ICM2060X_REG_INT_ENABLE, mTask.int_src0);

    mTask.sensors_handle[ANYMO].powered = true;

    mt_eint_unmask(mTask.hw->eint_num);

    return  spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static int anyMotionPowerOff(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                             void *inBuf, uint8_t inSize, uint8_t elemInSize,
                             void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    osLog(LOG_INFO, "anyMotionPowerOff\n");
    mTask.sensors_handle[ANYMO].powered = false;
    mTask.sensors_handle[ANYMO].configed = false;

    /*config wom mode to enable */
    mTask.smd_cfg_reg69 &= ~(BIT_ACC_INTEL_EN | BIT_ACC_INTEL_MODE | BIT_WOM_INT_MODE);
    SPI_WRITE(ICM2060X_REG_ACCEL_INTEL_CTRL, mTask.smd_cfg_reg69);

    /* disable interrupt of wom  */
    mTask.int_src0 &= ~ BIT_WOM_EN;
    SPI_WRITE(ICM2060X_REG_INT_ENABLE, mTask.int_src0);

    if (false == mTask.sensors[ACC].configed) {
        // disable acc
        mTask.sensors[ACC].powered = false;
        mTask.pwr_mgmr_config2 |= BIT_PWR_ACCEL_STBY;
        SPI_WRITE(ICM2060X_PWR_MGMT_2,
                  mTask.pwr_mgmr_config2); //set 20ms for accel start-up time, note:at least 200us here to sync fly changes
    }

    if ((mTask.sensors[ACC].powered == false) && (mTask.sensors_handle[ANYMO].powered == false)) {
        mt_eint_mask(mTask.hw->eint_num);
        /*  sleep mode in idle */

        SPI_WRITE(ICM2060X_PWR_MGMT_1, mTask.pwr_mgmr_config | ICM2060X_SLEEP);
        mTask.pwr_mgmr_config  |= ICM2060X_SLEEP;
    }
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

#endif


static void accGetSensorInfo(struct sensorInfo_t *data)
{
    strncpy(data->name, ACC_NAME, sizeof(data->name));
}

static void gyroGetSensorInfo(struct sensorInfo_t *data)
{
    strncpy(data->name, GYRO_NAME, sizeof(data->name));
}

static void sensorCoreRegistration(void)
{
    struct sensorCoreInfo mInfo;
    osLog(LOG_INFO, "ICM2060XRegisterCore\n");

    memset(&mInfo, 0x00, sizeof(struct sensorCoreInfo));
    /* Register sensor Core */
    mInfo.sensType = SENS_TYPE_ACCEL;
    mInfo.gain = GRAVITY_EARTH_1000;
    mInfo.sensitivity = mTask.sensors[ACC].sensitivity;
    mInfo.cvt = mTask.cvt;
    mInfo.getCalibration = accGetCalibration;
    mInfo.setCalibration = accSetCalibration;
    mInfo.getData = accGetData;
    mInfo.setDebugTrace = ICM2060XSetDebugTrace;
    mInfo.getSensorInfo = accGetSensorInfo;
    sensorCoreRegister(&mInfo);

    memset(&mInfo, 0x00, sizeof(struct sensorCoreInfo));
    mInfo.sensType = SENS_TYPE_GYRO;
    mInfo.gain = GYROSCOPE_INCREASE_NUM_AP;
    mInfo.sensitivity = mTask.sensors[GYR].sensitivity;
    mInfo.cvt = mTask.cvt;
    mInfo.getCalibration = gyroGetCalibration;
    mInfo.setCalibration = gyroSetCalibration;
    mInfo.getData = gyroGetData;
    mInfo.getSensorInfo = gyroGetSensorInfo;
    sensorCoreRegister(&mInfo);
}

static int ICM2060XSensorRegistration(I2cCallbackF i2cCallBack, SpiCbkF spiCallBack, void *next_state,
                                      void *inBuf, uint8_t inSize, uint8_t elemInSize,
                                      void *outBuf, uint8_t *outSize, uint8_t *elemOutSize)
{
    //record ICM2060X_REG_ACCEL_INTEL_CTRL
    mTask.smd_cfg_reg69 = mTask.statusBuffer[1];

    osLog(LOG_INFO, "ICM2060XEintRegistration interrupt status 0x%x\n", mTask.regBuffer[1]);
    //enable fifo over flow int
    SPI_WRITE(ICM2060X_REG_INT_ENABLE, mTask.regBuffer[1] | BIT_FIFO_OFLOW_EN);
    mTask.int_src0 = mTask.regBuffer[1] | BIT_FIFO_OFLOW_EN;

    sensorCoreRegistration();
    return spiBatchTxRx(&mTask.mode, spiCallBack, next_state, __FUNCTION__);
}

static struct sensorFsm ICM2060XFsm[] = {
    /* INT handle */
    sensorFsmCmd(STATE_HW_INT_STATUS_CHECK, STATE_HW_INT_HANDLING, ICM2060XIntStatusCheck),
    sensorFsmCmd(STATE_HW_INT_HANDLING, STATE_HW_INT_HANDLING_DONE, ICM2060XIntHandling),
    /* sample */
    sensorFsmCmd(STATE_SAMPLE, STATE_FIFO, ICM2060XSample),
    sensorFsmCmd(STATE_FIFO, STATE_CONVERT, ICM2060XReadFifo),
    sensorFsmCmd(STATE_CONVERT, STATE_SAMPLE_DONE, ICM2060XConvert),

    sensorFsmCmd(STATE_ACC_ENABLE, STATE_ACC_ENABLE_DONE, ICM2060XgEnable),
    sensorFsmCmd(STATE_ACC_DISABLE, STATE_ACC_DISABLE_DONE, ICM2060XgDisable),
#if DEBUG_PHASE
    sensorFsmCmd(STATE_ACC_RATECHG, STATE_REGDUMP_READ , ICM2060XgRate),
    sensorFsmCmd(STATE_REGDUMP_READ, STATE_REGDUMP_RESULT , ICM2060Xregdump),
    sensorFsmCmd(STATE_REGDUMP_RESULT, STATE_ACC_RATECHG_DONE , ICM2060Xregresult),
#else
    sensorFsmCmd(STATE_ACC_RATECHG, STATE_ACC_RATECHG_DONE, ICM2060XgRate),
#endif
    sensorFsmCmd(STATE_ACC_CALI, STATE_ACC_CALI_DONE, ICM2060XAccCali),
    sensorFsmCmd(STATE_ACC_CFG, STATE_ACC_CFG_DONE, ICM2060XAccCfgCali),

    sensorFsmCmd(STATE_GYRO_ENABLE, STATE_GYRO_ENABLE_DONE, ICM2060XgyEnable),
    sensorFsmCmd(STATE_GYRO_DISABLE, STATE_GYRO_DISABLE_DONE, ICM2060XgyDisable),
    sensorFsmCmd(STATE_GYRO_RATECHG, STATE_GYRO_RATECHG_DONE, ICM2060XgyRate),
    sensorFsmCmd(STATE_GYRO_CALI, STATE_GYRO_CALI_DONE, ICM2060XGyroCali),
    sensorFsmCmd(STATE_GYRO_CFG, STATE_GYRO_CFG_DONE, ICM2060XGyroCfgCali),

    /* init state */
    sensorFsmCmd(STATE_SW_RESET, STATE_SW_RESET_W, ICM2060XResetRead),
    sensorFsmCmd(STATE_SW_RESET_W, STATE_INT_STATUS, ICM2060XResetWrite),
    sensorFsmCmd(STATE_INT_STATUS, STATE_RESET_CHECK, ICM2060XResetRead),
    sensorFsmCmd(STATE_RESET_CHECK, STATE_CLKSEL_REG_READ, ICM2060XResetCheck),
    sensorFsmCmd(STATE_CLKSEL_REG_READ, STATE_CLKSEL_REG_WRITE, ICM2060X_PWRMGMT_Read),
    sensorFsmCmd(STATE_CLKSEL_REG_WRITE, STATE_POWER_R, ICM2060XClkSelWrite),
    sensorFsmCmd(STATE_POWER_R, STATE_ENPOWER_W, ICM2060X_PWRMGMT_Read),
    sensorFsmCmd(STATE_ENPOWER_W, STATE_POWER2_R , ICM2060XPowerEnableWrite),
    sensorFsmCmd(STATE_POWER2_R, STATE_INIT_REG, ICM2060X_PWRMGMT2_Read),
    sensorFsmCmd(STATE_INIT_REG, STATE_SENSOR_REGISTRATION, ICM2060XInitConfig),
    sensorFsmCmd(STATE_SENSOR_REGISTRATION, STATE_EINT_REGISTRATION, ICM2060XSensorRegistration),
    sensorFsmCmd(STATE_EINT_REGISTRATION, STATE_INIT_DONE, ICM2060XEintRegistration),
    /* For Anymotion */
#if SUPPORT_ANYMO
    sensorFsmCmd(STATE_ANYMO_ENABLE, STATE_ANYMO_ENABLE_DONE, anyMotionPowerOn),
    sensorFsmCmd(STATE_ANYMO_DISABLE, STATE_ANYMO_DISABLE_DONE, anyMotionPowerOff),
#endif
};

int ICM2060XInit(void)
{
    int ret = 0;
    uint8_t txData[2] = {0}, rxData[2] = {0};

    ICM2060XDebugPoint = &mTask;
    insertMagicNum(&mTask.accGyroPacket);
    mTask.hw = get_cust_accGyro("icm20600");
    if (NULL == mTask.hw) {
        osLog(LOG_ERROR, "get_cust_acc_hw fail\n");
        return 0;
    }
    osLog(LOG_INFO, "acc spi_num: %d\n", mTask.hw->i2c_num);

    if (0 != (ret = sensorDriverGetConvert(mTask.hw->direction, &mTask.cvt))) {
        osLog(LOG_ERROR, "invalid direction: %d\n", mTask.hw->direction);
    }
    osLog(LOG_INFO, "acc map[0]:%d, map[1]:%d, map[2]:%d, sign[0]:%d, sign[1]:%d, sign[2]:%d\n\r",
          mTask.cvt.map[AXIS_X], mTask.cvt.map[AXIS_Y], mTask.cvt.map[AXIS_Z],
          mTask.cvt.sign[AXIS_X], mTask.cvt.sign[AXIS_Y], mTask.cvt.sign[AXIS_Z]);

    mTask.latch_time_id = alloc_latch_time();
    enable_latch_time(mTask.latch_time_id, mTask.hw->eint_num);

    mTask.int_src0 = 0;
    mTask.smd_cfg_reg69 = 0;
    mTask.acc_cfg0 = 0;
    mTask.gyro_cfg0 = 0;
    mTask.pwr_mgmr_config = 0;
    mTask.pwr_mgmr_config2 = 0;

    memset(&mTask.sensors[ACC], 0, sizeof(struct ICM2060XSensor));
    memset(&mTask.sensors[GYR], 0, sizeof(struct ICM2060XSensor));
    mTask.sensors[ACC].latency = SENSOR_LATENCY_NODATA;
    mTask.sensors[GYR].latency = SENSOR_LATENCY_NODATA;
    //Only applicable for factory calibration, refer to sensor spec
    mTask.sensors[ACC].sensitivity = (float)65536 / (8 * 2);//LSB/g sensitivity,8g acc
    mTask.sensors[GYR].sensitivity = (float)65536 / (2000 * 2);//LSB/dps sensitivity

    mTask.mode.speed = 5000000;
    mTask.mode.bitsPerWord = 8;
    mTask.mode.cpol = SPI_CPOL_IDLE_HI;
    mTask.mode.cpha = SPI_CPHA_TRAILING_EDGE;
    mTask.mode.nssChange = true;
    mTask.mode.format = SPI_FORMAT_MSB_FIRST;
    mTask.mWbufCnt = 0;
    mTask.mRegCnt = 0;

    spiMasterRequest(mTask.hw->i2c_num, &mTask.spiDev);
    txData[0] = ICM2060X_REG_DEVID | 0x80;
    for (uint8_t i = 0; i < 3;) {
        ret = spiMasterRxTxSync(mTask.spiDev, rxData, txData, 2);
        if (ret >= 0 && (rxData[1] == ICM20600_WHOAMI))
            break;
        ++i;
        if (i >= 3) {
            ret = -1;
            spiMasterRelease(mTask.spiDev);
            disable_latch_time(mTask.latch_time_id);
            free_latch_time(mTask.latch_time_id);
            goto err_out;
        }
    }
    osLog(LOG_INFO, "ICM20600 success: %02x\n", rxData[1]);

    accSensorRegister();
    gyroSensorRegister();
#if SUPPORT_ANYMO
    anyMotionSensorRegister();
#endif
    registerAccGyroInterruptMode(ACC_GYRO_FIFO_INTERRUPTIBLE);
    registerAccGyroDriverFsm(ICM2060XFsm, ARRAY_SIZE(ICM2060XFsm));

err_out:
    return ret;
}
#ifndef CFG_OVERLAY_INIT_SUPPORT
MODULE_DECLARE(icm20600, SENS_TYPE_ACCEL, ICM2060XInit);
#else
#include "mtk_overlay_init.h"
OVERLAY_DECLARE(icm20600, OVERLAY_ID_ACCGYRO, ICM2060XInit);
#endif
