#ifndef AF6133E_H
#define AF6133E_H

#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <platform.h>

#define CFG_MAG_CALIBRATION_IN_AP
//#define AF6133E_SET_OFFSET
//#define AF6133E_TEMP_COMP
#define AF6133E_SELF_TEST
//#define AF6133E_LC_FILTER
#define AF6133E_SENHUB_DEBUG
#define MAG_BIST_LOOP_5_TIMES

/* Debug trace */
#define FUNC_MASK_BISC        0x01
#define FUNC_MASK_GAIN        0x02
#define FUNC_MASK_COEF        0x04
#define FUNC_MASK_TEMP        0x08
#define FUNC_MASK_DEBUG       0x10
#define FUNC_MASK_LC_FILTER   0x20

#define FUNC_MASK_ACTIVE      (FUNC_MASK_GAIN+FUNC_MASK_COEF+FUNC_MASK_LC_FILTER+FUNC_MASK_DEBUG)

#define I2C_SPEED             400000

#ifdef AF6133E_LC_FILTER
#define LC_FILTER_THRESHOLD   30
#define LC_FILTER_NUMBER      3
#endif

/* AF6133E Delay Time */
#define  AF6133E_MEASURE_DELAY       5000000ULL //5ms
#define  AF6133E_BIST_DELAY          6000000ULL //6ms
#define  AF6133E_FIELD_DELAY         1000000ULL //1ms

#define AF6133E_PID           0x68

/* Register Map */
#define REG_PCODE             0x00
#define REG_DATA              0x03
#define REG_MEASURE           0x0A
#define REG_RANGE             0x0B
#define REG_I2C_CHECK         0x10
#define REG_SW_RESET          0x11
#define REG_AVG               0x13
#define REG_SR_MODE           0x14
#define REG_AVG_2ND           0x15
#define REG_OSR               0x16
#define REG_LLPF              0x17
#define REG_OSC_FREQ          0x19
#define REG_TEMP              0x21
#define REG_ADC_GAIN          0x2B
#define REG_XY_WAITING        0x32
#define REG_Z_WAITING         0x33
#define REG_CHOPPER           0x35
#define REG_TEST2             0x37
#define REG_FLIPPING          0x38

/* Temp. Coeff. */
#define TEMP_RESOLUTION       0.908
#define TEMP_SENS_COEFF_X     0.00369 //(0.00369 * TEMP_RESOLUTION)
#define TEMP_SENS_COEFF_Y     0.00403 //(0.00403 * TEMP_RESOLUTION)
#define TEMP_SENS_COEFF_Z     0.00434 //(0.00434 * TEMP_RESOLUTION)
#define TEMP_DELTA_THRESHOLD  2 //degree
#define TEMP_MF_NUM           5
#define TEMP_MF_IDX           (uint16_t)(TEMP_MF_NUM / 2)


/* BIST parameters */
#define GAIN_X_MIN            0.6
#define GAIN_X_MAX            6.6
#define GAIN_Y_MIN            0.6
#define GAIN_Y_MAX            6.0
#define GAIN_Z_MIN            0.6
#define GAIN_Z_MAX            6.0
#define BIST_COEFF_LIMIT_XY   0.087
#define BIST_COEFF_LIMIT_Z    0.707
#ifdef MAG_BIST_LOOP_5_TIMES
#define MAG_BIST_LOOP         5
#else
#define MAG_BIST_LOOP         3
#endif
/* Self-test */
#define BIST_TEST_MIN         100
#define BIST_TEST_MAX         700

/* Resolition */
#define RESOLUTION_M          0.15f

#define BIST_COEFF_X          0.794f
#define BIST_COEFF_Y          0.817f
#define BIST_COEFF_Z          0.785f

#define BIST_GAIN_COEFF_X (int32_t)(666.666 / BIST_COEFF_X)
#define BIST_GAIN_COEFF_Y (int32_t)(666.666 / BIST_COEFF_Y)
#define BIST_GAIN_COEFF_Z (int32_t)(666.666 / BIST_COEFF_Z)

#define vtc_fabs(x)    ((x<0) ? (-x) : (x))

#endif
