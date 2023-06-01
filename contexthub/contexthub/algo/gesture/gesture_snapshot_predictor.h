#ifndef _GESTURE_SNAPSHOT_PREDICTOR_H_
#define _GESTURE_SNAPSHOT_PREDICTOR_H_

#include "rf_function.h"
#include "gesture_private.h"

void snapshot_predictor(uint32_t time_stamp);
void gesture_get_snapshot_result_ptr(gesture_notify_result_t *gesture_results);

// Get ptr of gesture random forest parameter function
#ifndef GESTURE_TREE_NON
Tree *get_gesture_snapshot_rf_para_ptr();
#else
Tree_None_Perfect *get_gesture_snapshot_rf_para_ptr();
#endif

RandomForestSetting *get_gesture_snapshot_rf_setting_ptr();

#endif
