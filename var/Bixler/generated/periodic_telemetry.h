/* This file has been generated from /home/chorus/chorus/conf/messages.xml and /home/chorus/chorus/conf/telemetry/default_fixedwing_imu.xml */
/* Please DO NOT EDIT */

#ifndef _VAR_PERIODIC_H_
#define _VAR_PERIODIC_H_

#include "std.h"
#include "generated/airframe.h"


/* Macros for Ap process */
#ifdef PERIODIC_C_AP
#ifndef TELEMETRY_MODE_AP
#define TELEMETRY_MODE_AP 0
#endif
uint8_t telemetry_mode_Ap = TELEMETRY_MODE_AP;
#else /* PERIODIC_C_AP not defined (general header) */
extern uint8_t telemetry_mode_Ap;
#endif /* PERIODIC_C_AP */
#define TELEMETRY_MODE_Ap_default 0
#define PERIOD_AIRSPEED_Ap_0 (1)
#define PERIOD_ALIVE_Ap_0 (5)
#define PERIOD_GPS_Ap_0 (0.25)
#define PERIOD_NAVIGATION_Ap_0 (1.)
#define PERIOD_ATTITUDE_Ap_0 (0.1)
#define PERIOD_ESTIMATOR_Ap_0 (0.5)
#define PERIOD_ENERGY_Ap_0 (2.5)
#define PERIOD_WP_MOVED_Ap_0 (0.5)
#define PERIOD_CIRCLE_Ap_0 (1.05)
#define PERIOD_DESIRED_Ap_0 (0.2)
#define PERIOD_BAT_Ap_0 (1.1)
#define PERIOD_BARO_MS5534A_Ap_0 (1.0)
#define PERIOD_SCP_STATUS_Ap_0 (1.0)
#define PERIOD_SEGMENT_Ap_0 (1.2)
#define PERIOD_CALIBRATION_Ap_0 (2.1)
#define PERIOD_NAVIGATION_REF_Ap_0 (9.)
#define PERIOD_PPRZ_MODE_Ap_0 (5.)
#define PERIOD_SETTINGS_Ap_0 (5.)
#define PERIOD_STATE_FILTER_STATUS_Ap_0 (2.2)
#define PERIOD_DOWNLINK_Ap_0 (5.1)
#define PERIOD_DL_VALUE_Ap_0 (1.5)
#define PERIOD_IR_SENSORS_Ap_0 (1.2)
#define PERIOD_SURVEY_Ap_0 (2.1)
#define PERIOD_GPS_SOL_Ap_0 (2.0)
#define PERIOD_IMU_ACCEL_Ap_0 (.8)
#define PERIOD_IMU_GYRO_Ap_0 (.6)
#define PERIOD_IMU_MAG_Ap_0 (1.3)
#define TELEMETRY_MODE_Ap_minimal 1
#define PERIOD_ALIVE_Ap_1 (5)
#define PERIOD_ATTITUDE_Ap_1 (4)
#define PERIOD_GPS_Ap_1 (1.05)
#define PERIOD_ESTIMATOR_Ap_1 (1.3)
#define PERIOD_WP_MOVED_Ap_1 (1.4)
#define PERIOD_CIRCLE_Ap_1 (3.05)
#define PERIOD_DESIRED_Ap_1 (4.05)
#define PERIOD_BAT_Ap_1 (1.1)
#define PERIOD_SEGMENT_Ap_1 (3.2)
#define PERIOD_CALIBRATION_Ap_1 (5.1)
#define PERIOD_NAVIGATION_REF_Ap_1 (9.)
#define PERIOD_NAVIGATION_Ap_1 (3.)
#define PERIOD_PPRZ_MODE_Ap_1 (5.)
#define PERIOD_STATE_FILTER_STATUS_Ap_1 (5.)
#define PERIOD_DOWNLINK_Ap_1 (5.1)
#define PERIOD_DL_VALUE_Ap_1 (1.5)
#define PERIOD_IR_SENSORS_Ap_1 (5.2)
#define PERIOD_SURVEY_Ap_1 (2.1)
#define PERIOD_GPS_SOL_Ap_1 (5.0)
#define TELEMETRY_MODE_Ap_extremal 2
#define PERIOD_ALIVE_Ap_2 (5)
#define PERIOD_GPS_Ap_2 (5.1)
#define PERIOD_ESTIMATOR_Ap_2 (5.3)
#define PERIOD_BAT_Ap_2 (10.1)
#define PERIOD_DESIRED_Ap_2 (10.2)
#define PERIOD_NAVIGATION_Ap_2 (5.4)
#define PERIOD_PPRZ_MODE_Ap_2 (7.5)
#define PERIOD_STATE_FILTER_STATUS_Ap_2 (8.)
#define PERIOD_DOWNLINK_Ap_2 (5.7)
#define TELEMETRY_MODE_Ap_raw_sensors 3
#define PERIOD_DL_VALUE_Ap_3 (0.5)
#define PERIOD_ALIVE_Ap_3 (2.1)
#define PERIOD_IMU_ACCEL_RAW_Ap_3 (.05)
#define PERIOD_IMU_GYRO_RAW_Ap_3 (.05)
#define PERIOD_IMU_MAG_RAW_Ap_3 (.05)
#define PERIOD_BARO_RAW_Ap_3 (0.5)
#define TELEMETRY_MODE_Ap_scaled_sensors 4
#define PERIOD_DL_VALUE_Ap_4 (0.5)
#define PERIOD_ALIVE_Ap_4 (2.1)
#define PERIOD_IMU_GYRO_Ap_4 (.075)
#define PERIOD_IMU_ACCEL_Ap_4 (.075)
#define PERIOD_IMU_MAG_Ap_4 (.1)
#define TELEMETRY_MODE_Ap_debug_imu 5
#define PERIOD_ATTITUDE_Ap_5 (0.1)
#define PERIOD_ALIVE_Ap_5 (5)
#define PERIOD_GPS_Ap_5 (5.1)
#define PERIOD_ESTIMATOR_Ap_5 (5.3)
#define PERIOD_BAT_Ap_5 (10.1)
#define PERIOD_DESIRED_Ap_5 (10.2)
#define PERIOD_NAVIGATION_Ap_5 (5.4)
#define PERIOD_PPRZ_MODE_Ap_5 (5.5)
#define PERIOD_STATE_FILTER_STATUS_Ap_5 (5.)
#define PERIOD_DOWNLINK_Ap_5 (5.7)
#define PERIOD_IMU_ACCEL_Ap_5 (.5)
#define PERIOD_IMU_GYRO_Ap_5 (.5)
#define PERIOD_IMU_MAG_Ap_5 (.5)
#define PERIOD_IMU_ACCEL_RAW_Ap_5 (.5)
#define PERIOD_IMU_GYRO_RAW_Ap_5 (.5)
#define PERIOD_IMU_MAG_RAW_Ap_5 (.5)
#define PeriodicSendAp(_trans, _dev) {  /* 60Hz */ \
  if (telemetry_mode_Ap == TELEMETRY_MODE_Ap_default) {\
    static uint8_t i6 = 0; i6++; if (i6>=6) i6=0;\
    static uint8_t i12 = 0; i12++; if (i12>=12) i12=0;\
    static uint8_t i15 = 0; i15++; if (i15>=15) i15=0;\
    static uint8_t i30 = 0; i30++; if (i30>=30) i30=0;\
    static uint8_t i36 = 0; i36++; if (i36>=36) i36=0;\
    static uint8_t i48 = 0; i48++; if (i48>=48) i48=0;\
    static uint8_t i60 = 0; i60++; if (i60>=60) i60=0;\
    static uint8_t i63 = 0; i63++; if (i63>=63) i63=0;\
    static uint8_t i66 = 0; i66++; if (i66>=66) i66=0;\
    static uint8_t i72 = 0; i72++; if (i72>=72) i72=0;\
    static uint8_t i78 = 0; i78++; if (i78>=78) i78=0;\
    static uint8_t i90 = 0; i90++; if (i90>=90) i90=0;\
    static uint8_t i120 = 0; i120++; if (i120>=120) i120=0;\
    static uint8_t i126 = 0; i126++; if (i126>=126) i126=0;\
    static uint8_t i132 = 0; i132++; if (i132>=132) i132=0;\
    static uint8_t i150 = 0; i150++; if (i150>=150) i150=0;\
    static uint16_t i300 = 0; i300++; if (i300>=300) i300=0;\
    static uint16_t i306 = 0; i306++; if (i306>=306) i306=0;\
    static uint16_t i540 = 0; i540++; if (i540>=540) i540=0;\
    if (i6 == 0) {\
      PERIODIC_SEND_ATTITUDE(_trans, _dev);\
    } \
    if (i12 == 6) {\
      PERIODIC_SEND_DESIRED(_trans, _dev);\
    } \
    if (i15 == 12) {\
      PERIODIC_SEND_GPS(_trans, _dev);\
    } \
    if (i30 == 18) {\
      PERIODIC_SEND_ESTIMATOR(_trans, _dev);\
    } \
    else if (i30 == 24) {\
      PERIODIC_SEND_WP_MOVED(_trans, _dev);\
    } \
    if (i36 == 30) {\
      PERIODIC_SEND_IMU_GYRO(_trans, _dev);\
    } \
    if (i48 == 36) {\
      PERIODIC_SEND_IMU_ACCEL(_trans, _dev);\
    } \
    if (i60 == 42) {\
      PERIODIC_SEND_AIRSPEED(_trans, _dev);\
    } \
    else if (i60 == 48) {\
      PERIODIC_SEND_NAVIGATION(_trans, _dev);\
    } \
    else if (i60 == 54) {\
      PERIODIC_SEND_BARO_MS5534A(_trans, _dev);\
    } \
    else if (i60 == 0) {\
      PERIODIC_SEND_SCP_STATUS(_trans, _dev);\
    } \
    if (i63 == 6) {\
      PERIODIC_SEND_CIRCLE(_trans, _dev);\
    } \
    if (i66 == 12) {\
      PERIODIC_SEND_BAT(_trans, _dev);\
    } \
    if (i72 == 18) {\
      PERIODIC_SEND_SEGMENT(_trans, _dev);\
    } \
    else if (i72 == 24) {\
      PERIODIC_SEND_IR_SENSORS(_trans, _dev);\
    } \
    if (i78 == 30) {\
      PERIODIC_SEND_IMU_MAG(_trans, _dev);\
    } \
    if (i90 == 36) {\
      PERIODIC_SEND_DL_VALUE(_trans, _dev);\
    } \
    if (i120 == 42) {\
      PERIODIC_SEND_GPS_SOL(_trans, _dev);\
    } \
    if (i126 == 48) {\
      PERIODIC_SEND_CALIBRATION(_trans, _dev);\
    } \
    else if (i126 == 54) {\
      PERIODIC_SEND_SURVEY(_trans, _dev);\
    } \
    if (i132 == 60) {\
      PERIODIC_SEND_STATE_FILTER_STATUS(_trans, _dev);\
    } \
    if (i150 == 66) {\
      PERIODIC_SEND_ENERGY(_trans, _dev);\
    } \
    if (i300 == 72) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
    else if (i300 == 78) {\
      PERIODIC_SEND_PPRZ_MODE(_trans, _dev);\
    } \
    else if (i300 == 84) {\
      PERIODIC_SEND_SETTINGS(_trans, _dev);\
    } \
    if (i306 == 90) {\
      PERIODIC_SEND_DOWNLINK(_trans, _dev);\
    } \
    if (i540 == 96) {\
      PERIODIC_SEND_NAVIGATION_REF(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Ap == TELEMETRY_MODE_Ap_minimal) {\
    static uint8_t i63 = 0; i63++; if (i63>=63) i63=0;\
    static uint8_t i66 = 0; i66++; if (i66>=66) i66=0;\
    static uint8_t i78 = 0; i78++; if (i78>=78) i78=0;\
    static uint8_t i84 = 0; i84++; if (i84>=84) i84=0;\
    static uint8_t i90 = 0; i90++; if (i90>=90) i90=0;\
    static uint8_t i126 = 0; i126++; if (i126>=126) i126=0;\
    static uint8_t i180 = 0; i180++; if (i180>=180) i180=0;\
    static uint8_t i183 = 0; i183++; if (i183>=183) i183=0;\
    static uint8_t i192 = 0; i192++; if (i192>=192) i192=0;\
    static uint8_t i240 = 0; i240++; if (i240>=240) i240=0;\
    static uint8_t i243 = 0; i243++; if (i243>=243) i243=0;\
    static uint16_t i300 = 0; i300++; if (i300>=300) i300=0;\
    static uint16_t i306 = 0; i306++; if (i306>=306) i306=0;\
    static uint16_t i312 = 0; i312++; if (i312>=312) i312=0;\
    static uint16_t i540 = 0; i540++; if (i540>=540) i540=0;\
    if (i63 == 0) {\
      PERIODIC_SEND_GPS(_trans, _dev);\
    } \
    if (i66 == 6) {\
      PERIODIC_SEND_BAT(_trans, _dev);\
    } \
    if (i78 == 12) {\
      PERIODIC_SEND_ESTIMATOR(_trans, _dev);\
    } \
    if (i84 == 18) {\
      PERIODIC_SEND_WP_MOVED(_trans, _dev);\
    } \
    if (i90 == 24) {\
      PERIODIC_SEND_DL_VALUE(_trans, _dev);\
    } \
    if (i126 == 30) {\
      PERIODIC_SEND_SURVEY(_trans, _dev);\
    } \
    if (i180 == 36) {\
      PERIODIC_SEND_NAVIGATION(_trans, _dev);\
    } \
    if (i183 == 42) {\
      PERIODIC_SEND_CIRCLE(_trans, _dev);\
    } \
    if (i192 == 48) {\
      PERIODIC_SEND_SEGMENT(_trans, _dev);\
    } \
    if (i240 == 54) {\
      PERIODIC_SEND_ATTITUDE(_trans, _dev);\
    } \
    if (i243 == 60) {\
      PERIODIC_SEND_DESIRED(_trans, _dev);\
    } \
    if (i300 == 66) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
    else if (i300 == 72) {\
      PERIODIC_SEND_PPRZ_MODE(_trans, _dev);\
    } \
    else if (i300 == 78) {\
      PERIODIC_SEND_STATE_FILTER_STATUS(_trans, _dev);\
    } \
    else if (i300 == 84) {\
      PERIODIC_SEND_GPS_SOL(_trans, _dev);\
    } \
    if (i306 == 90) {\
      PERIODIC_SEND_CALIBRATION(_trans, _dev);\
    } \
    else if (i306 == 96) {\
      PERIODIC_SEND_DOWNLINK(_trans, _dev);\
    } \
    if (i312 == 102) {\
      PERIODIC_SEND_IR_SENSORS(_trans, _dev);\
    } \
    if (i540 == 108) {\
      PERIODIC_SEND_NAVIGATION_REF(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Ap == TELEMETRY_MODE_Ap_extremal) {\
    static uint16_t i300 = 0; i300++; if (i300>=300) i300=0;\
    static uint16_t i306 = 0; i306++; if (i306>=306) i306=0;\
    static uint16_t i318 = 0; i318++; if (i318>=318) i318=0;\
    static uint16_t i324 = 0; i324++; if (i324>=324) i324=0;\
    static uint16_t i342 = 0; i342++; if (i342>=342) i342=0;\
    static uint16_t i450 = 0; i450++; if (i450>=450) i450=0;\
    static uint16_t i480 = 0; i480++; if (i480>=480) i480=0;\
    static uint16_t i606 = 0; i606++; if (i606>=606) i606=0;\
    static uint16_t i612 = 0; i612++; if (i612>=612) i612=0;\
    if (i300 == 0) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
    if (i306 == 6) {\
      PERIODIC_SEND_GPS(_trans, _dev);\
    } \
    if (i318 == 12) {\
      PERIODIC_SEND_ESTIMATOR(_trans, _dev);\
    } \
    if (i324 == 18) {\
      PERIODIC_SEND_NAVIGATION(_trans, _dev);\
    } \
    if (i342 == 24) {\
      PERIODIC_SEND_DOWNLINK(_trans, _dev);\
    } \
    if (i450 == 30) {\
      PERIODIC_SEND_PPRZ_MODE(_trans, _dev);\
    } \
    if (i480 == 36) {\
      PERIODIC_SEND_STATE_FILTER_STATUS(_trans, _dev);\
    } \
    if (i606 == 42) {\
      PERIODIC_SEND_BAT(_trans, _dev);\
    } \
    if (i612 == 48) {\
      PERIODIC_SEND_DESIRED(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Ap == TELEMETRY_MODE_Ap_raw_sensors) {\
    static uint8_t i3 = 0; i3++; if (i3>=3) i3=0;\
    static uint8_t i30 = 0; i30++; if (i30>=30) i30=0;\
    static uint8_t i126 = 0; i126++; if (i126>=126) i126=0;\
    if (i3 == 0) {\
      PERIODIC_SEND_IMU_ACCEL_RAW(_trans, _dev);\
    } \
    if (i3 == 0) {\
      PERIODIC_SEND_IMU_GYRO_RAW(_trans, _dev);\
    } \
    if (i3 == 0) {\
      PERIODIC_SEND_IMU_MAG_RAW(_trans, _dev);\
    } \
    if (i30 == 6) {\
      PERIODIC_SEND_DL_VALUE(_trans, _dev);\
    } \
    else if (i30 == 12) {\
      PERIODIC_SEND_BARO_RAW(_trans, _dev);\
    } \
    if (i126 == 18) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Ap == TELEMETRY_MODE_Ap_scaled_sensors) {\
    static uint8_t i4 = 0; i4++; if (i4>=4) i4=0;\
    static uint8_t i6 = 0; i6++; if (i6>=6) i6=0;\
    static uint8_t i30 = 0; i30++; if (i30>=30) i30=0;\
    static uint8_t i126 = 0; i126++; if (i126>=126) i126=0;\
    if (i4 == 0) {\
      PERIODIC_SEND_IMU_GYRO(_trans, _dev);\
    } \
    else if (i4 == 2) {\
      PERIODIC_SEND_IMU_ACCEL(_trans, _dev);\
    } \
    if (i6 == 2) {\
      PERIODIC_SEND_IMU_MAG(_trans, _dev);\
    } \
    if (i30 == 8) {\
      PERIODIC_SEND_DL_VALUE(_trans, _dev);\
    } \
    if (i126 == 14) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Ap == TELEMETRY_MODE_Ap_debug_imu) {\
    static uint8_t i6 = 0; i6++; if (i6>=6) i6=0;\
    static uint8_t i30 = 0; i30++; if (i30>=30) i30=0;\
    static uint16_t i300 = 0; i300++; if (i300>=300) i300=0;\
    static uint16_t i306 = 0; i306++; if (i306>=306) i306=0;\
    static uint16_t i318 = 0; i318++; if (i318>=318) i318=0;\
    static uint16_t i324 = 0; i324++; if (i324>=324) i324=0;\
    static uint16_t i330 = 0; i330++; if (i330>=330) i330=0;\
    static uint16_t i342 = 0; i342++; if (i342>=342) i342=0;\
    static uint16_t i606 = 0; i606++; if (i606>=606) i606=0;\
    static uint16_t i612 = 0; i612++; if (i612>=612) i612=0;\
    if (i6 == 0) {\
      PERIODIC_SEND_ATTITUDE(_trans, _dev);\
    } \
    if (i30 == 6) {\
      PERIODIC_SEND_IMU_ACCEL(_trans, _dev);\
    } \
    else if (i30 == 12) {\
      PERIODIC_SEND_IMU_GYRO(_trans, _dev);\
    } \
    else if (i30 == 18) {\
      PERIODIC_SEND_IMU_MAG(_trans, _dev);\
    } \
    else if (i30 == 24) {\
      PERIODIC_SEND_IMU_ACCEL_RAW(_trans, _dev);\
    } \
    else if (i30 == 0) {\
      PERIODIC_SEND_IMU_GYRO_RAW(_trans, _dev);\
    } \
    if (i30 == 6) {\
      PERIODIC_SEND_IMU_MAG_RAW(_trans, _dev);\
    } \
    if (i300 == 12) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
    else if (i300 == 18) {\
      PERIODIC_SEND_STATE_FILTER_STATUS(_trans, _dev);\
    } \
    if (i306 == 24) {\
      PERIODIC_SEND_GPS(_trans, _dev);\
    } \
    if (i318 == 30) {\
      PERIODIC_SEND_ESTIMATOR(_trans, _dev);\
    } \
    if (i324 == 36) {\
      PERIODIC_SEND_NAVIGATION(_trans, _dev);\
    } \
    if (i330 == 42) {\
      PERIODIC_SEND_PPRZ_MODE(_trans, _dev);\
    } \
    if (i342 == 48) {\
      PERIODIC_SEND_DOWNLINK(_trans, _dev);\
    } \
    if (i606 == 54) {\
      PERIODIC_SEND_BAT(_trans, _dev);\
    } \
    if (i612 == 60) {\
      PERIODIC_SEND_DESIRED(_trans, _dev);\
    } \
  }\
}

/* Macros for Fbw process */
#ifdef PERIODIC_C_FBW
#ifndef TELEMETRY_MODE_FBW
#define TELEMETRY_MODE_FBW 0
#endif
uint8_t telemetry_mode_Fbw = TELEMETRY_MODE_FBW;
#else /* PERIODIC_C_FBW not defined (general header) */
extern uint8_t telemetry_mode_Fbw;
#endif /* PERIODIC_C_FBW */
#define TELEMETRY_MODE_Fbw_default 0
#define PERIOD_COMMANDS_Fbw_0 (5)
#define PERIOD_FBW_STATUS_Fbw_0 (2)
#define PERIOD_ACTUATORS_Fbw_0 (5)
#define TELEMETRY_MODE_Fbw_debug 1
#define PERIOD_PPM_Fbw_1 (0.5)
#define PERIOD_RC_Fbw_1 (0.5)
#define PERIOD_COMMANDS_Fbw_1 (0.5)
#define PERIOD_FBW_STATUS_Fbw_1 (1)
#define PERIOD_ACTUATORS_Fbw_1 (5)
#define PeriodicSendFbw(_trans, _dev) {  /* 60Hz */ \
  if (telemetry_mode_Fbw == TELEMETRY_MODE_Fbw_default) {\
    static uint8_t i120 = 0; i120++; if (i120>=120) i120=0;\
    static uint16_t i300 = 0; i300++; if (i300>=300) i300=0;\
    if (i120 == 0) {\
      PERIODIC_SEND_FBW_STATUS(_trans, _dev);\
    } \
    if (i300 == 6) {\
      PERIODIC_SEND_COMMANDS(_trans, _dev);\
    } \
    else if (i300 == 12) {\
      PERIODIC_SEND_ACTUATORS(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Fbw == TELEMETRY_MODE_Fbw_debug) {\
    static uint8_t i30 = 0; i30++; if (i30>=30) i30=0;\
    static uint8_t i60 = 0; i60++; if (i60>=60) i60=0;\
    static uint16_t i300 = 0; i300++; if (i300>=300) i300=0;\
    if (i30 == 0) {\
      PERIODIC_SEND_PPM(_trans, _dev);\
    } \
    else if (i30 == 6) {\
      PERIODIC_SEND_RC(_trans, _dev);\
    } \
    else if (i30 == 12) {\
      PERIODIC_SEND_COMMANDS(_trans, _dev);\
    } \
    if (i60 == 18) {\
      PERIODIC_SEND_FBW_STATUS(_trans, _dev);\
    } \
    if (i300 == 24) {\
      PERIODIC_SEND_ACTUATORS(_trans, _dev);\
    } \
  }\
}
#endif // _VAR_PERIODIC_H_
