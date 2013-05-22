/* This file has been generated from /home/chorus/chorus/conf/messages.xml and /home/chorus/chorus/conf/telemetry/default_rotorcraft.xml */
/* Please DO NOT EDIT */

#ifndef _VAR_PERIODIC_H_
#define _VAR_PERIODIC_H_

#include "std.h"
#include "generated/airframe.h"


/* Macros for Main process */
#ifdef PERIODIC_C_MAIN
#ifndef TELEMETRY_MODE_MAIN
#define TELEMETRY_MODE_MAIN 0
#endif
uint8_t telemetry_mode_Main = TELEMETRY_MODE_MAIN;
#else /* PERIODIC_C_MAIN not defined (general header) */
extern uint8_t telemetry_mode_Main;
#endif /* PERIODIC_C_MAIN */
#define TELEMETRY_MODE_Main_default 0
#define PERIOD_DL_VALUE_Main_0 (1.1)
#define PERIOD_ROTORCRAFT_STATUS_Main_0 (1.2)
#define PERIOD_ROTORCRAFT_FP_Main_0 (0.25)
#define PERIOD_ALIVE_Main_0 (2.1)
#define PERIOD_INS_REF_Main_0 (5.1)
#define PERIOD_ROTORCRAFT_NAV_STATUS_Main_0 (1.6)
#define PERIOD_WP_MOVED_Main_0 (1.3)
#define PERIOD_ROTORCRAFT_CAM_Main_0 (1.)
#define PERIOD_GPS_INT_Main_0 (.25)
#define PERIOD_INS_Main_0 (.25)
#define PERIOD_I2C_ERRORS_Main_0 (4.1)
#define PERIOD_UART_ERRORS_Main_0 (3.1)
#define TELEMETRY_MODE_Main_ppm 1
#define PERIOD_ROTORCRAFT_CMD_Main_1 (.05)
#define PERIOD_PPM_Main_1 (0.5)
#define PERIOD_RC_Main_1 (0.5)
#define PERIOD_ROTORCRAFT_RADIO_CONTROL_Main_1 (0.5)
#define PERIOD_ROTORCRAFT_STATUS_Main_1 (1)
#define TELEMETRY_MODE_Main_raw_sensors 2
#define PERIOD_ROTORCRAFT_STATUS_Main_2 (1.2)
#define PERIOD_DL_VALUE_Main_2 (0.5)
#define PERIOD_ALIVE_Main_2 (2.1)
#define PERIOD_IMU_ACCEL_RAW_Main_2 (.05)
#define PERIOD_IMU_GYRO_RAW_Main_2 (.05)
#define PERIOD_IMU_MAG_RAW_Main_2 (.05)
#define PERIOD_BARO_RAW_Main_2 (.1)
#define TELEMETRY_MODE_Main_scaled_sensors 3
#define PERIOD_ROTORCRAFT_STATUS_Main_3 (1.2)
#define PERIOD_DL_VALUE_Main_3 (0.5)
#define PERIOD_ALIVE_Main_3 (2.1)
#define PERIOD_IMU_GYRO_SCALED_Main_3 (.075)
#define PERIOD_IMU_ACCEL_SCALED_Main_3 (.075)
#define PERIOD_IMU_MAG_SCALED_Main_3 (.1)
#define TELEMETRY_MODE_Main_ahrs 4
#define PERIOD_ROTORCRAFT_STATUS_Main_4 (1.2)
#define PERIOD_DL_VALUE_Main_4 (0.5)
#define PERIOD_ALIVE_Main_4 (2.1)
#define PERIOD_FILTER_ALIGNER_Main_4 (2.2)
#define PERIOD_FILTER_Main_4 (.5)
#define PERIOD_AHRS_GYRO_BIAS_INT_Main_4 (0.08)
#define PERIOD_AHRS_EULER_INT_Main_4 (.1)
#define TELEMETRY_MODE_Main_rate_loop 5
#define PERIOD_ROTORCRAFT_STATUS_Main_5 (1.2)
#define PERIOD_DL_VALUE_Main_5 (0.5)
#define PERIOD_ALIVE_Main_5 (2.1)
#define PERIOD_RATE_LOOP_Main_5 (.02)
#define TELEMETRY_MODE_Main_attitude_setpoint_viz 6
#define PERIOD_ROTORCRAFT_STATUS_Main_6 (1.2)
#define PERIOD_DL_VALUE_Main_6 (0.5)
#define PERIOD_ALIVE_Main_6 (0.9)
#define PERIOD_ROTORCRAFT_RADIO_CONTROL_Main_6 (0.1)
#define PERIOD_AHRS_REF_QUAT_Main_6 (0.05)
#define TELEMETRY_MODE_Main_attitude_loop 7
#define PERIOD_ROTORCRAFT_STATUS_Main_7 (1.2)
#define PERIOD_DL_VALUE_Main_7 (0.5)
#define PERIOD_ALIVE_Main_7 (0.9)
#define PERIOD_STAB_ATTITUDE_Main_7 (.03)
#define PERIOD_STAB_ATTITUDE_REF_Main_7 (.03)
#define TELEMETRY_MODE_Main_vert_loop 8
#define PERIOD_ROTORCRAFT_STATUS_Main_8 (1.2)
#define PERIOD_DL_VALUE_Main_8 (0.5)
#define PERIOD_ALIVE_Main_8 (0.9)
#define PERIOD_VFF_Main_8 (.05)
#define PERIOD_VERT_LOOP_Main_8 (.05)
#define PERIOD_INS_Main_8 (.05)
#define PERIOD_INS_REF_Main_8 (5.1)
#define TELEMETRY_MODE_Main_h_loop 9
#define PERIOD_ALIVE_Main_9 (0.9)
#define PERIOD_HOVER_LOOP_Main_9 (0.062)
#define PERIOD_GUIDANCE_H_REF_Main_9 (0.062)
#define PERIOD_STAB_ATTITUDE_Main_9 (0.4)
#define PERIOD_ROTORCRAFT_FP_Main_9 (0.8)
#define PERIOD_ROTORCRAFT_STATUS_Main_9 (1.2)
#define PERIOD_ROTORCRAFT_NAV_STATUS_Main_9 (1.6)
#define PERIOD_INS_REF_Main_9 (5.1)
#define PERIOD_HFF_Main_9 (.05)
#define PERIOD_HFF_GPS_Main_9 (.03)
#define PERIOD_HFF_DBG_Main_9 (.2)
#define TELEMETRY_MODE_Main_aligner 10
#define PERIOD_ALIVE_Main_10 (0.9)
#define PERIOD_FILTER_ALIGNER_Main_10 (0.02)
#define TELEMETRY_MODE_Main_hs_att_roll 11
#define PERIOD_ROTORCRAFT_STATUS_Main_11 (1.2)
#define PERIOD_ALIVE_Main_11 (0.9)
#define PERIOD_DL_VALUE_Main_11 (0.5)
#define TELEMETRY_MODE_Main_tune_hover 12
#define PERIOD_DL_VALUE_Main_12 (1.1)
#define PERIOD_ROTORCRAFT_STATUS_Main_12 (1.2)
#define PERIOD_ALIVE_Main_12 (2.1)
#define PERIOD_GUIDANCE_H_INT_Main_12 (0.05)
#define PERIOD_INS_REF_Main_12 (5.1)
#define TELEMETRY_MODE_Main_mag_current_calibration 13
#define PERIOD_ROTORCRAFT_STATUS_Main_13 (1.2)
#define PERIOD_DL_VALUE_Main_13 (0.5)
#define PERIOD_ALIVE_Main_13 (2.1)
#define PERIOD_IMU_MAG_CURRENT_CALIBRATION_Main_13 (0.05)
#define PeriodicSendMain(_trans, _dev) {  /* 60Hz */ \
  if (telemetry_mode_Main == TELEMETRY_MODE_Main_default) {\
    static uint8_t i15 = 0; i15++; if (i15>=15) i15=0;\
    static uint8_t i60 = 0; i60++; if (i60>=60) i60=0;\
    static uint8_t i66 = 0; i66++; if (i66>=66) i66=0;\
    static uint8_t i72 = 0; i72++; if (i72>=72) i72=0;\
    static uint8_t i78 = 0; i78++; if (i78>=78) i78=0;\
    static uint8_t i96 = 0; i96++; if (i96>=96) i96=0;\
    static uint8_t i126 = 0; i126++; if (i126>=126) i126=0;\
    static uint8_t i186 = 0; i186++; if (i186>=186) i186=0;\
    static uint8_t i245 = 0; i245++; if (i245>=245) i245=0;\
    static uint16_t i306 = 0; i306++; if (i306>=306) i306=0;\
    if (i15 == 0) {\
      PERIODIC_SEND_ROTORCRAFT_FP(_trans, _dev);\
    } \
    else if (i15 == 6) {\
      PERIODIC_SEND_GPS_INT(_trans, _dev);\
    } \
    else if (i15 == 12) {\
      PERIODIC_SEND_INS(_trans, _dev);\
    } \
    if (i60 == 18) {\
      PERIODIC_SEND_ROTORCRAFT_CAM(_trans, _dev);\
    } \
    if (i66 == 24) {\
      PERIODIC_SEND_DL_VALUE(_trans, _dev);\
    } \
    if (i72 == 30) {\
      PERIODIC_SEND_ROTORCRAFT_STATUS(_trans, _dev);\
    } \
    if (i78 == 36) {\
      PERIODIC_SEND_WP_MOVED(_trans, _dev);\
    } \
    if (i96 == 42) {\
      PERIODIC_SEND_ROTORCRAFT_NAV_STATUS(_trans, _dev);\
    } \
    if (i126 == 48) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
    if (i186 == 54) {\
      PERIODIC_SEND_UART_ERRORS(_trans, _dev);\
    } \
    if (i245 == 60) {\
      PERIODIC_SEND_I2C_ERRORS(_trans, _dev);\
    } \
    if (i306 == 66) {\
      PERIODIC_SEND_INS_REF(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Main == TELEMETRY_MODE_Main_ppm) {\
    static uint8_t i3 = 0; i3++; if (i3>=3) i3=0;\
    static uint8_t i30 = 0; i30++; if (i30>=30) i30=0;\
    static uint8_t i60 = 0; i60++; if (i60>=60) i60=0;\
    if (i3 == 0) {\
      PERIODIC_SEND_ROTORCRAFT_CMD(_trans, _dev);\
    } \
    if (i30 == 6) {\
      PERIODIC_SEND_PPM(_trans, _dev);\
    } \
    else if (i30 == 12) {\
      PERIODIC_SEND_RC(_trans, _dev);\
    } \
    else if (i30 == 18) {\
      PERIODIC_SEND_ROTORCRAFT_RADIO_CONTROL(_trans, _dev);\
    } \
    if (i60 == 24) {\
      PERIODIC_SEND_ROTORCRAFT_STATUS(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Main == TELEMETRY_MODE_Main_raw_sensors) {\
    static uint8_t i3 = 0; i3++; if (i3>=3) i3=0;\
    static uint8_t i6 = 0; i6++; if (i6>=6) i6=0;\
    static uint8_t i30 = 0; i30++; if (i30>=30) i30=0;\
    static uint8_t i72 = 0; i72++; if (i72>=72) i72=0;\
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
    if (i6 == 0) {\
      PERIODIC_SEND_BARO_RAW(_trans, _dev);\
    } \
    if (i30 == 6) {\
      PERIODIC_SEND_DL_VALUE(_trans, _dev);\
    } \
    if (i72 == 12) {\
      PERIODIC_SEND_ROTORCRAFT_STATUS(_trans, _dev);\
    } \
    if (i126 == 18) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Main == TELEMETRY_MODE_Main_scaled_sensors) {\
    static uint8_t i4 = 0; i4++; if (i4>=4) i4=0;\
    static uint8_t i6 = 0; i6++; if (i6>=6) i6=0;\
    static uint8_t i30 = 0; i30++; if (i30>=30) i30=0;\
    static uint8_t i72 = 0; i72++; if (i72>=72) i72=0;\
    static uint8_t i126 = 0; i126++; if (i126>=126) i126=0;\
    if (i4 == 0) {\
      PERIODIC_SEND_IMU_GYRO_SCALED(_trans, _dev);\
    } \
    else if (i4 == 2) {\
      PERIODIC_SEND_IMU_ACCEL_SCALED(_trans, _dev);\
    } \
    if (i6 == 2) {\
      PERIODIC_SEND_IMU_MAG_SCALED(_trans, _dev);\
    } \
    if (i30 == 8) {\
      PERIODIC_SEND_DL_VALUE(_trans, _dev);\
    } \
    if (i72 == 14) {\
      PERIODIC_SEND_ROTORCRAFT_STATUS(_trans, _dev);\
    } \
    if (i126 == 20) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Main == TELEMETRY_MODE_Main_ahrs) {\
    static uint8_t i4 = 0; i4++; if (i4>=4) i4=0;\
    static uint8_t i6 = 0; i6++; if (i6>=6) i6=0;\
    static uint8_t i30 = 0; i30++; if (i30>=30) i30=0;\
    static uint8_t i72 = 0; i72++; if (i72>=72) i72=0;\
    static uint8_t i126 = 0; i126++; if (i126>=126) i126=0;\
    static uint8_t i132 = 0; i132++; if (i132>=132) i132=0;\
    if (i4 == 0) {\
      PERIODIC_SEND_AHRS_GYRO_BIAS_INT(_trans, _dev);\
    } \
    if (i6 == 0) {\
      PERIODIC_SEND_AHRS_EULER_INT(_trans, _dev);\
    } \
    if (i30 == 6) {\
      PERIODIC_SEND_DL_VALUE(_trans, _dev);\
    } \
    else if (i30 == 12) {\
      PERIODIC_SEND_FILTER(_trans, _dev);\
    } \
    if (i72 == 18) {\
      PERIODIC_SEND_ROTORCRAFT_STATUS(_trans, _dev);\
    } \
    if (i126 == 24) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
    if (i132 == 30) {\
      PERIODIC_SEND_FILTER_ALIGNER(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Main == TELEMETRY_MODE_Main_rate_loop) {\
    static uint8_t i1 = 0; i1++; if (i1>=1) i1=0;\
    static uint8_t i30 = 0; i30++; if (i30>=30) i30=0;\
    static uint8_t i72 = 0; i72++; if (i72>=72) i72=0;\
    static uint8_t i126 = 0; i126++; if (i126>=126) i126=0;\
    if (i1 == 0) {\
      PERIODIC_SEND_RATE_LOOP(_trans, _dev);\
    } \
    if (i30 == 6) {\
      PERIODIC_SEND_DL_VALUE(_trans, _dev);\
    } \
    if (i72 == 12) {\
      PERIODIC_SEND_ROTORCRAFT_STATUS(_trans, _dev);\
    } \
    if (i126 == 18) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Main == TELEMETRY_MODE_Main_attitude_setpoint_viz) {\
    static uint8_t i3 = 0; i3++; if (i3>=3) i3=0;\
    static uint8_t i6 = 0; i6++; if (i6>=6) i6=0;\
    static uint8_t i30 = 0; i30++; if (i30>=30) i30=0;\
    static uint8_t i54 = 0; i54++; if (i54>=54) i54=0;\
    static uint8_t i72 = 0; i72++; if (i72>=72) i72=0;\
    if (i3 == 0) {\
      PERIODIC_SEND_AHRS_REF_QUAT(_trans, _dev);\
    } \
    if (i6 == 0) {\
      PERIODIC_SEND_ROTORCRAFT_RADIO_CONTROL(_trans, _dev);\
    } \
    if (i30 == 6) {\
      PERIODIC_SEND_DL_VALUE(_trans, _dev);\
    } \
    if (i54 == 12) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
    if (i72 == 18) {\
      PERIODIC_SEND_ROTORCRAFT_STATUS(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Main == TELEMETRY_MODE_Main_attitude_loop) {\
    static uint8_t i1 = 0; i1++; if (i1>=1) i1=0;\
    static uint8_t i30 = 0; i30++; if (i30>=30) i30=0;\
    static uint8_t i54 = 0; i54++; if (i54>=54) i54=0;\
    static uint8_t i72 = 0; i72++; if (i72>=72) i72=0;\
    if (i1 == 0) {\
      PERIODIC_SEND_STAB_ATTITUDE(_trans, _dev);\
    } \
    if (i1 == 0) {\
      PERIODIC_SEND_STAB_ATTITUDE_REF(_trans, _dev);\
    } \
    if (i30 == 6) {\
      PERIODIC_SEND_DL_VALUE(_trans, _dev);\
    } \
    if (i54 == 12) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
    if (i72 == 18) {\
      PERIODIC_SEND_ROTORCRAFT_STATUS(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Main == TELEMETRY_MODE_Main_vert_loop) {\
    static uint8_t i3 = 0; i3++; if (i3>=3) i3=0;\
    static uint8_t i30 = 0; i30++; if (i30>=30) i30=0;\
    static uint8_t i54 = 0; i54++; if (i54>=54) i54=0;\
    static uint8_t i72 = 0; i72++; if (i72>=72) i72=0;\
    static uint16_t i306 = 0; i306++; if (i306>=306) i306=0;\
    if (i3 == 0) {\
      PERIODIC_SEND_VFF(_trans, _dev);\
    } \
    if (i3 == 0) {\
      PERIODIC_SEND_VERT_LOOP(_trans, _dev);\
    } \
    if (i3 == 0) {\
      PERIODIC_SEND_INS(_trans, _dev);\
    } \
    if (i30 == 6) {\
      PERIODIC_SEND_DL_VALUE(_trans, _dev);\
    } \
    if (i54 == 12) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
    if (i72 == 18) {\
      PERIODIC_SEND_ROTORCRAFT_STATUS(_trans, _dev);\
    } \
    if (i306 == 24) {\
      PERIODIC_SEND_INS_REF(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Main == TELEMETRY_MODE_Main_h_loop) {\
    static uint8_t i1 = 0; i1++; if (i1>=1) i1=0;\
    static uint8_t i3 = 0; i3++; if (i3>=3) i3=0;\
    static uint8_t i12 = 0; i12++; if (i12>=12) i12=0;\
    static uint8_t i24 = 0; i24++; if (i24>=24) i24=0;\
    static uint8_t i48 = 0; i48++; if (i48>=48) i48=0;\
    static uint8_t i54 = 0; i54++; if (i54>=54) i54=0;\
    static uint8_t i72 = 0; i72++; if (i72>=72) i72=0;\
    static uint8_t i96 = 0; i96++; if (i96>=96) i96=0;\
    static uint16_t i306 = 0; i306++; if (i306>=306) i306=0;\
    if (i1 == 0) {\
      PERIODIC_SEND_HFF_GPS(_trans, _dev);\
    } \
    if (i3 == 0) {\
      PERIODIC_SEND_HOVER_LOOP(_trans, _dev);\
    } \
    if (i3 == 0) {\
      PERIODIC_SEND_GUIDANCE_H_REF(_trans, _dev);\
    } \
    if (i3 == 0) {\
      PERIODIC_SEND_HFF(_trans, _dev);\
    } \
    if (i12 == 6) {\
      PERIODIC_SEND_HFF_DBG(_trans, _dev);\
    } \
    if (i24 == 12) {\
      PERIODIC_SEND_STAB_ATTITUDE(_trans, _dev);\
    } \
    if (i48 == 18) {\
      PERIODIC_SEND_ROTORCRAFT_FP(_trans, _dev);\
    } \
    if (i54 == 24) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
    if (i72 == 30) {\
      PERIODIC_SEND_ROTORCRAFT_STATUS(_trans, _dev);\
    } \
    if (i96 == 36) {\
      PERIODIC_SEND_ROTORCRAFT_NAV_STATUS(_trans, _dev);\
    } \
    if (i306 == 42) {\
      PERIODIC_SEND_INS_REF(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Main == TELEMETRY_MODE_Main_aligner) {\
    static uint8_t i1 = 0; i1++; if (i1>=1) i1=0;\
    static uint8_t i54 = 0; i54++; if (i54>=54) i54=0;\
    if (i1 == 0) {\
      PERIODIC_SEND_FILTER_ALIGNER(_trans, _dev);\
    } \
    if (i54 == 6) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Main == TELEMETRY_MODE_Main_hs_att_roll) {\
    static uint8_t i30 = 0; i30++; if (i30>=30) i30=0;\
    static uint8_t i54 = 0; i54++; if (i54>=54) i54=0;\
    static uint8_t i72 = 0; i72++; if (i72>=72) i72=0;\
    if (i30 == 0) {\
      PERIODIC_SEND_DL_VALUE(_trans, _dev);\
    } \
    if (i54 == 6) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
    if (i72 == 12) {\
      PERIODIC_SEND_ROTORCRAFT_STATUS(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Main == TELEMETRY_MODE_Main_tune_hover) {\
    static uint8_t i3 = 0; i3++; if (i3>=3) i3=0;\
    static uint8_t i66 = 0; i66++; if (i66>=66) i66=0;\
    static uint8_t i72 = 0; i72++; if (i72>=72) i72=0;\
    static uint8_t i126 = 0; i126++; if (i126>=126) i126=0;\
    static uint16_t i306 = 0; i306++; if (i306>=306) i306=0;\
    if (i3 == 0) {\
      PERIODIC_SEND_GUIDANCE_H_INT(_trans, _dev);\
    } \
    if (i66 == 6) {\
      PERIODIC_SEND_DL_VALUE(_trans, _dev);\
    } \
    if (i72 == 12) {\
      PERIODIC_SEND_ROTORCRAFT_STATUS(_trans, _dev);\
    } \
    if (i126 == 18) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
    if (i306 == 24) {\
      PERIODIC_SEND_INS_REF(_trans, _dev);\
    } \
  }\
  if (telemetry_mode_Main == TELEMETRY_MODE_Main_mag_current_calibration) {\
    static uint8_t i3 = 0; i3++; if (i3>=3) i3=0;\
    static uint8_t i30 = 0; i30++; if (i30>=30) i30=0;\
    static uint8_t i72 = 0; i72++; if (i72>=72) i72=0;\
    static uint8_t i126 = 0; i126++; if (i126>=126) i126=0;\
    if (i3 == 0) {\
      PERIODIC_SEND_IMU_MAG_CURRENT_CALIBRATION(_trans, _dev);\
    } \
    if (i30 == 6) {\
      PERIODIC_SEND_DL_VALUE(_trans, _dev);\
    } \
    if (i72 == 12) {\
      PERIODIC_SEND_ROTORCRAFT_STATUS(_trans, _dev);\
    } \
    if (i126 == 18) {\
      PERIODIC_SEND_ALIVE(_trans, _dev);\
    } \
  }\
}
#endif // _VAR_PERIODIC_H_
