/* This file has been generated from /home/chorus/chorus/var/Bixler/settings_modules.xml /home/chorus/chorus/conf/settings/estimation/ins_neutrals.xml /home/chorus/chorus/conf/settings/control/ctl_basic.xml /home/chorus/chorus/conf/settings/fixedwing_basic.xml /home/chorus/chorus/var/Bixler/settings_telemetry.xml */
/* Please DO NOT EDIT */

#ifndef SETTINGS_H
#define SETTINGS_H

#define RCSettings(mode_changed) { \
}

#include "autopilot.h"
#include "guidance/guidance_v.h"
#include "inter_mcu.h"
#include "stabilization/stabilization_attitude.h"
#include "subsystems/ahrs.h"
#include "subsystems/gps.h"
#include "subsystems/nav.h"
#include "generated/modules.h"
#include "generated/periodic_telemetry.h"

#define DlSetting(_idx, _value) { \
  switch (_idx) { \
    case 0: telemetry_mode_Ap = _value; break;\
    case 1: telemetry_mode_Fbw = _value; break;\
    case 2: flight_altitude = _value; break;\
    case 3: nav_course = _value; break;\
    case 4: nav_IncreaseShift( _value ); _value = nav_shift; break;\
    case 5: autopilot_ResetFlightTimeAndLaunch( _value ); _value = autopilot_flight_time; break;\
    case 6: nav_SetNavRadius( _value ); _value = nav_radius; break;\
    case 7: pprz_mode = _value; break;\
    case 8: launch = _value; break;\
    case 9: kill_throttle = _value; break;\
    case 10: gps_Reset( _value ); _value = gps.reset; break;\
    case 11: ap_state->command_roll_trim = _value; break;\
    case 12: ap_state->command_pitch_trim = _value; break;\
    case 13: h_ctl_roll_pgain = _value; break;\
    case 14: h_ctl_roll_max_setpoint = _value; break;\
    case 15: h_ctl_pitch_pgain = _value; break;\
    case 16: h_ctl_pitch_dgain = _value; break;\
    case 17: h_ctl_elevator_of_roll = _value; break;\
    case 18: h_ctl_aileron_of_throttle = _value; break;\
    case 19: h_ctl_roll_attitude_gain = _value; break;\
    case 20: h_ctl_roll_rate_gain = _value; break;\
    case 21: v_ctl_altitude_pgain = _value; break;\
    case 22: guidance_v_SetCruiseThrottle( _value ); _value = v_ctl_auto_throttle_cruise_throttle; break;\
    case 23: v_ctl_auto_throttle_pgain = _value; break;\
    case 24: v_ctl_auto_throttle_igain = _value; break;\
    case 25: v_ctl_auto_throttle_dgain = _value; break;\
    case 26: v_ctl_auto_throttle_climb_throttle_increment = _value; break;\
    case 27: v_ctl_auto_throttle_pitch_of_vz_pgain = _value; break;\
    case 28: v_ctl_auto_throttle_pitch_of_vz_dgain = _value; break;\
    case 29: v_ctl_auto_pitch_pgain = _value; break;\
    case 30: v_ctl_auto_pitch_igain = _value; break;\
    case 31: h_ctl_course_pgain = _value; break;\
    case 32: h_ctl_course_dgain = _value; break;\
    case 33: h_ctl_course_pre_bank_correction = _value; break;\
    case 34: nav_glide_pitch_trim = _value; break;\
    case 35: h_ctl_roll_slew = _value; break;\
    case 36: nav_radius = _value; break;\
    case 37: nav_course = _value; break;\
    case 38: nav_mode = _value; break;\
    case 39: nav_climb = _value; break;\
    case 40: fp_pitch = _value; break;\
    case 41: nav_IncreaseShift( _value ); _value = nav_shift; break;\
    case 42: nav_ground_speed_setpoint = _value; break;\
    case 43: nav_ground_speed_pgain = _value; break;\
    case 44: nav_survey_shift = _value; break;\
    case 45: ins_roll_neutral = _value; break;\
    case 46: ins_pitch_neutral = _value; break;\
    default: break;\
  }\
}
#define PeriodicSendDlValue(_trans, _dev) { \
  static uint8_t i;\
  float var;\
  if (i >= 47) i = 0;\
  switch (i) { \
    case 0: var = telemetry_mode_Ap; break;\
    case 1: var = telemetry_mode_Fbw; break;\
    case 2: var = flight_altitude; break;\
    case 3: var = nav_course; break;\
    case 4: var = nav_shift; break;\
    case 5: var = autopilot_flight_time; break;\
    case 6: var = nav_radius; break;\
    case 7: var = pprz_mode; break;\
    case 8: var = launch; break;\
    case 9: var = kill_throttle; break;\
    case 10: var = gps.reset; break;\
    case 11: var = ap_state->command_roll_trim; break;\
    case 12: var = ap_state->command_pitch_trim; break;\
    case 13: var = h_ctl_roll_pgain; break;\
    case 14: var = h_ctl_roll_max_setpoint; break;\
    case 15: var = h_ctl_pitch_pgain; break;\
    case 16: var = h_ctl_pitch_dgain; break;\
    case 17: var = h_ctl_elevator_of_roll; break;\
    case 18: var = h_ctl_aileron_of_throttle; break;\
    case 19: var = h_ctl_roll_attitude_gain; break;\
    case 20: var = h_ctl_roll_rate_gain; break;\
    case 21: var = v_ctl_altitude_pgain; break;\
    case 22: var = v_ctl_auto_throttle_cruise_throttle; break;\
    case 23: var = v_ctl_auto_throttle_pgain; break;\
    case 24: var = v_ctl_auto_throttle_igain; break;\
    case 25: var = v_ctl_auto_throttle_dgain; break;\
    case 26: var = v_ctl_auto_throttle_climb_throttle_increment; break;\
    case 27: var = v_ctl_auto_throttle_pitch_of_vz_pgain; break;\
    case 28: var = v_ctl_auto_throttle_pitch_of_vz_dgain; break;\
    case 29: var = v_ctl_auto_pitch_pgain; break;\
    case 30: var = v_ctl_auto_pitch_igain; break;\
    case 31: var = h_ctl_course_pgain; break;\
    case 32: var = h_ctl_course_dgain; break;\
    case 33: var = h_ctl_course_pre_bank_correction; break;\
    case 34: var = nav_glide_pitch_trim; break;\
    case 35: var = h_ctl_roll_slew; break;\
    case 36: var = nav_radius; break;\
    case 37: var = nav_course; break;\
    case 38: var = nav_mode; break;\
    case 39: var = nav_climb; break;\
    case 40: var = fp_pitch; break;\
    case 41: var = nav_shift; break;\
    case 42: var = nav_ground_speed_setpoint; break;\
    case 43: var = nav_ground_speed_pgain; break;\
    case 44: var = nav_survey_shift; break;\
    case 45: var = ins_roll_neutral; break;\
    case 46: var = ins_pitch_neutral; break;\
    default: var = 0.; break;\
  }\
  DOWNLINK_SEND_DL_VALUE(_trans, _dev, &i, &var);\
  i++;\
}
static inline float settings_get_value(uint8_t i) {
  switch (i) { \
    case 0: return telemetry_mode_Ap;
    case 1: return telemetry_mode_Fbw;
    case 2: return flight_altitude;
    case 3: return nav_course;
    case 4: return nav_shift;
    case 5: return autopilot_flight_time;
    case 6: return nav_radius;
    case 7: return pprz_mode;
    case 8: return launch;
    case 9: return kill_throttle;
    case 10: return gps.reset;
    case 11: return ap_state->command_roll_trim;
    case 12: return ap_state->command_pitch_trim;
    case 13: return h_ctl_roll_pgain;
    case 14: return h_ctl_roll_max_setpoint;
    case 15: return h_ctl_pitch_pgain;
    case 16: return h_ctl_pitch_dgain;
    case 17: return h_ctl_elevator_of_roll;
    case 18: return h_ctl_aileron_of_throttle;
    case 19: return h_ctl_roll_attitude_gain;
    case 20: return h_ctl_roll_rate_gain;
    case 21: return v_ctl_altitude_pgain;
    case 22: return v_ctl_auto_throttle_cruise_throttle;
    case 23: return v_ctl_auto_throttle_pgain;
    case 24: return v_ctl_auto_throttle_igain;
    case 25: return v_ctl_auto_throttle_dgain;
    case 26: return v_ctl_auto_throttle_climb_throttle_increment;
    case 27: return v_ctl_auto_throttle_pitch_of_vz_pgain;
    case 28: return v_ctl_auto_throttle_pitch_of_vz_dgain;
    case 29: return v_ctl_auto_pitch_pgain;
    case 30: return v_ctl_auto_pitch_igain;
    case 31: return h_ctl_course_pgain;
    case 32: return h_ctl_course_dgain;
    case 33: return h_ctl_course_pre_bank_correction;
    case 34: return nav_glide_pitch_trim;
    case 35: return h_ctl_roll_slew;
    case 36: return nav_radius;
    case 37: return nav_course;
    case 38: return nav_mode;
    case 39: return nav_climb;
    case 40: return fp_pitch;
    case 41: return nav_shift;
    case 42: return nav_ground_speed_setpoint;
    case 43: return nav_ground_speed_pgain;
    case 44: return nav_survey_shift;
    case 45: return ins_roll_neutral;
    case 46: return ins_pitch_neutral;
    default: return 0.;
    }
  }

/* Persistent Settings */
struct PersistentSettings {
};

extern struct PersistentSettings pers_settings;

static inline void persistent_settings_store( void ) {
}

static inline void persistent_settings_load( void ) {
};

#endif // SETTINGS_H
