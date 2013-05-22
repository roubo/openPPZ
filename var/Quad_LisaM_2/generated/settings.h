/* This file has been generated from /home/chorus/chorus/var/Quad_LisaM_2/settings_modules.xml /home/chorus/chorus/conf/settings/control/stabilization_rate.xml /home/chorus/chorus/conf/settings/control/stabilization_att_int.xml /home/chorus/chorus/conf/settings/control/rotorcraft_guidance.xml /home/chorus/chorus/conf/settings/rotorcraft_basic.xml /home/chorus/chorus/var/Quad_LisaM_2/settings_telemetry.xml */
/* Please DO NOT EDIT */

#ifndef SETTINGS_H
#define SETTINGS_H

#define RCSettings(mode_changed) { \
}

#include "autopilot.h"
#include "guidance/guidance_h.h"
#include "guidance/guidance_v.h"
#include "navigation.h"
#include "stabilization/stabilization_attitude.h"
#include "stabilization/stabilization_rate.h"
#include "subsystems/ins.h"
#include "generated/modules.h"
#include "generated/periodic_telemetry.h"

#define DlSetting(_idx, _value) { \
  switch (_idx) { \
    case 0: telemetry_mode_Main = _value; break;\
    case 1: autopilot_mode_auto2 = _value; break;\
    case 2: autopilot_KillThrottle( _value ); _value = kill_throttle; break;\
    case 3: autopilot_SetPowerSwitch( _value ); _value = autopilot_power_switch; break;\
    case 4: guidance_v_kp = _value; break;\
    case 5: guidance_v_kd = _value; break;\
    case 6: guidance_v_SetKi( _value ); _value = guidance_v_ki; break;\
    case 7: guidance_v_nominal_throttle = _value; break;\
    case 8: guidance_v_z_sp = _value; break;\
    case 9: ins.vf_realign = _value; break;\
    case 10: guidance_h_SetUseRef( _value ); _value = guidance_h_use_ref; break;\
    case 11: guidance_h_pgain = _value; break;\
    case 12: guidance_h_dgain = _value; break;\
    case 13: guidance_h_SetKi( _value ); _value = guidance_h_igain; break;\
    case 14: guidance_h_again = _value; break;\
    case 15: guidance_h_pos_sp.x = _value; break;\
    case 16: guidance_h_pos_sp.y = _value; break;\
    case 17: ins.hf_realign = _value; break;\
    case 18: navigation_SetFlightAltitude( _value ); _value = flight_altitude; break;\
    case 19: nav_heading = _value; break;\
    case 20: nav_radius = _value; break;\
    case 21: stabilization_gains.p.x = _value; break;\
    case 22: stabilization_gains.d.x = _value; break;\
    case 23: stabilization_attitude_SetKiPhi( _value ); _value = stabilization_gains.i.x; break;\
    case 24: stabilization_gains.dd.x = _value; break;\
    case 25: stabilization_gains.p.y = _value; break;\
    case 26: stabilization_gains.d.y = _value; break;\
    case 27: stabilization_gains.i.y = _value; break;\
    case 28: stabilization_gains.dd.y = _value; break;\
    case 29: stabilization_gains.p.z = _value; break;\
    case 30: stabilization_gains.d.z = _value; break;\
    case 31: stabilization_gains.i.z = _value; break;\
    case 32: stabilization_gains.dd.z = _value; break;\
    case 33: stabilization_rate_gain.p = _value; break;\
    case 34: stabilization_rate_gain.q = _value; break;\
    case 35: stabilization_rate_gain.r = _value; break;\
    case 36: stabilization_rate_igain.p = _value; break;\
    case 37: stabilization_rate_igain.q = _value; break;\
    case 38: stabilization_rate_igain.r = _value; break;\
    case 39: stabilization_rate_ddgain.p = _value; break;\
    case 40: stabilization_rate_ddgain.q = _value; break;\
    case 41: stabilization_rate_ddgain.r = _value; break;\
    default: break;\
  }\
}
#define PeriodicSendDlValue(_trans, _dev) { \
  static uint8_t i;\
  float var;\
  if (i >= 42) i = 0;\
  switch (i) { \
    case 0: var = telemetry_mode_Main; break;\
    case 1: var = autopilot_mode_auto2; break;\
    case 2: var = kill_throttle; break;\
    case 3: var = autopilot_power_switch; break;\
    case 4: var = guidance_v_kp; break;\
    case 5: var = guidance_v_kd; break;\
    case 6: var = guidance_v_ki; break;\
    case 7: var = guidance_v_nominal_throttle; break;\
    case 8: var = guidance_v_z_sp; break;\
    case 9: var = ins.vf_realign; break;\
    case 10: var = guidance_h_use_ref; break;\
    case 11: var = guidance_h_pgain; break;\
    case 12: var = guidance_h_dgain; break;\
    case 13: var = guidance_h_igain; break;\
    case 14: var = guidance_h_again; break;\
    case 15: var = guidance_h_pos_sp.x; break;\
    case 16: var = guidance_h_pos_sp.y; break;\
    case 17: var = ins.hf_realign; break;\
    case 18: var = flight_altitude; break;\
    case 19: var = nav_heading; break;\
    case 20: var = nav_radius; break;\
    case 21: var = stabilization_gains.p.x; break;\
    case 22: var = stabilization_gains.d.x; break;\
    case 23: var = stabilization_gains.i.x; break;\
    case 24: var = stabilization_gains.dd.x; break;\
    case 25: var = stabilization_gains.p.y; break;\
    case 26: var = stabilization_gains.d.y; break;\
    case 27: var = stabilization_gains.i.y; break;\
    case 28: var = stabilization_gains.dd.y; break;\
    case 29: var = stabilization_gains.p.z; break;\
    case 30: var = stabilization_gains.d.z; break;\
    case 31: var = stabilization_gains.i.z; break;\
    case 32: var = stabilization_gains.dd.z; break;\
    case 33: var = stabilization_rate_gain.p; break;\
    case 34: var = stabilization_rate_gain.q; break;\
    case 35: var = stabilization_rate_gain.r; break;\
    case 36: var = stabilization_rate_igain.p; break;\
    case 37: var = stabilization_rate_igain.q; break;\
    case 38: var = stabilization_rate_igain.r; break;\
    case 39: var = stabilization_rate_ddgain.p; break;\
    case 40: var = stabilization_rate_ddgain.q; break;\
    case 41: var = stabilization_rate_ddgain.r; break;\
    default: var = 0.; break;\
  }\
  DOWNLINK_SEND_DL_VALUE(_trans, _dev, &i, &var);\
  i++;\
}
static inline float settings_get_value(uint8_t i) {
  switch (i) { \
    case 0: return telemetry_mode_Main;
    case 1: return autopilot_mode_auto2;
    case 2: return kill_throttle;
    case 3: return autopilot_power_switch;
    case 4: return guidance_v_kp;
    case 5: return guidance_v_kd;
    case 6: return guidance_v_ki;
    case 7: return guidance_v_nominal_throttle;
    case 8: return guidance_v_z_sp;
    case 9: return ins.vf_realign;
    case 10: return guidance_h_use_ref;
    case 11: return guidance_h_pgain;
    case 12: return guidance_h_dgain;
    case 13: return guidance_h_igain;
    case 14: return guidance_h_again;
    case 15: return guidance_h_pos_sp.x;
    case 16: return guidance_h_pos_sp.y;
    case 17: return ins.hf_realign;
    case 18: return flight_altitude;
    case 19: return nav_heading;
    case 20: return nav_radius;
    case 21: return stabilization_gains.p.x;
    case 22: return stabilization_gains.d.x;
    case 23: return stabilization_gains.i.x;
    case 24: return stabilization_gains.dd.x;
    case 25: return stabilization_gains.p.y;
    case 26: return stabilization_gains.d.y;
    case 27: return stabilization_gains.i.y;
    case 28: return stabilization_gains.dd.y;
    case 29: return stabilization_gains.p.z;
    case 30: return stabilization_gains.d.z;
    case 31: return stabilization_gains.i.z;
    case 32: return stabilization_gains.dd.z;
    case 33: return stabilization_rate_gain.p;
    case 34: return stabilization_rate_gain.q;
    case 35: return stabilization_rate_gain.r;
    case 36: return stabilization_rate_igain.p;
    case 37: return stabilization_rate_igain.q;
    case 38: return stabilization_rate_igain.r;
    case 39: return stabilization_rate_ddgain.p;
    case 40: return stabilization_rate_ddgain.q;
    case 41: return stabilization_rate_ddgain.r;
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
