/*
 * Copyright (C) 2003-2011 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file gps.h
 *  @brief Device independent GPS code (interface)
 *  独立的gps代码
 *
 */

#ifndef GPS_H
#define GPS_H


#include "std.h"
#include "math/pprz_geodetic_int.h"

#include "mcu_periph/sys_time.h"

/* GPS model specific implementation or sim */
#ifdef GPS_TYPE_H
#include GPS_TYPE_H
#endif

#define GPS_FIX_NONE 0x00
#define GPS_FIX_2D   0x02
#define GPS_FIX_3D   0x03

#define GpsFixValid() (gps.fix == GPS_FIX_3D)


#ifndef GPS_NB_CHANNELS
#define GPS_NB_CHANNELS 1
#endif

/** data structure for Space Vehicle Information of a single satellite */
//一个单一卫星的数据结构
struct SVinfo {
  uint8_t svid;  ///< Satellite ID   卫星ID号
  uint8_t flags; ///< bitfield with GPS receiver specific flags  gps接收机的特定标志
  uint8_t qi;    ///< quality bitfield (GPS receiver specific)  gps接收机特定的特性
  uint8_t cno;   ///< Carrier to Noise Ratio (Signal Strength) in dbHz   信噪比
  int8_t elev;   ///< elevation in deg  高度
  int16_t azim;  ///< azimuth in deg  方位角
};

/** data structure for GPS information */
  //gps信息的数据结构
struct GpsState {
  struct EcefCoor_i ecef_pos;    ///< position in ECEF in cm  ecef坐标系
  struct LlaCoor_i lla_pos;      ///< position in LLA (lat,lon: rad*1e7; alt: mm over ellipsoid)  lla坐标系
  struct UtmCoor_i utm_pos;      ///< position in UTM (north,east: cm; alt: mm over ellipsoid)   utm坐标系
  int32_t hmsl;                  ///< height above mean sea level in mm   海平面高度
  struct EcefCoor_i ecef_vel;    ///< speed ECEF in cm/s  ecef坐标系下的速度
  struct NedCoor_i ned_vel;      ///< speed NED in cm/s   NED坐标系下的速度
  int16_t gspeed;                ///< norm of 2d ground speed in cm/s   二维速度
  int16_t speed_3d;              ///< norm of 3d speed in cm/s  三维速度
  int32_t course;                ///< GPS course over ground in rad*1e7, [0, 2*Pi]*1e7 (CW/north)   GPS在地面上的进程
  uint32_t pacc;                 ///< position accuracy in cm   位置的精确度
  uint32_t sacc;                 ///< speed accuracy in cm/s   速度的精确度
  uint32_t cacc;                 ///< course accuracy in rad*1e7  进程的精确度
  uint16_t pdop;                 ///< position dilution of precision scaled by 100  位置精度被降低100倍
  uint8_t num_sv;                ///< number of sat in fix   数目修复
  uint8_t fix;                   ///< status of fix   状态修复
  int16_t week;                  ///< GPS week
  uint32_t tow;                  ///< GPS time of week in ms   GPS周期

  uint8_t nb_channels;           ///< Number of scanned satellites   扫描到的卫星的数目
  struct SVinfo svinfos[GPS_NB_CHANNELS]; ///< holds information from the Space Vehicles (Satellites)    从卫星获得的信息

  uint32_t last_fix_ticks;       ///< cpu time ticks at last valid fix  最后一次有效修复的cpu滴答时钟
  uint32_t last_fix_time;        ///< cpu time in sec at last valid fix  最后一次有效修复的cpu时间
  uint16_t reset;                ///< hotstart, warmstart, coldstart  热启动  温启动  冷启动
};

/** data structure for GPS time sync */
  //gps 同步时钟的数据结构
struct GpsTimeSync {
  uint32_t t0_tow;      ///< GPS time of week in ms from last message   GPS周期
  int32_t t0_tow_frac;  ///< fractional ns remainder of tow [ms], range -500000 .. 500000
  uint32_t t0_ticks;    ///< hw clock ticks when GPS message is received
};

/** global GPS state */
extern struct GpsState gps;

/** initialize the global GPS state */
extern void gps_init(void);

/* GPS model specific init implementation */
extern void gps_impl_init(void);


/* mark GPS as lost when no valid 3D fix was received for GPS_TIMEOUT secs */
#ifndef GPS_TIMEOUT
#define GPS_TIMEOUT 5
#endif

inline bool_t GpsIsLost(void);

inline bool_t GpsIsLost(void) {
  if (sys_time.nb_sec - gps.last_fix_time > GPS_TIMEOUT) {
    gps.fix = GPS_FIX_NONE;
    return TRUE;
  }
  return FALSE;
}


/**
 * GPS Reset  重置ｇｐｓ
 * @todo this still needs to call gps specific stuff
 × 只有需要调用ｇｐｓ特殊的东西时候才做这
 */
#define gps_Reset(_val) {                               \
}


/*
 * For GPS time synchronizaiton...   ｇｐｓ同步
 */
extern struct GpsTimeSync gps_time_sync;

/**
 * Convert time in sys_time ticks to GPS time of week.   在同步的滴答时钟下转换时间
 * The resolution is sys_time.resolution
 * @return GPS tow in ms
 */
extern uint32_t gps_tow_from_sys_ticks(uint32_t sys_ticks);

#endif /* GPS_H */
