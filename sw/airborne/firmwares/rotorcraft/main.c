/*
 * Copyright (C) 2008-2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file firmwares/rotorcraft/main.c
 *
 * Rotorcraft main loop.
 * 旋翼机的主循环！！！
 */

#define MODULES_C

#include <inttypes.h>//一些常用的变量类型
#include "mcu.h"//mcu的初始化：mcu_init(),包括adc,spi，sys_time,mcu_arch,led等的初始化
#include "mcu_periph/sys_time.h"//子系统时钟
#include "led.h"//Led的初始化，LED_INIT,LED_OFF,LED_TROGGLE,LED_PERIODIC

#include "subsystems/datalink/downlink.h"// 从通道调用传送函数 的宏
#include "firmwares/rotorcraft/telemetry.h"// 旋翼机的要测系统 的宏
#include "subsystems/datalink/datalink.h"// 三种通讯协议的 宏 PPRZ ，XBEE ，W5100
#include "subsystems/settings.h"// 主函数初始化 后的设置init,store
#include "subsystems/datalink/xbee.h"// xbee的宏定义

#include "subsystems/commands.h"//  命令command 
#include "subsystems/actuators.h"// 传动机构设置
#if USE_MOTOR_MIXING
#include "subsystems/actuators/motor_mixing.h"//传动机构的混合电机
#endif

#include "subsystems/imu.h"// imu
#include "subsystems/gps.h"// gps

#include "subsystems/sensors/baro.h"//baro 气压计

#include "subsystems/electrical.h"//

#include "firmwares/rotorcraft/autopilot.h"//旋翼的飞控

#include "firmwares/rotorcraft/stabilization.h"//旋翼的稳态
#include "firmwares/rotorcraft/guidance.h"//旋翼的引导？？

#include "subsystems/ahrs.h"//ahrs 
#include "subsystems/ins.h"//ins惯性导航系统

#include "state.h"//状态

#include "firmwares/rotorcraft/main.h"

#ifdef SITL
#include "nps_autopilot_rotorcraft.h"
#endif

#include "generated/modules.h"//未找到该头文件？？


/* if PRINT_CONFIG is defined, print some config options */
/*可打印消息：Config:Ponfig:PERIODIC_FREQUENCY=60
 *            Config:Ponfig:MODULES_FREQUENCY=512
 *            Config:Ponfig:BARO_PERIODIC_FREQUENCY=50
 */
PRINT_CONFIG_VAR(PERIODIC_FREQUENCY)

#ifndef MODULES_FREQUENCY
#define MODULES_FREQUENCY 512
#endif
PRINT_CONFIG_VAR(MODULES_FREQUENCY)

#ifndef BARO_PERIODIC_FREQUENCY
#define BARO_PERIODIC_FREQUENCY 50
#endif
PRINT_CONFIG_VAR(BARO_PERIODIC_FREQUENCY)


static inline void on_gyro_event( void );
static inline void on_accel_event( void );
static inline void on_baro_abs_event( void );
static inline void on_baro_dif_event( void );
static inline void on_gps_event( void );
static inline void on_mag_event( void );


tid_t main_periodic_tid; ///< id for main_periodic() timer                main_periodic()的定时器ID
tid_t modules_tid;       ///< id for modules_periodic_task() timer        modules_periodic_task()的定时器ID
tid_t failsafe_tid;      ///< id for failsafe_check() timer               failsafe_check()的定时器ID
tid_t radio_control_tid; ///< id for radio_control_periodic_task() timer  radio_control_periodic_task()定时器ID
tid_t electrical_tid;    ///< id for electrical_periodic() timer          electrial_periodic()定时器ID
tid_t baro_tid;          ///< id for baro_periodic() timer                baro_periodic()定时器ID baro:气压计
tid_t telemetry_tid;     ///< id for telemetry_periodic() timer           telemetry_periodic()定时器ID：遥测

#ifndef SITL
int main( void ) {
  main_init();//主函数初始化

  while(1) {
    handle_periodic_tasks();//周期性任务处理
    main_event();//主要处理任务
  }
  return 0;
}
#endif /* SITL */

STATIC_INLINE void main_init( void ) {

  mcu_init();

  electrical_init();//注册一个缓冲区用来存储特殊的通道转换，给ADC通道的？？

  stateInit();//状态初始化

  actuators_init();//执行机构初始化
#if USE_MOTOR_MIXING
  motor_mixing_init();
#endif

  radio_control_init();//RC初始化

  baro_init();//气压传感器初始化
  imu_init();//imu初始化
  autopilot_init();//飞控初始化
  nav_init();//导航模式初始化
  guidance_h_init();//引导模式水平方向参数初始化
  guidance_v_init();//引导模式垂直方向参数初始化
  stabilization_init();//稳定模式初始化

  ahrs_aligner_init();//arhs校准器初始化
  ahrs_init();//arhs初始化

  ins_init();//惯性导航系统初始化

#if USE_GPS
  gps_init();//GPS初始化
#endif

  modules_init();//？？

  settings_init();//设置初始化

  mcu_int_enable();//mcu 初始化使能

#if DATALINK == XBEE
  xbee_init();//数传使用的是xbee
#endif

  // register the timers for the periodic functions
  //通过周期函数来记录时间
  main_periodic_tid = sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  modules_tid = sys_time_register_timer(1./MODULES_FREQUENCY, NULL);
  radio_control_tid = sys_time_register_timer((1./60.), NULL);
  failsafe_tid = sys_time_register_timer(0.05, NULL);
  electrical_tid = sys_time_register_timer(0.1, NULL);
  baro_tid = sys_time_register_timer(1./BARO_PERIODIC_FREQUENCY, NULL);
  telemetry_tid = sys_time_register_timer((1./60.), NULL);
}
/*功能：处理周期性任务
 *      包括：主函数，module ,RC,失效保护模式检测，ADC的供压计算
 *            气压计的状态检测，遥测信息检测
 **/
STATIC_INLINE void handle_periodic_tasks( void ) {
  if (sys_time_check_and_ack_timer(main_periodic_tid))
    main_periodic();
  if (sys_time_check_and_ack_timer(modules_tid))
    modules_periodic_task();//该版本新加的？？？未找到该函数的声明
  if (sys_time_check_and_ack_timer(radio_control_tid))
    radio_control_periodic_task();//RC的周期性任务：RC的状态和板载RC_led指示灯
  if (sys_time_check_and_ack_timer(failsafe_tid))
    failsafe_check();//失效保护模式检测
  if (sys_time_check_and_ack_timer(electrical_tid))
    electrical_periodic();//ADC的供压计算
  if (sys_time_check_and_ack_timer(baro_tid))
    baro_periodic();//气压计的状态检测，读压力和温度数据
  if (sys_time_check_and_ack_time_periodic_task();(telemetry_tid))
    telemetry_periodic();//遥测信息检测
}

STATIC_INLINE void main_periodic( void ) {
 
  /*imu周期处理函数：包括adc_max1168的读取，磁力计_hmc58xx的读取*/
  imu_periodic();

  /* run control loops */
  /* 运行控制环:设置飞控的模式->失效保护模式 or Kill模式
   * 并且计算了失效保护模式下的两个方向水平h和垂直v的姿态信息
   */
  autopilot_periodic();

  /* set actuators     */
  /* 设置执行机构*/
  //actuators_set(autopilot_motors_on);
  /* 根据命令设置执行机构的操作*/
  SetActuatorsFromCommands(commands);
  
  /*如果是飞行模式下：飞控飞行时间，通信时间均以固定频率增加*/
  if (autopilot_in_flight) {
    RunOnceEvery(PERIODIC_FREQUENCY, { autopilot_flight_time++; datalink_time++; });
  }
  /*Led的周期性执行（闪烁）*/
  RunOnceEvery(10, LED_PERIODIC());
}
//定时向主函数发送要测信息，使用的是DOWNLINK协议
STATIC_INLINE void telemetry_periodic(void) {
  PeriodicSendMain(DefaultChannel,DefaultDevice);
}

STATIC_INLINE void failsafe_check( void ) {
  //设定飞控为失效保护模式
  if (radio_control.status != RC_OK &&
      autopilot_mode != AP_MODE_KILL &&
      autopilot_mode != AP_MODE_NAV)
  {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }

#if USE_GPS//使用GPS
  if (autopilot_mode == AP_MODE_NAV &&
#if NO_GPS_LOST_WITH_RC_VALID
      radio_control.status != RC_OK &&
#endif
      GpsIsLost())//GPS是否丢失
  {
    autopilot_set_mode(AP_MODE_FAILSAFE);
  }
#endif
}

STATIC_INLINE void main_event( void ) {

  i2c_event();//使用开门狗定时器作为I2C1和I2C2的中断处理定时器

  DatalinkEvent();//定义数据协议的类型：XBEE ，PPRZ ，W5100
  
  //判断是否使用遥控器
  if (autopilot_rc) {
    RadioControlEvent(autopilot_on_rc_frame);//RC接收数据处理
  }

  ImuEvent(on_gyro_event, on_accel_event, on_mag_event);//IMU单元的陀螺仪，加速度计，磁力计处理

  BaroEvent(on_baro_abs_event, on_baro_dif_event);//气压计数据处理

#if USE_GPS
  GpsEvent(on_gps_event);//GPS数据处理
#endif
/*地面 失效保护检测 或者 地面关闭？？检测*/
#if FAILSAFE_GROUND_DETECT || KILL_ON_GROUND_DETECT
  DetectGroundEvent();//地面战检测处理
#endif

  modules_event_task();

}

static inline void on_accel_event( void ) {
  ImuScaleAccel(imu);//imu测量三轴加速度

  if (ahrs.status != AHRS_UNINIT) {
    ahrs_update_accel();//通过加速度计的测量来更新AHRS的状态
  }
}

static inline void on_gyro_event( void ) {

  ImuScaleGyro(imu);//imu测量陀螺仪三轴姿态

  if (ahrs.status == AHRS_UNINIT) {
    ahrs_aligner_run();//运行ahrs校准器
    if (ahrs_aligner.status == AHRS_ALIGNER_LOCKED)
      ahrs_align();
  }
  else {
    ahrs_propagate();//ahrs（groy）速度到角度的整合
#ifdef SITL
    if (nps_bypass_ahrs) sim_overwrite_ahrs();
#endif
    ins_propagate();//ahrs(groy)速度到角度的整合
  }
#ifdef USE_VEHICLE_INTERFACE
  vi_notify_imu_available();
#endif
}

static inline void on_baro_abs_event( void ) {
  ins_update_baro();//气压计数据更新
#ifdef USE_VEHICLE_INTERFACE
  vi_notify_baro_abs_available();
#endif
}

static inline void on_baro_dif_event( void ) {

}

static inline void on_gps_event(void) {
  ins_update_gps();//gps数据更新
#ifdef USE_VEHICLE_INTERFACE
  if (gps.fix == GPS_FIX_3D)
    vi_notify_gps_available();
#endif
}

static inline void on_mag_event(void) {
  ImuScaleMag(imu); //imu单元中磁力计的测量

#if USE_MAGNETOMETER
  if (ahrs.status == AHRS_RUNNING) {
    ahrs_update_mag();//更新磁力计数据
  }
#endif

#ifdef USE_VEHICLE_INTERFACE
  vi_notify_mag_available();
#endif
}
