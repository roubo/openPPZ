/*
 *
 * Copyright (C) 2012, Christophe De Wagter
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
 *
 */

/**
 * @file modules/nav/nav_catapult.h
 * @brief catapult launch timing system  定时发射系统
 *
 *
 * - Phase 1: Zero Roll, Climb Pitch, Zero Throttle
 * - Phase 2: After detecting the Start Acceleration\n
 *            Zero Roll, Climb Pitch, Full Throttle
 * - Phase 3: After getting the GPS heading (time based)\n
 *            Place climb 300m in front of us\n
 *            GoTo(climb)
 */

 //  第一阶段：零滚转，府仰爬行，没有油门
 //  第二阶段：在检测到起始加速度之后，零滚转，府仰爬行，推满油门
 //  第三阶段：在获得ＧＰＳ，飞机在我们前方300米之后，ＧｏＴｏ

#include "generated/airframe.h"
#include "state.h"
#include "ap_downlink.h"
#include "modules/nav/nav_catapult.h"
#include "subsystems/nav.h"
#include "generated/flight_plan.h"
#include "firmwares/fixedwing/autopilot.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"

// Imu is required
#include "subsystems/imu.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/datalink.h"


static bool_t nav_catapult_armed = FALSE;
static uint16_t nav_catapult_launch = 0;

#ifndef NAV_CATAPULT_ACCELERATION_THRESHOLD
#define NAV_CATAPULT_ACCELERATION_THRESHOLD 1.5
#endif

float nav_catapult_acceleration_threshold = NAV_CATAPULT_ACCELERATION_THRESHOLD;

#ifndef NAV_CATAPULT_MOTOR_DELAY
#define NAV_CATAPULT_MOTOR_DELAY  0.75		// seconds
#endif

float nav_catapult_motor_delay = NAV_CATAPULT_MOTOR_DELAY;

#ifndef NAV_CATAPULT_HEADING_DELAY
#define NAV_CATAPULT_HEADING_DELAY 3.0  // seconds
#endif

float nav_catapult_heading_delay = NAV_CATAPULT_HEADING_DELAY;

#ifndef NAV_CATAPULT_INITIAL_PITCH
#define NAV_CATAPULT_INITIAL_PITCH RadOfDeg(10)
#endif

float nav_catapult_initial_pitch = NAV_CATAPULT_INITIAL_PITCH;

#ifndef NAV_CATAPULT_INITIAL_THROTTLE
#define NAV_CATAPULT_INITIAL_THROTTLE 1.0
#endif

float nav_catapult_initial_throttle = NAV_CATAPULT_INITIAL_THROTTLE;

/////// Store Take-Off Point   存储起飞点

static float nav_catapult_x = 0;
static float nav_catapult_y = 0;

//###############################################################################################
// Code that Runs in a Fast Module  快速模式的代码

void nav_catapult_highrate_module(void)
{
  // Only run when  只在跑（飞）的时候执行
  if (nav_catapult_armed)
  {
    if (nav_catapult_launch < nav_catapult_heading_delay * NAV_CATAPULT_HIGHRATE_MODULE_FREQ) {
      nav_catapult_launch++;
    }

    // Launch detection Filter  再次判断（滤波）
    if (nav_catapult_launch < 5)
    {
      // Five consecutive measurements > 1.5   五个连续的测量会大于1.5
#ifndef SITL
      struct Int32Vect3 accel_meas_body;
      INT32_RMAT_TRANSP_VMULT(accel_meas_body, imu.body_to_imu_rmat, imu.accel);
      if (ACCEL_FLOAT_OF_BFP(accel_meas_body.x)  < (nav_catapult_acceleration_threshold * 9.81))
#else
      if (launch != 1)
#endif
      {
        nav_catapult_launch = 0;
      }
    }
    // Launch was detected: Motor Delay Counter  检测到发射：电机持续
    else if (nav_catapult_launch >= nav_catapult_motor_delay * NAV_CATAPULT_HIGHRATE_MODULE_FREQ)
    {
      // Turn on Motor 打开点击，设置油门
      NavVerticalThrottleMode(9600*(nav_catapult_initial_throttle));
      launch = 1;
    }
  }
  else
  {
    nav_catapult_launch = 0;
  }
}

//###############################################################################################
// Code that runs in 4Hz Nav

bool_t nav_catapult_init(void)
{

  nav_catapult_armed = TRUE;
  nav_catapult_launch = 0;

  return FALSE;
}



bool_t nav_catapult(uint8_t _to, uint8_t _climb)
{
  float alt = WaypointAlt(_climb);  

  nav_catapult_armed = 1;

  // No Roll, Climb Pitch, No motor Phase   零滚转 府仰爬行 没有电机阶段
  if (nav_catapult_launch <= nav_catapult_motor_delay * NAV_CATAPULT_HIGHRATE_MODULE_FREQ)
  {
    NavAttitude(RadOfDeg(0));  //高度设置
    NavVerticalAutoThrottleMode(nav_catapult_initial_pitch);   //自动油门模式
    NavVerticalThrottleMode(9600*(0));   //设定油门


    // Store take-off waypoint   存储起飞点
    WaypointX(_to) = GetPosX();   //获得ｘ坐标
    WaypointY(_to) = GetPosY();   //获得ｙ坐标
    WaypointAlt(_to) = GetPosAlt();   //获得高度

    nav_catapult_x = stateGetPositionEnu_f()->x;   //起飞点ｘ坐标
    nav_catapult_y = stateGetPositionEnu_f()->y;   //起飞点ｙ坐标

  }
  // No Roll, Climb Pitch, Full Power   零滚转  府仰爬行  满油门
  else if (nav_catapult_launch < nav_catapult_heading_delay * NAV_CATAPULT_HIGHRATE_MODULE_FREQ)
  {
    NavAttitude(RadOfDeg(0));   //高度设置
    NavVerticalAutoThrottleMode(nav_catapult_initial_pitch);   //自动油门模式
    NavVerticalThrottleMode(9600*(nav_catapult_initial_throttle));   //设定油门
  }
  // Normal Climb: Heading Locked by Waypoint Target    
  // 正常爬行：锁定给定航点
  else if (nav_catapult_launch == 0xffff)
  {
    NavVerticalAltitudeMode(alt, 0);	// vertical mode (folow glideslope)  水平模式（跟随滑坡）
    NavVerticalAutoThrottleMode(0);		// throttle mode  油门模式
    NavGotoWaypoint(_climb);				// horizontal mode (stay on localiser)   垂直模式（保持定位）
  }
  else
  {
    // Store Heading, move Climb   
    nav_catapult_launch = 0xffff;

    float dir_x = stateGetPositionEnu_f()->x - nav_catapult_x;
    float dir_y = stateGetPositionEnu_f()->y - nav_catapult_y;

    float dir_L = sqrt(dir_x * dir_x + dir_y * dir_y);

    WaypointX(_climb) = nav_catapult_x + (dir_x / dir_L) * 300;
    WaypointY(_climb) = nav_catapult_y + (dir_y / dir_L) * 300;

    DownlinkSendWp(DefaultChannel, DefaultDevice, _climb);
  }


return TRUE;

}

bool_t nav_select_touch_down(uint8_t _td)
{
  WaypointX(_td) = GetPosX();
  WaypointY(_td) = GetPosY();
  WaypointAlt(_td) = GetPosAlt();
  return FALSE;
}

