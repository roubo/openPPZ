/*
 *
 * Copyright (C) 2009-2011 The Paparazzi Team
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
 * @file mcu_periph/sys_time.c
 * @brief Architecture independent timing functions.
 * 独立于架构的时间函数
 */

#include "mcu_periph/sys_time.h"
#include "mcu.h"

PRINT_CONFIG_VAR(SYS_TIME_FREQUENCY)

struct sys_time sys_time;

/*注册一个新的系统定时器 */
int sys_time_register_timer(float duration, sys_time_cb cb) {

  uint32_t start_time = sys_time.nb_tick;//开始时的滴答数
  for (int i = 0; i< SYS_TIME_NB_TIMER; i++) {
    //判断第i个定时器的使用情况
    if (!sys_time.timer[i].in_use) {//没有被使用时
      sys_time.timer[i].cb         = cb;
      sys_time.timer[i].elapsed    = FALSE;//该定时器未溢出
      sys_time.timer[i].end_time   = start_time + sys_time_ticks_of_sec(duration);//结束时的滴答数
      sys_time.timer[i].duration   = sys_time_ticks_of_sec(duration);//持续的滴答数
      sys_time.timer[i].in_use     = TRUE;//定时器使用
      return i;//返回当前定时器的id号
    }
  }
  return -1;
}

/*通过id号删除一个系统定时器*/
void sys_time_cancel_timer(tid_t id) {
  sys_time.timer[id].in_use     = FALSE;
  sys_time.timer[id].cb         = NULL;
  sys_time.timer[id].elapsed    = FALSE;
  sys_time.timer[id].end_time   = 0;
  sys_time.timer[id].duration   = 0;
}

// FIXME: race condition ??
/* 更新定时器持续时间直至定时器计数结束*/
void sys_time_update_timer(tid_t id, float duration) {
  mcu_int_disable();
  sys_time.timer[id].end_time -= (sys_time.timer[id].duration - sys_time_ticks_of_sec(duration));
  sys_time.timer[id].duration = sys_time_ticks_of_sec(duration);
  mcu_int_enable();
}
/*系统定时器初始化*/
void sys_time_init( void ) {
  sys_time.nb_sec     = 0;
  sys_time.nb_sec_rem = 0;
  sys_time.nb_tick    = 0;

  sys_time.ticks_per_sec = SYS_TIME_FREQUENCY;//该值决定了滴答时钟的频率，即=滴答时钟频率
  sys_time.resolution = 1.0 / sys_time.ticks_per_sec;

  for (unsigned int i=0; i<SYS_TIME_NB_TIMER; i++) {
    sys_time.timer[i].in_use     = FALSE;
    sys_time.timer[i].cb         = NULL;
    sys_time.timer[i].elapsed    = FALSE;
    sys_time.timer[i].end_time   = 0;
    sys_time.timer[i].duration   = 0;
  }

  sys_time_arch_init();
}
