/*
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
 * Copyright (C) 2012 Gautier Hattenberger
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

/** @file actuators.c
 *  Hardware independent actuators code.
 *  独立于硬件执行器的代码
 */

#include "subsystems/actuators.h"
#include "mcu_periph/sys_time.h"

#if ACTUATORS_NB

int16_t actuators[ACTUATORS_NB];

uint32_t actuators_delay_time;
bool_t   actuators_delay_done;

void actuators_init(void)
 {
//
#if defined ACTUATORS_START_DELAY && ! defined SITL
  actuators_delay_done = FALSE;
  SysTimeTimerStart(actuators_delay_time);
#else
  actuators_delay_done = TRUE;
  actuators_delay_time = 0;
#endif

  // Init macro from generated airframe.h
  //根据生成的airframe.h初始化宏
  AllActuatorsInit();

}

#endif
