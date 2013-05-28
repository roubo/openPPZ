/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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

/**
 * @file subsystems/ins.h
 * Integrated Navigation System interface.   完整的导航系统接口
 */

#ifndef INS_H
#define INS_H

#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_float.h"
#include "state.h"

#define INS_UNINIT  0
#define INS_RUNNING 1

/* underlying includes (needed for parameters) */
#ifdef INS_TYPE_H
#include INS_TYPE_H
#endif

/** Inertial Navigation System state */
 //惯性导航系统声明
struct Ins {
  uint8_t status; ///< status of the INS  INS的状态
  bool_t hf_realign; ///< realign horizontally if true  如果是真的，水平重组
  bool_t vf_realign; ///< realign vertically if true   如果是真的，垂直重组
};

/** global INS state */
extern struct Ins ins;

/** INS initialization. Called at startup.   INS初始化，在开始时调用
 *  Needs to be implemented by each INS algorithm.   在每一个INS算法中需要被执行
 */
extern void ins_init( void );

/** INS periodic call.  INS周期调用
 *  Needs to be implemented by each INS algorithm.  在每一个INS算法中需要被执行
 */
extern void ins_periodic( void );

/** INS horizontal realign.  INS水平重组
 *  @param pos new horizontal position to set   设置新的水平位置
 *  @param speed new horizontal speed to set    设置新的水平速度
 *  Needs to be implemented by each INS algorithm.   在每一个INS算法中需要被执行
 */
extern void ins_realign_h(struct FloatVect2 pos, struct FloatVect2 speed);

/** INS vertical realign.   INS垂直重组
 *  @param z new altitude to set   设置新的高度
 *  Needs to be implemented by each INS algorithm.   在每一个INS算法中需要被执行
 */
extern void ins_realign_v(float z);

/** Propagation. Usually integrates the gyro rates to angles.  传递。经常传递陀螺仪的角速度
 *  Reads the global #imu data struct.  读取IMU数据结构
 *  Needs to be implemented by each INS algorithm.    在每一个INS算法中需要被执行
 */
extern void ins_propagate( void );

/** Update INS state with barometer measurements.   通过气压计测量值更新INS状态
 *  Reads the global #baro data struct.   读取气压计数据结构
 *  Needs to be implemented by each INS algorithm.   在每一个INS算法中需要被执行
 */
extern void ins_update_baro( void );

/** Update INS state with GPS measurements.   通过GPS测量值更新INS
 *  Reads the global #gps data struct.   读取gps数据结构
 *  Needs to be implemented by each INS algorithm.   在每一个INS算法中需要被执行
 */
extern void ins_update_gps( void );

/** Update INS state with sonar measurements.   通过声呐数据更新INS
 *  Reads the global #sonar data struct.读取声呐数据结构
 *  Needs to be implemented by each INS algorithm.  在每一个INS算法中需要被执行
 */
extern void ins_update_sonar( void );


#endif /* INS_H */
