/*
 * Copyright (C) 2009 Felix Ruess <felix.ruess@gmail.com>
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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
 * @file subsystems/ahrs/ahrs_float_utils.h
 *
 * Utility functions for floating point AHRS implementations.
 *
 */

#ifndef AHRS_FLOAT_UTILS_H
#define AHRS_FLOAT_UTILS_H

#include "subsystems/ahrs/ahrs_magnetic_field_model.h"

#include "std.h" // for ABS

static inline void ahrs_float_get_euler_from_accel_mag(struct FloatEulers* e, struct Int32Vect3* accel, struct Int32Vect3* mag) {
  /* get phi and theta from accelerometer */
  struct FloatVect3 accelf;
  ACCELS_FLOAT_OF_BFP(accelf, *accel);
  const float phi   = atan2f(-accelf.y, -accelf.z);
  const float cphi = cosf(phi);
  const float theta = atan2f(cphi*accelf.x, -accelf.z);

  /* get psi from magnetometer */
  /* project mag on local tangeant plane */
  struct FloatVect3 magf;
  MAGS_FLOAT_OF_BFP(magf, *mag);
  const float sphi   = sinf(phi);
  const float ctheta = cosf(theta);
  const float stheta = sinf(theta);
  const float mn = ctheta * magf.x + sphi*stheta*magf.y + cphi*stheta*magf.z;
  const float me =     0. * magf.x + cphi       *magf.y - sphi       *magf.z;
  float psi = -atan2f(me, mn) + atan2(AHRS_H_Y, AHRS_H_X);
  if (psi > M_PI) psi -= 2.*M_PI; if (psi < -M_PI) psi+= 2.*M_PI;
  EULERS_ASSIGN(*e, phi, theta, psi);

}

/** Compute a quaternion representing roll and pitch from an accelerometer measurement. */
//根据加速度计的测量值来计算一个四元数来代表roll(滚转)和pitch(俯仰)
static inline void ahrs_float_get_quat_from_accel(struct FloatQuat* q, struct Int32Vect3* accel) {
  /* normalized accel measurement in floating point */
  //使加速度计测量值变为浮点型的
  struct FloatVect3 acc_normalized;
  ACCELS_FLOAT_OF_BFP(acc_normalized, *accel);
  FLOAT_VECT3_NORMALIZE(acc_normalized);//标准化该浮点型向量

  /* check for 180deg case */
  // 180度检测
  if ( ABS(acc_normalized.z - 1.0) < 5*FLT_MIN ) {
    QUAT_ASSIGN(*q, 0.0, 1.0, 0.0, 0.0);//z值小于一定数时
  }
  else {
    /*
     * axis we want to rotate around is cross product of accel and reference [0,0,-g]
     * normalized: cross(acc_normalized, [0,0,-1])
     * vector part of quaternion is the axis
     * scalar part (angle): 1.0 + dot(acc_normalized, [0,0,-1])
     * 我们想要旋转的轴是加速度计值和参考值[0,0,-g]的叉乘
     * 标准化： cross(acc_normalized, [0,0,-1])
     * 标量部分（角度）：1.0+dot(acc_normalized, [0,0,-1])

     */
    q->qx = - acc_normalized.y;
    q->qy = acc_normalized.x;
    q->qz = 0.0;
    q->qi = 1.0 - acc_normalized.z;
    FLOAT_QUAT_NORMALIZE(*q);
  }
}

static inline void ahrs_float_get_quat_from_accel_mag(struct FloatQuat* q, struct Int32Vect3* accel, struct Int32Vect3* mag) {

  /* the quaternion representing roll and pitch from acc measurement */
  //根据加速度计的测量，四元数代表roll和pitch(x,y)
  struct FloatQuat q_a;
  ahrs_float_get_quat_from_accel(&q_a, accel);


  /* convert mag measurement to float */
  //转换磁力计的测量值为浮点型
  struct FloatVect3 mag_float;
  MAGS_FLOAT_OF_BFP(mag_float, *mag);

  /* and rotate to horizontal plane using the quat from above */
  //使用上面的q_a四元数将其旋转到水平面
  struct FloatRMat rmat_phi_theta;
  FLOAT_RMAT_OF_QUAT(rmat_phi_theta, q_a);//将上面由加速度计得到的四元数转换为矩阵
  struct FloatVect3 mag_ltp;
  FLOAT_RMAT_VECT3_TRANSP_MUL(mag_ltp, rmat_phi_theta, mag_float);//将磁力计与加速度计的数据融合（矩阵乘）

  /* heading from mag -> make quaternion to rotate around ltp z axis*/
  //使四元数绕z轴旋转(根据mag来heading)
  struct FloatQuat q_m;

  /* dot([mag_n.x, mag_n.x, 0], [AHRS_H_X, AHRS_H_Y, 0]) */
  //只是取x的值，AHRS_H_Y=0??
  float dot = mag_ltp.x * AHRS_H_X + mag_ltp.y * AHRS_H_Y;

  /* |v1||v2| */
  // 计算向量模的乘积
  float norm2 = sqrtf(SQUARE(mag_ltp.x) + SQUARE(mag_ltp.y))
    * sqrtf(SQUARE(AHRS_H_X) + SQUARE(AHRS_H_Y));

  // catch 180deg case 180度检测
  if (ABS(norm2 + dot) < 5*FLT_MIN) {
    QUAT_ASSIGN(q_m, 0.0, 0.0, 0.0, 1.0);
  } else {
    /* q_xyz = cross([mag_n.x, mag_n.y, 0], [AHRS_H_X, AHRS_H_Y, 0]) */
    q_m.qx = 0.0;
    q_m.qy = 0.0;
    q_m.qz = mag_ltp.x * AHRS_H_Y - mag_ltp.y * AHRS_H_X;
    q_m.qi = norm2 + dot;
    FLOAT_QUAT_NORMALIZE(q_m);
  }

  // q_ltp2imu = q_a * q_m
  // and wrap and normalize
  FLOAT_QUAT_COMP_NORM_SHORTEST(*q, q_m, q_a);
}

#endif /* AHRS_FLOAT_UTILS_H */
