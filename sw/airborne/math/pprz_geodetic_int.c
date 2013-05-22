/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include "pprz_geodetic_int.h"
#include "pprz_algebra_int.h"

#define HIGH_RES_TRIG_FRAC  20


/*ecef坐标系到ltp坐标系的转换（int型精度）*/

void ltp_def_from_ecef_i(struct LtpDef_i* def, struct EcefCoor_i* ecef) {

  /* store the origin of the tangeant plane */
  /* 保存原始的切平面数据*/
  VECT3_COPY(def->ecef, *ecef);
  /* compute the lla representation of the origin */
  /* 计算LLA原始点的位置信息（lon:rad*1e7,lat:rad*1e7,alt：mm）*/
  lla_of_ecef_i(&def->lla, &def->ecef);
  /* store the rotation matrix                    */
  /* 存储旋转矩阵值*/

#if 1
  //计算经度和纬度的正弦和余弦的值（先转换成float型，再求sin和cos ,最后*<<20,求得整数值）
  int32_t sin_lat = rint(BFP_OF_REAL(sinf(RAD_OF_EM7RAD((float)def->lla.lat)), HIGH_RES_TRIG_FRAC));
  int32_t cos_lat = rint(BFP_OF_REAL(cosf(RAD_OF_EM7RAD((float)def->lla.lat)), HIGH_RES_TRIG_FRAC));
  int32_t sin_lon = rint(BFP_OF_REAL(sinf(RAD_OF_EM7RAD((float)def->lla.lon)), HIGH_RES_TRIG_FRAC));
  int32_t cos_lon = rint(BFP_OF_REAL(cosf(RAD_OF_EM7RAD((float)def->lla.lon)), HIGH_RES_TRIG_FRAC));
#else
  int32_t sin_lat = rint(BFP_OF_REAL(sin(RAD_OF_EM7RAD((double)def->lla.lat)), HIGH_RES_TRIG_FRAC));
  int32_t cos_lat = rint(BFP_OF_REAL(cos(RAD_OF_EM7RAD((double)def->lla.lat)), HIGH_RES_TRIG_FRAC));
  int32_t sin_lon = rint(BFP_OF_REAL(sin(RAD_OF_EM7RAD((double)def->lla.lon)), HIGH_RES_TRIG_FRAC));
  int32_t cos_lon = rint(BFP_OF_REAL(cos(RAD_OF_EM7RAD((double)def->lla.lon)), HIGH_RES_TRIG_FRAC));
#endif

/* 以下几位ECEF到lla坐标系的转换矩阵*/
  def->ltp_of_ecef.m[0] = -sin_lon;
  def->ltp_of_ecef.m[1] =  cos_lon;
  def->ltp_of_ecef.m[2] =  0; /* this element is always zero http://en.wikipedia.org/wiki/Geodetic_system#From_ECEF_to_ENU */
  def->ltp_of_ecef.m[3] = (int32_t)((-(int64_t)sin_lat*(int64_t)cos_lon)>>HIGH_RES_TRIG_FRAC);
  def->ltp_of_ecef.m[4] = (int32_t)((-(int64_t)sin_lat*(int64_t)sin_lon)>>HIGH_RES_TRIG_FRAC);
  def->ltp_of_ecef.m[5] =  cos_lat;
  def->ltp_of_ecef.m[6] = (int32_t)(( (int64_t)cos_lat*(int64_t)cos_lon)>>HIGH_RES_TRIG_FRAC);
  def->ltp_of_ecef.m[7] = (int32_t)(( (int64_t)cos_lat*(int64_t)sin_lon)>>HIGH_RES_TRIG_FRAC);
  def->ltp_of_ecef.m[8] =  sin_lat;

}

void ltp_def_from_lla_i(struct LtpDef_i* def, struct LlaCoor_i* lla) {

  /* store the origin of the tangeant plane */
  // 存储切平面的原始信息（lla）
  LLA_COPY(def->lla, *lla);
  /* compute the ecef representation of the origin */
  //由lla计算ecef的信息
  ecef_of_lla_i(&def->ecef, &def->lla);
  /* store the rotation matrix                    */

#if 1
  int32_t sin_lat = rint(BFP_OF_REAL(sinf(RAD_OF_EM7RAD((float)def->lla.lat)), HIGH_RES_TRIG_FRAC));
  int32_t cos_lat = rint(BFP_OF_REAL(cosf(RAD_OF_EM7RAD((float)def->lla.lat)), HIGH_RES_TRIG_FRAC));
  int32_t sin_lon = rint(BFP_OF_REAL(sinf(RAD_OF_EM7RAD((float)def->lla.lon)), HIGH_RES_TRIG_FRAC));
  int32_t cos_lon = rint(BFP_OF_REAL(cosf(RAD_OF_EM7RAD((float)def->lla.lon)), HIGH_RES_TRIG_FRAC));
#else
  int32_t sin_lat = rint(BFP_OF_REAL(sin(RAD_OF_EM7RAD((double)def->lla.lat)), HIGH_RES_TRIG_FRAC));
  int32_t cos_lat = rint(BFP_OF_REAL(cos(RAD_OF_EM7RAD((double)def->lla.lat)), HIGH_RES_TRIG_FRAC));
  int32_t sin_lon = rint(BFP_OF_REAL(sin(RAD_OF_EM7RAD((double)def->lla.lon)), HIGH_RES_TRIG_FRAC));
  int32_t cos_lon = rint(BFP_OF_REAL(cos(RAD_OF_EM7RAD((double)def->lla.lon)), HIGH_RES_TRIG_FRAC));
#endif

/* 以下几位ECEF到lla坐标系的转换矩阵*/

  def->ltp_of_ecef.m[0] = -sin_lon;
  def->ltp_of_ecef.m[1] =  cos_lon;
  def->ltp_of_ecef.m[2] =  0; /* this element is always zero http://en.wikipedia.org/wiki/Geodetic_system#From_ECEF_to_ENU */
  def->ltp_of_ecef.m[3] = (int32_t)((-(int64_t)sin_lat*(int64_t)cos_lon)>>HIGH_RES_TRIG_FRAC);
  def->ltp_of_ecef.m[4] = (int32_t)((-(int64_t)sin_lat*(int64_t)sin_lon)>>HIGH_RES_TRIG_FRAC);
  def->ltp_of_ecef.m[5] =  cos_lat;
  def->ltp_of_ecef.m[6] = (int32_t)(( (int64_t)cos_lat*(int64_t)cos_lon)>>HIGH_RES_TRIG_FRAC);
  def->ltp_of_ecef.m[7] = (int32_t)(( (int64_t)cos_lat*(int64_t)sin_lon)>>HIGH_RES_TRIG_FRAC);
  def->ltp_of_ecef.m[8] =  sin_lat;

}

void enu_of_ecef_point_i(struct EnuCoor_i* enu, struct LtpDef_i* def, struct EcefCoor_i* ecef) {

  struct EcefCoor_i delta;
  //*ecef代表pd（待测点）,def->ecef代表p0(选定的测站点)，向量差给delat
  VECT3_DIFF(delta, *ecef, def->ecef);
  //由ecef坐标系转换到enu坐标系，其所乘为 旋转矩阵
  const int64_t tmpx = (int64_t)def->ltp_of_ecef.m[0]*delta.x +
                       (int64_t)def->ltp_of_ecef.m[1]*delta.y +
                       0; /* this element is always zero http://en.wikipedia.org/wiki/Geodetic_system#From_ECEF_to_ENU */
  enu->x = (int32_t)(tmpx>>HIGH_RES_TRIG_FRAC);
  const int64_t tmpy = (int64_t)def->ltp_of_ecef.m[3]*delta.x +
                       (int64_t)def->ltp_of_ecef.m[4]*delta.y +
                       (int64_t)def->ltp_of_ecef.m[5]*delta.z;
  enu->y = (int32_t)(tmpy>>HIGH_RES_TRIG_FRAC);
  const int64_t tmpz = (int64_t)def->ltp_of_ecef.m[6]*delta.x +
                       (int64_t)def->ltp_of_ecef.m[7]*delta.y +
                       (int64_t)def->ltp_of_ecef.m[8]*delta.z;
  enu->z = (int32_t)(tmpz>>HIGH_RES_TRIG_FRAC);

}


void ned_of_ecef_point_i(struct NedCoor_i* ned, struct LtpDef_i* def, struct EcefCoor_i* ecef) {

  struct EnuCoor_i enu;
  enu_of_ecef_point_i(&enu, def, ecef);//先得到enu坐标信息

  ENU_OF_TO_NED(*ned, enu);//enu坐标转换到ned坐标，z轴取反即可

}

void enu_of_ecef_vect_i(struct EnuCoor_i* enu, struct LtpDef_i* def, struct EcefCoor_i* ecef) {
//由旋转矩阵乘pd（待测点）得到enu向量值
  const int64_t tmpx = (int64_t)def->ltp_of_ecef.m[0]*ecef->x +
                       (int64_t)def->ltp_of_ecef.m[1]*ecef->y +
                       0; /* this element is always zero http://en.wikipedia.org/wiki/Geodetic_system#From_ECEF_to_ENU */
  enu->x = (int32_t)(tmpx>>HIGH_RES_TRIG_FRAC);
  const int64_t tmpy = (int64_t)def->ltp_of_ecef.m[3]*ecef->x +
                       (int64_t)def->ltp_of_ecef.m[4]*ecef->y +
                       (int64_t)def->ltp_of_ecef.m[5]*ecef->z;
  enu->y = (int32_t)(tmpy>>HIGH_RES_TRIG_FRAC);
  const int64_t tmpz = (int64_t)def->ltp_of_ecef.m[6]*ecef->x +
                       (int64_t)def->ltp_of_ecef.m[7]*ecef->y +
                       (int64_t)def->ltp_of_ecef.m[8]*ecef->z;
  enu->z = (int32_t)(tmpz>>HIGH_RES_TRIG_FRAC);

}


void ned_of_ecef_vect_i(struct NedCoor_i* ned, struct LtpDef_i* def, struct EcefCoor_i* ecef) {
  struct EnuCoor_i enu;
  enu_of_ecef_vect_i(&enu, def, ecef);
  ENU_OF_TO_NED(*ned, enu);//取反得到NED的向量坐标
}

/* check if resolution of INT32_TRIG_FRAC (14) is enough here */
void ecef_of_enu_point_i(struct EcefCoor_i* ecef, struct LtpDef_i* def, struct EnuCoor_i* enu) {
  INT32_RMAT_TRANSP_VMULT(*ecef, def->ltp_of_ecef, *enu);//将enu坐标系转换成ecef坐标系,乘的是转置矩阵
  INT32_VECT3_ADD(*ecef, def->ecef);//将def里面存储的ecef加入到由enu转换后的ecef
}

void ecef_of_ned_point_i(struct EcefCoor_i* ecef, struct LtpDef_i* def, struct NedCoor_i* ned) {
  struct EnuCoor_i enu;
  ENU_OF_TO_NED(enu, *ned);//先将ned转换城enu
  ecef_of_enu_point_i(ecef, def, &enu);//将enu转换为ecef
}

void ecef_of_enu_vect_i(struct EcefCoor_i* ecef, struct LtpDef_i* def, struct EnuCoor_i* enu) {
  INT32_RMAT_TRANSP_VMULT(*ecef, def->ltp_of_ecef, *enu);//得到ecef的向量坐标
}

void ecef_of_ned_vect_i(struct EcefCoor_i* ecef, struct LtpDef_i* def, struct NedCoor_i* ned) {
  struct EnuCoor_i enu;
  ENU_OF_TO_NED(enu, *ned);//先将ned转换城enu
  ecef_of_enu_vect_i(ecef, def, &enu);////将enu转换为ecef
}


void enu_of_lla_point_i(struct EnuCoor_i* enu, struct LtpDef_i* def, struct LlaCoor_i* lla) {
  struct EcefCoor_i ecef;
  ecef_of_lla_i(&ecef,lla);//先将lla坐标系转换成ecef坐标系
  enu_of_ecef_point_i(enu,def,&ecef);//再由ecef坐标系转换成enu坐标系（点）
}

void ned_of_lla_point_i(struct NedCoor_i* ned, struct LtpDef_i* def, struct LlaCoor_i* lla) {
  struct EcefCoor_i ecef;
  ecef_of_lla_i(&ecef,lla);
  ned_of_ecef_point_i(ned,def,&ecef);//由ecef坐标系转换成ned坐标系（点）
}

void enu_of_lla_vect_i(struct EnuCoor_i* enu, struct LtpDef_i* def, struct LlaCoor_i* lla) {
  struct EcefCoor_i ecef;
  ecef_of_lla_i(&ecef,lla);
  enu_of_ecef_vect_i(enu,def,&ecef);//由ecef坐标系转换成enu坐标系（向量）
}

void ned_of_lla_vect_i(struct NedCoor_i* ned, struct LtpDef_i* def, struct LlaCoor_i* lla) {
  struct EcefCoor_i ecef;
  ecef_of_lla_i(&ecef,lla);
  ned_of_ecef_vect_i(ned,def,&ecef);////由ecef坐标系转换成ned坐标系（向量）
}

/*
   For now we cheat and call the floating point version
   Anyone up for writing it in fixed point ?
*/
#include "pprz_geodetic_float.h"
#include "pprz_geodetic_double.h"

void lla_of_ecef_i(struct LlaCoor_i* out, struct EcefCoor_i* in) {

  /* convert our input to floating point */
  /* 将输入转换成双精度浮点型 cm—>m的转换*/
  struct EcefCoor_d in_d;
  in_d.x = M_OF_CM((double)in->x);
  in_d.y = M_OF_CM((double)in->y);
  in_d.z = M_OF_CM((double)in->z);
  /* calls the floating point transformation */
  /* 调用浮点转换*/
  struct LlaCoor_d out_d;
  lla_of_ecef_d(&out_d, &in_d);//得到经度，纬度，高度的值
  /* convert the output to fixed point       */
  /* 将输出转换成定点型（整型）*/
  out->lon = (int32_t)rint(EM7RAD_OF_RAD(out_d.lon));
  out->lat = (int32_t)rint(EM7RAD_OF_RAD(out_d.lat));
  out->alt = (int32_t)MM_OF_M(out_d.alt);

}

void ecef_of_lla_i(struct EcefCoor_i* out, struct LlaCoor_i* in) {

  /* convert our input to floating point */
  //将输入的整型数转换成单精度型数
  struct LlaCoor_d in_d;
  in_d.lon = RAD_OF_EM7RAD((double)in->lon);
  in_d.lat = RAD_OF_EM7RAD((double)in->lat);
  in_d.alt = M_OF_MM((double)in->alt);
  /* calls the floating point transformation */
  // 调用单精度浮点型的转换,得到ecef坐标信息
  struct EcefCoor_d out_d;
  ecef_of_lla_d(&out_d, &in_d);
  /* convert the output to fixed point       */
  //将输出转换成整型数据
  out->x = (int32_t)CM_OF_M(out_d.x);
  out->y = (int32_t)CM_OF_M(out_d.y);
  out->z = (int32_t)CM_OF_M(out_d.z);

}
