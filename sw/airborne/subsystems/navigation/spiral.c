/*
 * Copyright (C) 2011  The Paparazzi Team
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
 * @file subsystems/navigation/spiral.c
 *
 * Fixedwing navigation in a spiral/helix from Uni Stuttgart.
 * 固定翼螺旋航线
 * creating a helix:    创建一条螺旋线
 * - start radius to end radius, increasing after reaching alphamax   开始半径到结束半径，增长直到达最大的到阿尔法角
 * - Alphamax is calculated from given segments   最大的阿尔法角是由给定的段数计算的
 * - IMPORTANT: numer of segments has to be larger than 2!   注意：段数必须大于2
 */

#include "subsystems/navigation/spiral.h"

#include "subsystems/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

#ifdef DIGITAL_CAM
#include "modules/digital_cam/dc.h"
#endif

enum SpiralStatus { Outside, StartCircle, Circle, IncSpiral };
static enum SpiralStatus CSpiralStatus;
// static float SpiralTheta;
// static float Fly2X;
// static float Fly2Y;

static float FlyFromX;
static float FlyFromY;
static float TransCurrentX;
static float TransCurrentY;
static float TransCurrentZ;
static float EdgeCurrentX;
static float EdgeCurrentY;
static float LastCircleX;
static float LastCircleY;
static float DistanceFromCenter;
static float Spiralradius;
static uint8_t Center;
static uint8_t Edge;
static float SRad;
static float IRad;
static float Alphalimit;
static float Segmente;
static float ZPoint;
static float nav_radius_min;

#ifndef MIN_CIRCLE_RADIUS
#define MIN_CIRCLE_RADIUS 120
#endif


bool_t InitializeSpiral(uint8_t CenterWP, uint8_t EdgeWP, float StartRad, float IncRad, float Segments, float ZKoord)
{
  Center = CenterWP;    // center of the helix         螺旋线的中心
  Edge = EdgeWP;        // edge point on the maximaum radius      最大半径的边缘点
  SRad = StartRad;	// start radius of the helix         螺旋线的开始半径
  Segmente = Segments;
  ZPoint = ZKoord;
  nav_radius_min = MIN_CIRCLE_RADIUS;
  if (SRad < nav_radius_min) SRad = nav_radius_min;
  IRad = IncRad;		// multiplier for increasing the spiral    螺旋线增长的乘数

  EdgeCurrentX = WaypointX(Edge) - WaypointX(Center);
  EdgeCurrentY = WaypointY(Edge) - WaypointY(Center);

  Spiralradius = sqrt(EdgeCurrentX*EdgeCurrentX+EdgeCurrentY*EdgeCurrentY);   //螺旋线的半径

  TransCurrentX = stateGetPositionEnu_f()->x - WaypointX(Center);
  TransCurrentY = stateGetPositionEnu_f()->y - WaypointY(Center);
  TransCurrentZ = stateGetPositionEnu_f()->z - ZPoint;
  DistanceFromCenter = sqrt(TransCurrentX*TransCurrentX+TransCurrentY*TransCurrentY);

  //    SpiralTheta = atan2(TransCurrentY,TransCurrentX);
  //    Fly2X = Spiralradius*cos(SpiralTheta+M_PI)+WaypointX(Center);
  //    Fly2Y = Spiralradius*sin(SpiralTheta+M_PI)+WaypointY(Center);

  // Alphalimit denotes angle, where the radius will be increased  
   //有阿尔法角，半径就增长
  Alphalimit = 2*M_PI / Segments;   //阿尔法角由段数计算而得
  //current position 当前位置
  FlyFromX = stateGetPositionEnu_f()->x;
  FlyFromY = stateGetPositionEnu_f()->y;

  if(DistanceFromCenter > Spiralradius)
    CSpiralStatus = Outside;
  return FALSE;
}

bool_t SpiralNav(void)
{
  TransCurrentX = stateGetPositionEnu_f()->x - WaypointX(Center);
  TransCurrentY = stateGetPositionEnu_f()->y - WaypointY(Center);
  DistanceFromCenter = sqrt(TransCurrentX*TransCurrentX+TransCurrentY*TransCurrentY);

  float DistanceStartEstim;
  float CircleAlpha;

  switch(CSpiralStatus)
  {
  case Outside:
    //flys until center of the helix is reached an start helix
      //螺旋线的中心到达起始螺旋线时飞
    nav_route_xy(FlyFromX,FlyFromY,WaypointX(Center), WaypointY(Center));
    // center reached?   判断中心是否到达
    if (nav_approaching_xy(WaypointX(Center), WaypointY(Center), FlyFromX, FlyFromY, 0)) {
      // nadir image   图像最低点
#ifdef DIGITAL_CAM
      dc_send_command(DC_SHOOT);
#endif
      CSpiralStatus = StartCircle;
    }
    break;
  case StartCircle:
    // Starts helix   开始螺旋线
    // storage of current coordinates   存储当前坐标
    // calculation needed, State switch to Circle   计算需要的，声明switch
    nav_circle_XY(WaypointX(Center), WaypointY(Center), SRad);
    if(DistanceFromCenter >= SRad){
      LastCircleX = stateGetPositionEnu_f()->x;
      LastCircleY = stateGetPositionEnu_f()->y;
      CSpiralStatus = Circle;
      // Start helix   开始螺旋
#ifdef DIGITAL_CAM
      dc_Circle(360/Segmente);
#endif
    }
    break;
  case Circle: {
    nav_circle_XY(WaypointX(Center), WaypointY(Center), SRad);
    // Trigonometrische Berechnung des bereits geflogenen Winkels alpha  德语！！！！
    // equation:
    // alpha = 2 * asin ( |Starting position angular - current positon| / (2* SRad)
    // if alphamax already reached, increase radius.   如果已经到达阿尔法的最大值，增加半径

    //DistanceStartEstim = |Starting position angular - current positon|
    DistanceStartEstim = sqrt (((LastCircleX-stateGetPositionEnu_f()->x)*(LastCircleX-stateGetPositionEnu_f()->x))
                               + ((LastCircleY-stateGetPositionEnu_f()->y)*(LastCircleY-stateGetPositionEnu_f()->y)));
    CircleAlpha = (2.0 * asin (DistanceStartEstim / (2 * SRad)));
    if (CircleAlpha >= Alphalimit) {
      LastCircleX = stateGetPositionEnu_f()->x;
      LastCircleY = stateGetPositionEnu_f()->y;
      CSpiralStatus = IncSpiral;
    }
    break;
  }
  case IncSpiral:
    // increasing circle radius as long as it is smaller than max helix radius   只要圆的半径比螺旋半径小，增加圆的半径
    if(SRad + IRad < Spiralradius)
    {
      SRad = SRad + IRad;
#ifdef DIGITAL_CAM
      if (dc_cam_tracing) {
        // calculating Cam angle for camera alignment   为相机校准计算凸轮角
        TransCurrentZ = stateGetPositionEnu_f()->z - ZPoint;
        dc_cam_angle = atan(SRad/TransCurrentZ) * 180  / M_PI;
      }
#endif
    }
    else {
      SRad = Spiralradius;
#ifdef DIGITAL_CAM
      // Stopps DC
      dc_stop();
#endif
    }
    CSpiralStatus = Circle;
    break;
  default:
    break;
  }
  return TRUE;
}
