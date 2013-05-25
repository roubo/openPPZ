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
 * @file subsystems/navigation/OSAMNav.c
 * 
 */

#include "subsystems/navigation/OSAMNav.h"

#include "subsystems/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

/************** Flower Navigation **********************************************/
                /*花形航线*/

/** Makes a flower pattern.
	CenterWP is the center of the flower. The Navigation Height is taken from this waypoint.
	EdgeWP defines the radius of the flower (distance from CenterWP to EdgeWP)
*/
//制作花的模型，中心航点是花的中心。航行的高度从这个航点获得。边缘航点决定花的半径（即边缘航点与中心航点的距离）

enum FlowerStatus { Outside, FlowerLine, Circle };
static enum FlowerStatus CFlowerStatus;
static float CircleX;
static float CircleY;
static float Fly2X;
static float Fly2Y;
static float FlyFromX;
static float FlyFromY;
static float TransCurrentX;
static float TransCurrentY;
static float EdgeCurrentX;
static float EdgeCurrentY;
static float DistanceFromCenter;
static float FlowerTheta;
static float Flowerradius;
static uint8_t Center;
static uint8_t Edge;

#ifndef LINE_START_FUNCTION
#define LINE_START_FUNCTION {}
#endif
#ifndef LINE_STOP_FUNCTION
#define LINE_STOP_FUNCTION {}
#endif

bool_t InitializeFlower(uint8_t CenterWP, uint8_t EdgeWP)            //初始化
{
  Center = CenterWP;
  Edge = EdgeWP;

  EdgeCurrentX = WaypointX(Edge) - WaypointX(Center);
  EdgeCurrentY = WaypointY(Edge) - WaypointY(Center);

  Flowerradius = sqrt(EdgeCurrentX*EdgeCurrentX+EdgeCurrentY*EdgeCurrentY);    //计算半径

  TransCurrentX = stateGetPositionEnu_f()->x - WaypointX(Center);
  TransCurrentY = stateGetPositionEnu_f()->y - WaypointY(Center);
  DistanceFromCenter = sqrt(TransCurrentX*TransCurrentX+TransCurrentY*TransCurrentY);   //当前点与中心点的距离

  FlowerTheta = atan2(TransCurrentY,TransCurrentX);
  Fly2X = Flowerradius*cos(FlowerTheta+3.14)+WaypointX(Center);
  Fly2Y = Flowerradius*sin(FlowerTheta+3.14)+WaypointY(Center);
  FlyFromX = stateGetPositionEnu_f()->x;       //当前点的x,y坐标
  FlyFromY = stateGetPositionEnu_f()->y;

  if(DistanceFromCenter > Flowerradius)     //判断是否出花的边界了
    CFlowerStatus = Outside;
  else
    CFlowerStatus = FlowerLine;

  CircleX = 0;
  CircleY = 0;
  return FALSE;
}

bool_t FlowerNav(void)
{
  TransCurrentX = stateGetPositionEnu_f()->x - WaypointX(Center);
  TransCurrentY = stateGetPositionEnu_f()->y - WaypointY(Center);
  DistanceFromCenter = sqrt(TransCurrentX*TransCurrentX+TransCurrentY*TransCurrentY);   //当前点与花的中心距离

  bool_t InCircle = TRUE;
  float CircleTheta;

  if(DistanceFromCenter > Flowerradius)  // 不在花型之内
    InCircle = FALSE;

  NavVerticalAutoThrottleMode(0); /* No pitch */   //自动油门模式
  NavVerticalAltitudeMode(waypoints[Center].a, 0.);   //高度模式

  switch(CFlowerStatus)
  {
  case Outside:
    nav_route_xy(FlyFromX,FlyFromY,Fly2X,Fly2Y);
    if(InCircle)       //Outside怎么还能Incircle?矛盾啦！！！！
    {
      CFlowerStatus = FlowerLine;
      FlowerTheta = atan2(TransCurrentY,TransCurrentX);
      Fly2X = Flowerradius*cos(FlowerTheta+3.14)+WaypointX(Center);
      Fly2Y = Flowerradius*sin(FlowerTheta+3.14)+WaypointY(Center);
      FlyFromX = stateGetPositionEnu_f()->x;
      FlyFromY = stateGetPositionEnu_f()->y;
      nav_init_stage();
    }
    break;
  case FlowerLine:
    nav_route_xy(FlyFromX,FlyFromY,Fly2X,Fly2Y);
    if(!InCircle)
    {
      CFlowerStatus = Circle;
      CircleTheta = nav_radius/Flowerradius;
      CircleX = Flowerradius*cos(FlowerTheta+3.14-CircleTheta)+WaypointX(Center);
      CircleY = Flowerradius*sin(FlowerTheta+3.14-CircleTheta)+WaypointY(Center);
      nav_init_stage();
    }
    break;
  case Circle:
    nav_circle_XY(CircleX, CircleY, nav_radius);
    if(InCircle)
    {
      EdgeCurrentX = WaypointX(Edge) - WaypointX(Center);
      EdgeCurrentY = WaypointY(Edge) - WaypointY(Center);
      Flowerradius = sqrt(EdgeCurrentX*EdgeCurrentX+EdgeCurrentY*EdgeCurrentY);
      if(DistanceFromCenter > Flowerradius)
        CFlowerStatus = Outside;
      else
        CFlowerStatus = FlowerLine;
      FlowerTheta = atan2(TransCurrentY,TransCurrentX);
      Fly2X = Flowerradius*cos(FlowerTheta+3.14)+WaypointX(Center);
      Fly2Y = Flowerradius*sin(FlowerTheta+3.14)+WaypointY(Center);
      FlyFromX = stateGetPositionEnu_f()->x;
      FlyFromY = stateGetPositionEnu_f()->y;
      nav_init_stage();
    }
    break;

  default:
    break;
  }
  return TRUE;
}

/************** Bungee Takeoff **********************************************/
                /* 弹力起飞*/
/** Takeoff functions for bungee takeoff.
    Run initialize function when the plane is on the bungee, the bungee is fully extended and you are ready to
    launch the plane. After initialized, the plane will follow a line drawn by the position of the plane on initialization and the
    position of the bungee (given in the arguments). Once the plane crosses the throttle line, which is perpendicular to the line the plane is following,
    and intersects the position of the bungee (plus or minus a fixed distance (TakeOff_Distance in airframe file) from the bungee just in case the bungee doesn't release directly above the bungee) the prop will come on. The plane will then continue to follow the line until it has reached a specific
    height (defined in as Takeoff_Height in airframe file) above the bungee waypoint and speed (defined as Takeoff_Speed in the airframe file).
    @verbatim
    <section name="Takeoff" prefix="Takeoff_">
    <define name="Height" value="30" unit="m"/>
    <define name="Speed" value="15" unit="m/s"/>
    <define name="Distance" value="10" unit="m"/>
    <define name="MinSpeed" value="5" unit="m/s"/>
    </section>
    @endverbatim
*/

/*弹力起飞函数
  当飞机在弹力绳上准备时，初始化起跑函数，弹力绳尽可能的被拉长，并且要准备好发射飞机。初始化之后，飞机将沿着一条在初始化时已经画好的线飞，一旦飞机越过了起跑线，该起跑线与飞机飞行路线垂直，且与弹力绳的位置相交（从弹力绳位置处加或减一个固定的距离以防弹力绳没有完全的松弛）。飞机讲继续沿着这条线飞直到达到弹力航点之上的一定高度和速度为止。（高度，速度和上文提到的固定的距离在airframe文件里定义）*/

#ifndef Takeoff_Distance
#define Takeoff_Distance 10
#endif
#ifndef Takeoff_Height
#define Takeoff_Height 30
#endif
#ifndef Takeoff_Speed
#define Takeoff_Speed 15
#endif
#ifndef Takeoff_MinSpeed
#define Takeoff_MinSpeed 5
#endif

enum TakeoffStatus { Launch, Throttle, Finished };
static enum TakeoffStatus CTakeoffStatus;
static float throttlePx;
static float throttlePy;
static float initialx;
static float initialy;
static float ThrottleSlope;
static bool_t AboveLine;
static float BungeeAlt;
static float TDistance;
static uint8_t BungeeWaypoint;

bool_t InitializeBungeeTakeoff(uint8_t BungeeWP)    //初始化弹力起飞函数
{
  float ThrottleB;

  initialx = stateGetPositionEnu_f()->x;
  initialy = stateGetPositionEnu_f()->y;

  BungeeWaypoint = BungeeWP;

  //Takeoff_Distance can only be positive   起飞距离只能是正数
  TDistance = fabs(Takeoff_Distance);

  //Translate initial position so that the position of the bungee is (0,0)    转化初始化的位置，以使弹力绳的位置是（0，0）
  float Currentx = initialx-(WaypointX(BungeeWaypoint));
  float Currenty = initialy-(WaypointY(BungeeWaypoint));

  //Record bungee alt (which should be the ground alt at that point)    记录弹力绳的高度（这个点的高度可以是该点的地面高度）
  BungeeAlt = waypoints[BungeeWaypoint].a;

  //Find Launch line slope and Throttle line slope   找发射线的斜度和制动线（终点线）的斜度
  float MLaunch = Currenty/Currentx;

  //Find Throttle Point (the point where the throttle line and launch line intersect)   //找到制动线（这个点在制动线和发射线的交点）
  if(Currentx < 0)
    throttlePx = TDistance/sqrt(MLaunch*MLaunch+1);
  else
    throttlePx = -(TDistance/sqrt(MLaunch*MLaunch+1));

  if(Currenty < 0)
    throttlePy = sqrt((TDistance*TDistance)-(throttlePx*throttlePx));
  else
    throttlePy = -sqrt((TDistance*TDistance)-(throttlePx*throttlePx));

  //Find ThrottleLine   找到制动线
  ThrottleSlope = tan(atan2(Currenty,Currentx)+(3.14/2));
  ThrottleB = (throttlePy - (ThrottleSlope*throttlePx));

  //Determine whether the UAV is below or above the throttle line   决定无人机在制动线的上方还是下方
  if(Currenty > ((ThrottleSlope*Currentx)+ThrottleB))
    AboveLine = TRUE;
  else
    AboveLine = FALSE;

  //Enable Launch Status and turn kill throttle on   使能发射，关闭油门
  CTakeoffStatus = Launch;
  kill_throttle = 1;

  //Translate the throttle point back   转换油门
  throttlePx = throttlePx+(WaypointX(BungeeWP));
  throttlePy = throttlePy+(WaypointY(BungeeWP));

  return FALSE;
}

bool_t BungeeTakeoff(void)    //初始化弹力起飞
{
  //Translate current position so Throttle point is (0,0)
  //转化当前坐标使制动点坐标是（0，0）
  float Currentx = stateGetPositionEnu_f()->x-throttlePx;
  float Currenty = stateGetPositionEnu_f()->y-throttlePy;
  bool_t CurrentAboveLine;
  float ThrottleB;

  switch(CTakeoffStatus)
  {
  case Launch:   //起飞标志
    //Follow Launch Line   沿着发射线
    NavVerticalAutoThrottleMode(0);
    NavVerticalAltitudeMode(BungeeAlt+Takeoff_Height, 0.);
    nav_route_xy(initialx,initialy,throttlePx,throttlePy);

    kill_throttle = 1;     //关闭油门

    //recalculate lines if below min speed   如果速度比最低速度小，重新计算线
    if((*stateGetHorizontalSpeedNorm_f()) < Takeoff_MinSpeed)
    {
      initialx = stateGetPositionEnu_f()->x;
      initialy = stateGetPositionEnu_f()->y;

      //Translate initial position so that the position of the bungee is (0,0)
      //转换初始化坐标使弹力点坐标为（0,0）
      Currentx = initialx-(WaypointX(BungeeWaypoint));
      Currenty = initialy-(WaypointY(BungeeWaypoint));

      //Find Launch line slope   找到发射线的斜度
      float MLaunch = Currenty/Currentx;

      //Find Throttle Point (the point where the throttle line and launch line intersect)
       //找到制动点（制动点是制动线和起跑线的交点）
      if(Currentx < 0)
        throttlePx = TDistance/sqrt(MLaunch*MLaunch+1);
      else
        throttlePx = -(TDistance/sqrt(MLaunch*MLaunch+1));

      if(Currenty < 0)
        throttlePy = sqrt((TDistance*TDistance)-(throttlePx*throttlePx));
      else
        throttlePy = -sqrt((TDistance*TDistance)-(throttlePx*throttlePx));

      //Find ThrottleLine   找制动线
      ThrottleSlope = tan(atan2(Currenty,Currentx)+(3.14/2));
      ThrottleB = (throttlePy - (ThrottleSlope*throttlePx));

      //Determine whether the UAV is below or above the throttle line
    // 决定无人机在制动线的上方还是下方
      if(Currenty > ((ThrottleSlope*Currentx)+ThrottleB))
        AboveLine = TRUE;
      else
        AboveLine = FALSE;

      //Translate the throttle point back  转换油门
      throttlePx = throttlePx+(WaypointX(BungeeWaypoint));
      throttlePy = throttlePy+(WaypointY(BungeeWaypoint));
    }

    //Find out if the UAV is currently above the line 
    //找出无人机是否在线的上方
    if(Currenty > (ThrottleSlope*Currentx))
      CurrentAboveLine = TRUE;
    else
      CurrentAboveLine = FALSE;

    //Find out if UAV has crossed the line
     //找出无人机是否与线交叉
    if(AboveLine != CurrentAboveLine && (*stateGetHorizontalSpeedNorm_f()) > Takeoff_MinSpeed)
    {
      CTakeoffStatus = Throttle;
      kill_throttle = 0;
      nav_init_stage();
    }
    break;
  case Throttle:   //油门标志
    //Follow Launch Line   //沿着发射线
    NavVerticalAutoThrottleMode(AGR_CLIMB_PITCH);
    NavVerticalThrottleMode(9600*(1));
    nav_route_xy(initialx,initialy,throttlePx,throttlePy);
    kill_throttle = 0;

    if((stateGetPositionEnu_f()->z > BungeeAlt+Takeoff_Height-10) && ((*stateGetHorizontalSpeedNorm_f()) > Takeoff_Speed))
    {
      CTakeoffStatus = Finished;
      return FALSE;
    }
    else
    {
      return TRUE;
    }
    break;
  default:
    break;
  }
  return TRUE;
}

/************** Polygon Survey **********************************************/
              /*多变形测绘*/
/** This routine will cover the enitre area of any Polygon defined in the flightplan which is a convex polygon.
     这个程序将覆盖整个面积的任何在飞行计划中定义的凸多边形
 */

enum SurveyStatus { Init, Entry, Sweep, SweepCircle };
static enum SurveyStatus CSurveyStatus;
static struct Point2D SmallestCorner;
static struct Line Edges[PolygonSize];
static float EdgeMaxY[PolygonSize];
static float EdgeMinY[PolygonSize];
static float SurveyTheta;
static float dSweep;
static float SurveyRadius;
static struct Point2D SurveyToWP;
static struct Point2D SurveyFromWP;
static struct Point2D SurveyCircle;
static uint8_t SurveyEntryWP;
static uint8_t SurveySize;
static float SurveyCircleQdr;
static float MaxY;
uint16_t PolySurveySweepNum;
uint16_t PolySurveySweepBackNum;

bool_t InitializePolygonSurvey(uint8_t EntryWP, uint8_t Size, float sw, float Orientation)
{
  SmallestCorner.x = 0;
  SmallestCorner.y = 0;
  int i = 0;
  float ys = 0;
  static struct Point2D EntryPoint;
  float LeftYInt;
  float RightYInt;
  float temp;
  float XIntercept1 = 0;
  float XIntercept2 = 0;

  SurveyTheta = RadOfDeg(Orientation);
  PolySurveySweepNum = 0;
  PolySurveySweepBackNum = 0;

  SurveyEntryWP = EntryWP;
  SurveySize = Size;

  struct Point2D Corners[PolygonSize];

  CSurveyStatus = Init;

  if (Size == 0)
    return TRUE;

  //Don't initialize if Polygon is too big or if the orientation is not between 0 and 90
  //如果多边形太大或者如果方向不在-90和90之间，不要初始化
  if(Size <= PolygonSize && Orientation >= -90 && Orientation <= 90)
  {
    //Initialize Corners  初始化角
    for(i = 0; i < Size; i++)
    {
      Corners[i].x = waypoints[i+EntryWP].x;
      Corners[i].y = waypoints[i+EntryWP].y;
    }

    //Rotate Corners so sweeps are parellel with x axis旋转角以使整个区域和x轴平行（某个边与x轴平行）
    for(i = 0; i < Size; i++)
      TranslateAndRotateFromWorld(&Corners[i], SurveyTheta, 0, 0);

    //Find min x and min y       找到最小的x和y
    SmallestCorner.y = Corners[0].y;
    SmallestCorner.x = Corners[0].x;
    for(i = 1; i < Size; i++)
    {
      if(Corners[i].y < SmallestCorner.y)
        SmallestCorner.y = Corners[i].y;

      if(Corners[i].x < SmallestCorner.x)
        SmallestCorner.x = Corners[i].x;
    }

    //Translate Corners all exist in quad #1
     //转换角
    for(i = 0; i < Size; i++)
      TranslateAndRotateFromWorld(&Corners[i], 0, SmallestCorner.x, SmallestCorner.y);

    //Rotate and Translate Entry Point  旋转和转换进入点
    EntryPoint.x = Corners[0].x;
    EntryPoint.y = Corners[0].y;

    //Find max y   找到最大的y
    MaxY = Corners[0].y;
    for(i = 1; i < Size; i++)
    {
      if(Corners[i].y > MaxY)
        MaxY = Corners[i].y;
    }

    //Find polygon edges   找到多边形的边
    for(i = 0; i < Size; i++)
    {
      if(i == 0)
        if(Corners[Size-1].x == Corners[i].x) //Don't divide by zero!   防止下面的式子的分母为0
          Edges[i].m = MaxFloat;
        else
          Edges[i].m = ((Corners[Size-1].y-Corners[i].y)/(Corners[Size-1].x-Corners[i].x));
      else
        if(Corners[i].x == Corners[i-1].x)
          Edges[i].m = MaxFloat;
        else
          Edges[i].m = ((Corners[i].y-Corners[i-1].y)/(Corners[i].x-Corners[i-1].x));

      //Edges[i].m = MaxFloat;
      Edges[i].b = (Corners[i].y - (Corners[i].x*Edges[i].m));
    }

    //Find Min and Max y for each line
    //找到每条线的最小最大的y值
    FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[0], Edges[1]);
    FindInterceptOfTwoLines(&temp, &RightYInt, Edges[0], Edges[Size-1]);

    if(LeftYInt > RightYInt)  //给初值
    {
      EdgeMaxY[0] = LeftYInt;
      EdgeMinY[0] = RightYInt;
    }
    else
    {
      EdgeMaxY[0] = RightYInt;
      EdgeMinY[0] = LeftYInt;
    }

    for(i = 1; i < Size-1; i++)   //循环比较
    {
      FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[i], Edges[i+1]);
      FindInterceptOfTwoLines(&temp, &RightYInt, Edges[i], Edges[i-1]);

      if(LeftYInt > RightYInt)
      {
        EdgeMaxY[i] = LeftYInt;
        EdgeMinY[i] = RightYInt;
      }
      else
      {
        EdgeMaxY[i] = RightYInt;
        EdgeMinY[i] = LeftYInt;
      }
    }

    FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[Size-1], Edges[0]);
    FindInterceptOfTwoLines(&temp, &RightYInt, Edges[Size-1], Edges[Size-2]);

    if(LeftYInt > RightYInt)    //比较最后两个值
    {
      EdgeMaxY[Size-1] = LeftYInt;
      EdgeMinY[Size-1] = RightYInt;
    }
    else
    {
      EdgeMaxY[Size-1] = RightYInt;
      EdgeMinY[Size-1] = LeftYInt;
    }

    //Find amount to increment by every sweep  找到每次递增的量
    if(EntryPoint.y >= MaxY/2)
      dSweep = -sw;
    else
      dSweep = sw;

    //CircleQdr tells the plane when to exit the circle   Circleqdr告诉飞机什么时候退出弯
    if(dSweep >= 0)
      SurveyCircleQdr = -DegOfRad(SurveyTheta);
    else
      SurveyCircleQdr = 180-DegOfRad(SurveyTheta);

    //Find y value of the first sweep   找到第一次扫描时的y值
    ys = EntryPoint.y+(dSweep/2);

    //Find the edges which intercet the sweep line first   找到和第一次扫描的线相交的边缘
    for(i = 0; i < SurveySize; i++)
    {
      if(EdgeMinY[i] <= ys && EdgeMaxY[i] > ys)
      {
        XIntercept2 = XIntercept1;
        XIntercept1 = EvaluateLineForX(ys, Edges[i]);
      }
    }

    //Find point to come from and point to go to
    //找到起始点
    if(fabs(EntryPoint.x - XIntercept2) <= fabs(EntryPoint.x - XIntercept1))
    {
      SurveyToWP.x = XIntercept1;
      SurveyToWP.y = ys;

      SurveyFromWP.x = XIntercept2;
      SurveyFromWP.y = ys;
    }
    else
    {
      SurveyToWP.x = XIntercept2;
      SurveyToWP.y = ys;

      SurveyFromWP.x = XIntercept1;
      SurveyFromWP.y = ys;
    }

    //Find the direction to circle   找到圈的方向
    if(ys > 0 && SurveyToWP.x > SurveyFromWP.x)
      SurveyRadius = dSweep/2;
    else if(ys < 0 && SurveyToWP.x < SurveyFromWP.x)
      SurveyRadius = dSweep/2;
    else
      SurveyRadius = -dSweep/2;

    //Find the entry circle   找到进入圈的起点
    SurveyCircle.x = SurveyFromWP.x;
    SurveyCircle.y = EntryPoint.y;

    //Go into entry circle state   进入圈
    CSurveyStatus = Entry;
    LINE_STOP_FUNCTION;
  }

  return FALSE;
}

bool_t PolygonSurvey(void)
{
  struct Point2D C;
  struct Point2D ToP;
  struct Point2D FromP;
  float ys;
  static struct Point2D LastPoint;
  int i;
  bool_t SweepingBack = FALSE;
  float XIntercept1 = 0;
  float XIntercept2 = 0;
  float DInt1 = 0;
  float DInt2 = 0;

  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(waypoints[SurveyEntryWP].a, 0.);

  switch(CSurveyStatus)
  {
  case Entry:    //进入模式
    //Rotate and translate circle point into real world
    //旋转并转换点到达真实世界
    C.x = SurveyCircle.x;
    C.y = SurveyCircle.y;
    RotateAndTranslateToWorld(&C, 0, SmallestCorner.x, SmallestCorner.y);
    RotateAndTranslateToWorld(&C, SurveyTheta, 0, 0);

    //follow the circle  沿着圈非
    nav_circle_XY(C.x, C.y, SurveyRadius);

    if(NavQdrCloseTo(SurveyCircleQdr) && NavCircleCount() > .1 && stateGetPositionEnu_f()->z > waypoints[SurveyEntryWP].a-10)
    {
      CSurveyStatus = Sweep;
      nav_init_stage();
      LINE_START_FUNCTION;
    }
    break;
  case Sweep:   扫描模式
    //Rotate and Translate Line points into real world  
    //旋转并转换线点到真实世界
    ToP.x = SurveyToWP.x;
    ToP.y = SurveyToWP.y;
    FromP.x = SurveyFromWP.x;
    FromP.y = SurveyFromWP.y;

    RotateAndTranslateToWorld(&ToP, 0, SmallestCorner.x, SmallestCorner.y);
    RotateAndTranslateToWorld(&ToP, SurveyTheta, 0, 0);

    RotateAndTranslateToWorld(&FromP, 0, SmallestCorner.x, SmallestCorner.y);
    RotateAndTranslateToWorld(&FromP, SurveyTheta, 0, 0);

    //follow the line  沿着线飞
    nav_route_xy(FromP.x,FromP.y,ToP.x,ToP.y);
    if(nav_approaching_xy(ToP.x,ToP.y,FromP.x,FromP.y, 0))
    {
      LastPoint.x = SurveyToWP.x;
      LastPoint.y = SurveyToWP.y;

      if(LastPoint.y+dSweep >= MaxY || LastPoint.y+dSweep <= 0) //Your out of the Polygon so Sweep Back   出了多边形，所以要扫描回来
      {
        dSweep = -dSweep;
        ys = LastPoint.y+(dSweep/2);

        if(dSweep >= 0)
          SurveyCircleQdr = -DegOfRad(SurveyTheta);
        else
          SurveyCircleQdr = 180-DegOfRad(SurveyTheta);
        SweepingBack = TRUE;
        PolySurveySweepBackNum++;
      }
      else
      {
        //Find y value of the first sweep   找到第一次扫描的y值
        ys = LastPoint.y+dSweep;
      }

      //Find the edges which intercet the sweep line first   找到和第一次扫描线相交的边缘
      for(i = 0; i < SurveySize; i++)
      {
        if(EdgeMinY[i] < ys && EdgeMaxY[i] >= ys)
        {
          XIntercept2 = XIntercept1;
          XIntercept1 = EvaluateLineForX(ys, Edges[i]);
        }
      }

      //Find point to come from and point to go to   找到起始点
      DInt1 = XIntercept1 -  LastPoint.x;
      DInt2 = XIntercept2 - LastPoint.x;

      if(DInt1 * DInt2 >= 0)
      {
        if(fabs(DInt2) <= fabs(DInt1))
        {
          SurveyToWP.x = XIntercept1;
          SurveyToWP.y = ys;

          SurveyFromWP.x = XIntercept2;
          SurveyFromWP.y = ys;
        }
        else
        {
          SurveyToWP.x = XIntercept2;
          SurveyToWP.y = ys;

          SurveyFromWP.x = XIntercept1;
          SurveyFromWP.y = ys;
        }
      }
      else
      {
        if((SurveyToWP.x - SurveyFromWP.x) > 0 && DInt2 > 0)
        {
          SurveyToWP.x = XIntercept1;
          SurveyToWP.y = ys;

          SurveyFromWP.x = XIntercept2;
          SurveyFromWP.y = ys;
        }
        else if((SurveyToWP.x - SurveyFromWP.x) < 0 && DInt2 < 0)
        {
          SurveyToWP.x = XIntercept1;
          SurveyToWP.y = ys;

          SurveyFromWP.x = XIntercept2;
          SurveyFromWP.y = ys;
        }
        else
        {
          SurveyToWP.x = XIntercept2;
          SurveyToWP.y = ys;

          SurveyFromWP.x = XIntercept1;
          SurveyFromWP.y = ys;
        }
      }



      if(fabs(LastPoint.x-SurveyToWP.x) > fabs(SurveyFromWP.x-SurveyToWP.x))
        SurveyCircle.x = LastPoint.x;
      else
        SurveyCircle.x = SurveyFromWP.x;


      if(!SweepingBack)
        SurveyCircle.y = LastPoint.y+(dSweep/2);
      else
        SurveyCircle.y = LastPoint.y;

      //Find the direction to circle   找到圈的方向
      if(ys > 0 && SurveyToWP.x > SurveyFromWP.x)
        SurveyRadius = dSweep/2;
      else if(ys < 0 && SurveyToWP.x < SurveyFromWP.x)
        SurveyRadius = dSweep/2;
      else
        SurveyRadius = -dSweep/2;

      //Go into circle state
      CSurveyStatus = SweepCircle;
      nav_init_stage();
      LINE_STOP_FUNCTION;
      PolySurveySweepNum++;
    }

    break;
  case SweepCircle:   扫描圈的模式
    //Rotate and translate circle point into real world
    //旋转并转换圈点到真实世界
    C.x = SurveyCircle.x;
    C.y = SurveyCircle.y;
    RotateAndTranslateToWorld(&C, 0, SmallestCorner.x, SmallestCorner.y);
    RotateAndTranslateToWorld(&C, SurveyTheta, 0, 0);

    //follow the circle   沿着圈飞
    nav_circle_XY(C.x, C.y, SurveyRadius);

    if(NavQdrCloseTo(SurveyCircleQdr) && NavCircleCount() > 0)
    {
      CSurveyStatus = Sweep;
      nav_init_stage();
      LINE_START_FUNCTION;
    }
    break;
  case Init:
    return FALSE;
  default:
    return FALSE;
  }
  return TRUE;
}

/************** Vertical Raster **********************************************/
               /*垂直光栅？？？*/

/** Copy of nav line. The only difference is it changes altitude every sweep, but doesn't come out of circle until
    it reaches altitude.
   复制航线。惟一的区别是每次扫描时改变高度，但是不出这个圈，直到到达一定的高度
*/
enum line_status { LR12, LQC21, LTC2, LQC22, LR21, LQC12, LTC1, LQC11 };
static enum line_status line_status;

bool_t InitializeVerticalRaster( void ) {
  line_status = LR12;
  return FALSE;
}

bool_t VerticalRaster(uint8_t l1, uint8_t l2, float radius, float AltSweep) {
  radius = fabs(radius);
  float alt = waypoints[l1].a;
  waypoints[l2].a = alt;

  float l2_l1_x = WaypointX(l1) - WaypointX(l2);
  float l2_l1_y = WaypointY(l1) - WaypointY(l2);
  float d = sqrt(l2_l1_x*l2_l1_x+l2_l1_y*l2_l1_y);

  /* Unit vector from l1 to l2    计算由l1到l2的向量 */
  float u_x = l2_l1_x / d;
  float u_y = l2_l1_y / d;

  /* The half circle centers and the other leg */
  struct point l2_c1 = { WaypointX(l1) + radius * u_y,
                         WaypointY(l1) + radius * -u_x,
                         alt  };
  struct point l2_c2 = { WaypointX(l1) + 1.732*radius * u_x,
                         WaypointY(l1) + 1.732*radius * u_y,
                         alt  };
  struct point l2_c3 = { WaypointX(l1) + radius * -u_y,
                         WaypointY(l1) + radius * u_x,
                         alt  };

  struct point l1_c1 = { WaypointX(l2) + radius * -u_y,
                         WaypointY(l2) + radius * u_x,
                         alt  };
  struct point l1_c2 = { WaypointX(l2) +1.732*radius * -u_x,
                         WaypointY(l2) + 1.732*radius * -u_y,
                         alt  };
  struct point l1_c3 = { WaypointX(l2) + radius * u_y,
                         WaypointY(l2) + radius * -u_x,
                         alt  };
  float qdr_out_2_1 = M_PI/3. - atan2(u_y, u_x);

  float qdr_out_2_2 = -M_PI/3. - atan2(u_y, u_x);
  float qdr_out_2_3 = M_PI - atan2(u_y, u_x);

  /* Vertical target    垂直目标*/
  NavVerticalAutoThrottleMode(0); /* No pitch */
  NavVerticalAltitudeMode(WaypointAlt(l1), 0.);

  switch (line_status) {
  case LR12: /* From wp l2 to wp l1 */
    NavSegment(l2, l1);
    if (NavApproachingFrom(l1, l2, CARROT)) {
      line_status = LQC21;
      waypoints[l1].a = waypoints[l1].a+AltSweep;
      nav_init_stage();
    }
    break;
  case LQC21:
    nav_circle_XY(l2_c1.x, l2_c1.y, radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_1)-10)) {
      line_status = LTC2;
      nav_init_stage();
    }
    break;
  case LTC2:
    nav_circle_XY(l2_c2.x, l2_c2.y, -radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_2)+10) && stateGetPositionEnu_f()->z >= (waypoints[l1].a-10)) {
      line_status = LQC22;
      nav_init_stage();
    }
    break;
  case LQC22:
    nav_circle_XY(l2_c3.x, l2_c3.y, radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_3)-10)) {
      line_status = LR21;
      nav_init_stage();
    }
    break;
  case LR21: /* From wp l1 to wp l2 */
    NavSegment(l1, l2);
    if (NavApproachingFrom(l2, l1, CARROT)) {
      line_status = LQC12;
      waypoints[l1].a = waypoints[l1].a+AltSweep;
      nav_init_stage();
    }
    break;
  case LQC12:
    nav_circle_XY(l1_c1.x, l1_c1.y, radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_1 + M_PI)-10)) {
      line_status = LTC1;
      nav_init_stage();
    }
    break;
  case LTC1:
    nav_circle_XY(l1_c2.x, l1_c2.y, -radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_2 + M_PI)+10) && stateGetPositionEnu_f()->z >= (waypoints[l1].a-5)) {
      line_status = LQC11;
      nav_init_stage();
    }
    break;
  case LQC11:
    nav_circle_XY(l1_c3.x, l1_c3.y, radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_3 + M_PI)-10)) {
      line_status = LR12;
      nav_init_stage();
    }
  default:
    break;
  }
  return TRUE; /* This pattern never ends */
}

/************** SkidLanding **********************************************/
              //刹车着陆
/**
   Landing Routine

   @verbatim
   <section name="Landing" prefix="Landing_">
   <define name="AFHeight" value="50" unit="m"/>
   <define name="FinalHeight" value="5" unit="m"/>
   <define name="FinalStageTime" value="5" unit="s"/>
   </section>
   @endverbatim
*/

#ifndef Landing_AFHeight
#define Landing_AFHeight 50
#endif
#ifndef Landing_FinalHeight
#define Landing_FinalHeight 5
#endif
#ifndef Landing_FinalStageTime
#define Landing_FinalStageTime 5
#endif

enum LandingStatus { CircleDown, LandingWait, Final, Approach };
static enum LandingStatus CLandingStatus;
static uint8_t AFWaypoint;
static uint8_t TDWaypoint;
static float LandRadius;
static struct Point2D LandCircle;
static float LandAppAlt;
static float LandCircleQDR;
static float ApproachQDR;
static float FinalLandAltitude;
static uint8_t FinalLandCount;

bool_t InitializeSkidLanding(uint8_t AFWP, uint8_t TDWP, float radius)
{
  AFWaypoint = AFWP;
  TDWaypoint = TDWP;
  CLandingStatus = CircleDown;
  LandRadius = radius;
  LandAppAlt = stateGetPositionEnu_f()->z;
  FinalLandAltitude = Landing_FinalHeight;
  FinalLandCount = 1;
  waypoints[AFWaypoint].a = waypoints[TDWaypoint].a + Landing_AFHeight;

  float x_0 = WaypointX(TDWaypoint) - WaypointX(AFWaypoint);
  float y_0 = WaypointY(TDWaypoint) - WaypointY(AFWaypoint);

  /* Unit vector from AF to TD */
  float d = sqrt(x_0*x_0+y_0*y_0);
  float x_1 = x_0 / d;
  float y_1 = y_0 / d;

  LandCircle.x = WaypointX(AFWaypoint) + y_1 * LandRadius;
  LandCircle.y = WaypointY(AFWaypoint) - x_1 * LandRadius;

  LandCircleQDR = atan2(WaypointX(AFWaypoint)-LandCircle.x, WaypointY(AFWaypoint)-LandCircle.y);

  if(LandRadius > 0)
  {
    ApproachQDR = LandCircleQDR-RadOfDeg(90);
    LandCircleQDR = LandCircleQDR-RadOfDeg(45);
  }
  else
  {
    ApproachQDR = LandCircleQDR+RadOfDeg(90);
    LandCircleQDR = LandCircleQDR+RadOfDeg(45);
  }


  return FALSE;
}

//CircleDown, LandingWait, Final, Approach 几种模式下的刹车操作方式
bool_t SkidLanding(void)
{
  switch(CLandingStatus)
  {
  case CircleDown:
    NavVerticalAutoThrottleMode(0); /* No pitch */

    if(NavCircleCount() < .1)
    {
      NavVerticalAltitudeMode(LandAppAlt, 0);
    }
    else
      NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0);

    nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius);

    if(stateGetPositionEnu_f()->z < waypoints[AFWaypoint].a + 5)
    {
      CLandingStatus = LandingWait;
      nav_init_stage();
    }

    break;

  case LandingWait:
    NavVerticalAutoThrottleMode(0); /* No pitch */
    NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0);
    nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius);

    if(NavCircleCount() > 0.5 && NavQdrCloseTo(DegOfRad(ApproachQDR)))
    {
      CLandingStatus = Approach;
      nav_init_stage();
    }
    break;

  case Approach:
    kill_throttle = 1;
    NavVerticalAutoThrottleMode(0); /* No pitch */
    NavVerticalAltitudeMode(waypoints[AFWaypoint].a, 0);
    nav_circle_XY(LandCircle.x, LandCircle.y, LandRadius);

    if(NavQdrCloseTo(DegOfRad(LandCircleQDR)))
    {
      CLandingStatus = Final;
      nav_init_stage();
    }
    break;

  case Final:
    kill_throttle = 1;
    NavVerticalAutoThrottleMode(0);
    NavVerticalAltitudeMode(waypoints[TDWaypoint].a+FinalLandAltitude, 0);
    nav_route_xy(WaypointX(AFWaypoint),WaypointY(AFWaypoint),WaypointX(TDWaypoint),WaypointY(TDWaypoint));
    if(stage_time >= Landing_FinalStageTime*FinalLandCount)
    {
      FinalLandAltitude = FinalLandAltitude/2;
      FinalLandCount++;
    }
    break;

  default:

    break;
  }
  return TRUE;
}

enum FLStatus { FLInitialize, FLCircleS, FLLine, FLFinished };
static enum FLStatus CFLStatus = FLInitialize;
static struct Point2D FLCircle;
static struct Point2D FLFROMWP;
static struct Point2D FLTOWP;
static float FLQDR;
static float FLRadius;

bool_t FlightLine(uint8_t From_WP, uint8_t To_WP, float radius, float Space_Before, float Space_After)
{
  struct Point2D V;
  struct Point2D P;
  float dv;

  switch(CFLStatus)
  {
  case FLInitialize:

    //Translate WPs so From_WP is origin   旋转航点以使起始航点是源
    V.x = WaypointX(To_WP) - WaypointX(From_WP);
    V.y = WaypointY(To_WP) - WaypointY(From_WP);

    //Record Aircraft Position   记录飞机的位置
    P.x = stateGetPositionEnu_f()->x;
    P.y = stateGetPositionEnu_f()->y;

    //Rotate Aircraft Position so V is aligned with x axis      旋转飞机的位置以使V和x轴对齐
    TranslateAndRotateFromWorld(&P, atan2(V.y,V.x), WaypointX(From_WP), WaypointY(From_WP));

    //Find which side of the flight line the aircraft is on  找出飞机在航线的哪一侧
    if(P.y > 0)
      FLRadius = -radius;
    else
      FLRadius = radius;

    //Find unit vector of V   计算V向量
    dv = sqrt(V.x*V.x+V.y*V.y);
    V.x = V.x / dv;
    V.y = V.y / dv;

    //Find begin and end points of flight line   找到航线的起始点
    FLFROMWP.x = -V.x*Space_Before;
    FLFROMWP.y = -V.y*Space_Before;

    FLTOWP.x = V.x*(dv+Space_After);
    FLTOWP.y = V.y*(dv+Space_After);

    //Find center of circle   找到圈的中心
    FLCircle.x = FLFROMWP.x + V.y * FLRadius;
    FLCircle.y = FLFROMWP.y - V.x * FLRadius;

    //Find the angle to exit the circle    找到退出圈的角度
    FLQDR = atan2(FLFROMWP.x-FLCircle.x, FLFROMWP.y-FLCircle.y);

    //Translate back   转换回来
    FLFROMWP.x = FLFROMWP.x + WaypointX(From_WP);
    FLFROMWP.y = FLFROMWP.y + WaypointY(From_WP);

    FLTOWP.x = FLTOWP.x + WaypointX(From_WP);
    FLTOWP.y = FLTOWP.y + WaypointY(From_WP);

    FLCircle.x = FLCircle.x + WaypointX(From_WP);
    FLCircle.y = FLCircle.y + WaypointY(From_WP);

    CFLStatus = FLCircleS;
    nav_init_stage();

    break;

  case FLCircleS:  //一种模式：飞圈

    NavVerticalAutoThrottleMode(0); /* No pitch */
    NavVerticalAltitudeMode(waypoints[From_WP].a, 0);

    nav_circle_XY(FLCircle.x, FLCircle.y, FLRadius);

    if(NavCircleCount() > 0.2 && NavQdrCloseTo(DegOfRad(FLQDR)))
    {
      CFLStatus = FLLine;
      LINE_START_FUNCTION;
      nav_init_stage();
    }
    break;

  case FLLine:    //一种飞行模式：飞线

    NavVerticalAutoThrottleMode(0); /* No pitch */
    NavVerticalAltitudeMode(waypoints[From_WP].a, 0);

    nav_route_xy(FLFROMWP.x,FLFROMWP.y,FLTOWP.x,FLTOWP.y);


    if(nav_approaching_xy(FLTOWP.x,FLTOWP.y,FLFROMWP.x,FLFROMWP.y, 0))
    {
      CFLStatus = FLFinished;
      LINE_STOP_FUNCTION;
      nav_init_stage();
    }
    break;

  case FLFinished:    //飞行结束
    CFLStatus = FLInitialize;
    nav_init_stage();
    return FALSE;
    break;

  default:
    break;
  }
  return TRUE;

}

static uint8_t FLBlockCount = 0;

bool_t FlightLineBlock(uint8_t First_WP, uint8_t Last_WP, float radius, float Space_Before, float Space_After)
{
  if(First_WP < Last_WP)
  {
    FlightLine(First_WP+FLBlockCount, First_WP+FLBlockCount+1, radius, Space_Before, Space_After);

    if(CFLStatus == FLInitialize)
    {
      FLBlockCount++;
      if(First_WP+FLBlockCount >= Last_WP)
      {
        FLBlockCount = 0;
        return FALSE;
      }
    }
  }
  else
  {
    FlightLine(First_WP-FLBlockCount, First_WP-FLBlockCount-1, radius, Space_Before, Space_After);

    if(CFLStatus == FLInitialize)
    {
      FLBlockCount++;
      if(First_WP-FLBlockCount <= Last_WP)
      {
        FLBlockCount = 0;
        return FALSE;
      }
    }
  }

  return TRUE;
}


/*
  Translates point so (transX, transY) are (0,0) then rotates the point around z by Zrot
  转换点的坐标使（transY,transX）变成（0,0），然后绕着z轴旋转
*/
void TranslateAndRotateFromWorld(struct Point2D *p, float Zrot, float transX, float transY)
{
  float temp;

  p->x = p->x - transX;
  p->y = p->y - transY;

  temp = p->x;
  p->x = p->x*cos(Zrot)+p->y*sin(Zrot);
  p->y = -temp*sin(Zrot)+p->y*cos(Zrot);
}

/// Rotates point round z by -Zrot then translates so (0,0) becomes (transX,transY)
 //绕着z轴旋转，然后转换坐标使（0,0）变成（transX，transY）
void RotateAndTranslateToWorld(struct Point2D *p, float Zrot, float transX, float transY)
{
  float temp = p->x;

  p->x = p->x*cos(Zrot)-p->y*sin(Zrot);
  p->y = temp*sin(Zrot)+p->y*cos(Zrot);

  p->x = p->x + transX;
  p->y = p->y + transY;
}

void FindInterceptOfTwoLines(float *x, float *y, struct Line L1, struct Line L2)
{
  *x = ((L2.b-L1.b)/(L1.m-L2.m));
  *y = L1.m*(*x)+L1.b;
}

float EvaluateLineForY(float x, struct Line L)
{
  return (L.m*x)+L.b;
}

float EvaluateLineForX(float y, struct Line L)
{
  return ((y-L.b)/L.m);
}

float DistanceEquation(struct Point2D p1,struct Point2D p2)
{
  return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}
