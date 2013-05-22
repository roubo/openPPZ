/* This file has been generated from /home/chorus/chorus/conf/flight_plans/versatile.xml */
/* Please DO NOT EDIT */

#ifndef FLIGHT_PLAN_H
#define FLIGHT_PLAN_H

#include "std.h"
#include "generated/modules.h"
#include "subsystems/datalink/datalink.h"
#define FLIGHT_PLAN_NAME "Versatile"
#define NAV_UTM_EAST0 360286
#define NAV_UTM_NORTH0 4813592
#define NAV_UTM_ZONE0 31
#define NAV_LAT0 434622000 /* 1e7deg */
#define NAV_LON0 12729000 /* 1e7deg */
#define NAV_ALT0 0 /* mm above msl */
#define NAV_MSL0 51850 /* mm, EGM96 geoid-height (msl) over ellipsoid */
#define QFU 270.0
#define WP_dummy 0
#define WP_HOME 1
#define WP_STDBY 2
#define WP_1 3
#define WP_2 4
#define WP_MOB 5
#define WP_S1 6
#define WP_S2 7
#define WP_AF 8
#define WP_TD 9
#define WP_BASELEG 10
#define WP__1 11
#define WP__2 12
#define WP__3 13
#define WP__4 14
#define WP_CLIMB 15
#define WAYPOINTS { \
 {42.0, 42.0, 75},\
 {0.0, 0.0, 75},\
 {20.0, 80.0, 75},\
 {44.8, 102.2, 75},\
 {-63.5, 122.9, 75},\
 {-11.5, -21.2, 75},\
 {-151.6, 80.4, 75},\
 {180.1, 214.9, 75},\
 {200.0, -10.0, 30},\
 {80.0, 20.0, 0},\
 {26.9, -23.0, 75},\
 {-100.0, 0.0, 75},\
 {-100.0, 200.0, 75},\
 {100.0, 200.0, 75},\
 {100.0, 0.0, 75},\
 {-122.5, 35.4, 75},\
};
#define WAYPOINTS_LLA { \
 {434625858, 12734081, 7500}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {434621999, 12729000, 7500}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {434629237, 12731266, 7500}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {434631281, 12734273, 7500}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {434632942, 12720838, 7500}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {434620070, 12727633, 7500}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {434628953, 12710061, 7500}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {434641677, 12750703, 7500}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {434621473, 12753737, 3000}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {434623949, 12738833, 0}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {434619980, 12732382, 7500}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {434621813, 12716643, 7500}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {434639813, 12716130, 7500}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {434640187, 12740843, 7500}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {434622186, 12741356, 7500}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {434624957, 12713772, 7500}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
};
#define NB_WAYPOINT 16
#define NB_BLOCK 35
#define GROUND_ALT 0.
#define GROUND_ALT_CM 0
#define SECURITY_HEIGHT 25.
#define SECURITY_ALT 25.
#define HOME_MODE_HEIGHT 50.
#define MAX_DIST_FROM_HOME 1500.
static inline bool_t InsideSquare(float _x, float _y) { \
  if (_y <= 200.0) {
    if (_y <= 0.0) {
      return FALSE;
    } else {
      return (-100.0<= _x && _x <= 100.0);
    }
  } else {
    return FALSE;
  }
}
#ifdef NAV_C

static inline void auto_nav(void) {
  switch (nav_block) {
    Block(0) // Wait GPS
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        kill_throttle = 1;
        NextStageAndBreak();
        break;
      Label(while_1)
      Stage(1)
        if (! (!(GpsFixValid()))) Goto(endwhile_2) else NextStageAndBreak();
        Stage(2)
          Goto(while_1)
        Label(endwhile_2)
      default:
      Stage(3)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(1) // Geo init
    ; // pre_call
    switch(nav_stage) {
      Label(while_3)
      Stage(0)
        if (! (LessThan(NavBlockTime(),10))) Goto(endwhile_4) else NextStageAndBreak();
        Stage(1)
          Goto(while_3)
        Label(endwhile_4)
      Stage(2)
        if (! (NavSetGroundReferenceHere()))
          NextStageAndBreak();
        break;
      default:
      Stage(3)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(2) // Holding point
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        kill_throttle = 1;
        NextStageAndBreak();
        break;
      Stage(1)
        {
          NavAttitude(RadOfDeg(0));
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalThrottleMode(9600*(0));
        }
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(3) // Takeoff
    ; // pre_call
    if ((nav_block != 4) && (GetPosAlt()>(ground_alt+25))) { GotoBlock(4); return; }
    switch(nav_stage) {
      Stage(0)
        kill_throttle = 0;
        NextStageAndBreak();
        break;
      Stage(1)
        if (NavApproaching(15,CARROT)) NextStageAndBreakFrom(15) else {
          NavGotoWaypoint(15);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode(WaypointAlt(15), 0.);
        }
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(4) // Standby
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(2), 0.);
        NavCircleWaypoint(2, nav_radius);
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(5) // Figure 8 around wp 1
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        nav_eight_init();
        NextStageAndBreak();
      Stage(1)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(3), 0.);
        Eight(3, 4, nav_radius);
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(6) // Oval 1-2
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        nav_oval_init();
        NextStageAndBreak();
      Stage(1)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(3), 0.);
        Oval(3, 4, nav_radius);
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(7) // MOB
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (! (NavSetWaypointHere(WP_MOB)))
          NextStageAndBreak();
        break;
      Stage(1)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(5), 0.);
        NavCircleWaypoint(5, 100);
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(8) // Auto pitch (circle wp 1)
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        NavVerticalAutoPitchMode(9600*(0.700000));
        NavVerticalAltitudeMode(WaypointAlt(3), 0.);
        NavCircleWaypoint(3, 75);
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(9) // Climb 75% throttle
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        NavVerticalAutoThrottleMode(RadOfDeg(10));
        NavVerticalThrottleMode(9600*(0.750000));
        NavCircleWaypoint(3, (50+((GetPosAlt()-ground_alt)/2)));
        if (((10>PowerVoltage())||(GetPosAlt()>(ground_alt+1350)))) NextStageAndBreak();
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(10) // Climb 0m/s
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalClimbMode(0);
        NavCircleWaypoint(3, nav_radius);
        if ((10>PowerVoltage())) NextStageAndBreak();
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(11) // Climb 1m/s
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        NavVerticalAutoThrottleMode(RadOfDeg(5));
        NavVerticalClimbMode(1);
        NavCircleWaypoint(3, (50+((GetPosAlt()-ground_alt)/2)));
        if ((10>PowerVoltage())) NextStageAndBreak();
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(12) // Climb nav_climb m/s
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalClimbMode(nav_climb);
        NavCircleWaypoint(3, nav_radius);
        if (((10>PowerVoltage())||(GetPosAlt()>(ground_alt+1350)))) NextStageAndBreak();
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(13) // Circle 0% throttle
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        NavVerticalAutoThrottleMode(RadOfDeg(fp_pitch));
        NavVerticalThrottleMode(9600*(0.000000));
        NavCircleWaypoint(3, nav_radius);
        if (((ground_alt+50)>GetPosAlt())) NextStageAndBreak();
        break;
      Stage(1)
        GotoBlock(4);
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(14) // Oval 0% throttle
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        nav_oval_init();
        NextStageAndBreak();
      Stage(1)
        NavVerticalAutoThrottleMode(RadOfDeg(fp_pitch));
        NavVerticalThrottleMode(9600*(0.000000));
        Oval(3, 4, nav_radius);
        if (((ground_alt+50)>GetPosAlt())) NextStageAndBreak();
        break;
      Stage(2)
        GotoBlock(4);
        break;
      default:
      Stage(3)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(15) // Route 1-2
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (NavApproachingFrom(4,3,0)) NextStageAndBreakFrom(4) else {
          NavSegment(3, 4);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode(WaypointAlt(4), 0.);
        }
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(16) // Stack wp 2
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(4), 0.);
        NavCircleWaypoint(4, 75);
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(17) // Route 2-1
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (NavApproachingFrom(3,4,0)) NextStageAndBreakFrom(3) else {
          NavSegment(4, 3);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode(WaypointAlt(3), 0.);
        }
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(18) // Stack wp 1
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(3), 0.);
        NavCircleWaypoint(3, 75);
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(19) // Glide 1-2
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (NavApproachingFrom(4,3,CARROT)) NextStageAndBreakFrom(4) else {
          NavSegment(3, 4);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavGlide(3, 4);
        }
        break;
      Stage(1)
        GotoBlock(4);
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(20) // Survey S1-S2
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        NavSurveyRectangleInit(6, 7, 150, NS);
        NextStageAndBreak();
      Stage(1)
        NavSurveyRectangle(6, 7);
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(21) // Land Right AF-TD
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        nav_radius = DEFAULT_CIRCLE_RADIUS;
        NextStageAndBreak();
        break;
      Stage(1)
        GotoBlock(23);
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(22) // Land Left AF-TD
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        nav_radius = -(DEFAULT_CIRCLE_RADIUS);
        NextStageAndBreak();
        break;
      Stage(1)
        GotoBlock(23);
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(23) // land
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (! (nav_compute_baseleg(WP_AF, WP_TD, WP_BASELEG, nav_radius)))
          NextStageAndBreak();
        break;
      Stage(1)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(10), 0.);
        NavCircleWaypoint(10, nav_radius);
        if ((NavCircleCount()>0.500000)) NextStageAndBreak();
        break;
      Stage(2)
        v_ctl_auto_throttle_cruise_throttle = V_CTL_AUTO_THROTTLE_MIN_CRUISE_THROTTLE;
        NextStageAndBreak();
        break;
      Stage(3)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(10), 0.);
        NavCircleWaypoint(10, nav_radius);
        if ((NavQdrCloseTo((DegOfRad(baseleg_out_qdr)-((nav_radius/fabs(nav_radius))*10)))&&(10>fabs((GetPosAlt()-WaypointAlt(WP_BASELEG)))))) NextStageAndBreak();
        break;
      default:
      Stage(4)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(24) // final
    ; // pre_call
    if ((nav_block != 25) && ((ground_alt+10)>GetPosAlt())) { GotoBlock(25); return; }
    switch(nav_stage) {
      Stage(0)
        if (NavApproachingFrom(9,8,CARROT)) NextStageAndBreakFrom(9) else {
          NavSegment(8, 9);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavGlide(8, 9);
        }
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(25) // flare
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (NavApproachingFrom(9,8,0)) NextStageAndBreakFrom(9) else {
          NavSegment(8, 9);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalThrottleMode(9600*(0.000000));
        }
        break;
      Stage(1)
        if (FALSE) NextStageAndBreak() else {
          NavAttitude(RadOfDeg(0.000000));
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalThrottleMode(9600*(0.000000));
        }
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(26) // Steps roll -10, +10
    ; // pre_call
    switch(nav_stage) {
      Label(while_5)
      Stage(0)
        if (! (TRUE)) Goto(endwhile_6) else NextStageAndBreak();
        Stage(1)
          if ((stage_time>6)) NextStageAndBreak() else {
            NavAttitude(RadOfDeg(10.000000));
            NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
            NavVerticalAltitudeMode(250, 0.);
          }
          break;
        Stage(2)
          if ((stage_time>6)) NextStageAndBreak() else {
            NavAttitude(RadOfDeg(-(10.000000)));
            NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
            NavVerticalAltitudeMode(250, 0.);
          }
          break;
        Stage(3)
          Goto(while_5)
        Label(endwhile_6)
      default:
      Stage(4)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(27) // Steps roll -20, +20
    ; // pre_call
    switch(nav_stage) {
      Label(while_7)
      Stage(0)
        if (! (TRUE)) Goto(endwhile_8) else NextStageAndBreak();
        Stage(1)
          if ((stage_time>3)) NextStageAndBreak() else {
            NavAttitude(RadOfDeg(20.000000));
            NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
            NavVerticalAltitudeMode(250, 0.);
          }
          break;
        Stage(2)
          if ((stage_time>3)) NextStageAndBreak() else {
            NavAttitude(RadOfDeg(-(20.000000)));
            NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
            NavVerticalAltitudeMode(250, 0.);
          }
          break;
        Stage(3)
          Goto(while_7)
        Label(endwhile_8)
      default:
      Stage(4)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(28) // Steps pitch -10, +10
    ; // pre_call
    switch(nav_stage) {
      Label(while_9)
      Stage(0)
        if (! (TRUE)) Goto(endwhile_10) else NextStageAndBreak();
        Stage(1)
          if ((stage_time>2)) NextStageAndBreak() else {
            NavAttitude(RadOfDeg(0.000000));
            NavVerticalAutoThrottleMode(RadOfDeg(10));
            NavVerticalAltitudeMode(250, 0.);
          }
          break;
        Stage(2)
          if ((stage_time>2)) NextStageAndBreak() else {
            NavAttitude(RadOfDeg(0.000000));
            NavVerticalAutoThrottleMode(RadOfDeg(-(10)));
            NavVerticalAltitudeMode(250, 0.);
          }
          break;
        Stage(3)
          Goto(while_9)
        Label(endwhile_10)
      default:
      Stage(4)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(29) // Heading 30
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (FALSE) NextStageAndBreak() else {
          NavHeading(RadOfDeg(30));
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode((ground_alt+50), 0.);
        }
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(30) // For loop (circles wp 1)
    ; // pre_call
    switch(nav_stage) {
      static int8_t _var_i;
      static int8_t _var_i_to;
      Stage(0)
        _var_i = 0 - 1;
        _var_i_to = 3;
      Label(for_11)
      Stage(1)
        if (++_var_i > _var_i_to) Goto(endfor_12) else NextStageAndBreak();
        Stage(2)
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode(WaypointAlt(3), 0.);
          NavCircleWaypoint(3, (DEFAULT_CIRCLE_RADIUS+(_var_i*10)));
          if ((NavCircleCount()>1)) NextStageAndBreak();
          break;
        Stage(3)
          Goto(for_11)
        Label(endfor_12)
      Stage(4)
        GotoBlock(4);
        break;
      default:
      Stage(5)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(31) // Test datalink (go to wp 2)
    ; // pre_call
    if ((nav_block != 4) && (datalink_time>22)) { GotoBlock(4); return; }
    switch(nav_stage) {
      Stage(0)
        if (NavApproachingFrom(4,2,CARROT)) NextStageAndBreakFrom(4) else {
          NavSegment(2, 4);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode(WaypointAlt(4), 0.);
        }
        break;
      Stage(1)
        if (NavApproachingFrom(2,4,CARROT)) NextStageAndBreakFrom(2) else {
          NavSegment(4, 2);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode(WaypointAlt(2), 0.);
        }
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(32) // Fly in Square
    ; // pre_call
    if ((nav_block != 33) && !(InsideSquare(GetPosX(),GetPosY()))) { GotoBlock(33); return; }
    switch(nav_stage) {
      Stage(0)
        {
          NavAttitude(RadOfDeg(0));
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode((ground_alt+75), 0.);
        }
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(33) // Come back wp 1
    ; // pre_call
    if ((nav_block != 32) && InsideSquare(GetPosX(),GetPosY())) { GotoBlock(32); return; }
    switch(nav_stage) {
      Stage(0)
        if (NavApproaching(3,CARROT)) NextStageAndBreakFrom(3) else {
          NavGotoWaypoint(3);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode(WaypointAlt(3), 0.);
        }
        break;
      Stage(1)
        GotoBlock(32);
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(34) // HOME
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        nav_home();
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    default: break;
  }
}
#endif // NAV_C

#endif // FLIGHT_PLAN_H
