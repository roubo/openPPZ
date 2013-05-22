/* This file has been generated from /home/chorus/chorus/conf/flight_plans/rotorcraft_basic.xml */
/* Please DO NOT EDIT */

#ifndef FLIGHT_PLAN_H
#define FLIGHT_PLAN_H

#include "std.h"
#include "generated/modules.h"
#include "autopilot.h"
#define FLIGHT_PLAN_NAME "Booz Test Enac"
#define NAV_UTM_EAST0 377349
#define NAV_UTM_NORTH0 4824583
#define NAV_UTM_ZONE0 31
#define NAV_LAT0 435641194 /* 1e7deg */
#define NAV_LON0 14812805 /* 1e7deg */
#define NAV_ALT0 147000 /* mm above msl */
#define NAV_MSL0 51850 /* mm, EGM96 geoid-height (msl) over ellipsoid */
#define QFU 0.0
#define WP_dummy 0
#define WP_HOME 1
#define WP_CLIMB 2
#define WP_STDBY 3
#define WP_p1 4
#define WP_p2 5
#define WP_p3 6
#define WP_p4 7
#define WP_CAM 8
#define WP_TD 9
#define WAYPOINTS { \
 {42.0, 42.0, 152},\
 {0.0, 0.0, 152},\
 {0.0, 5.0, 152},\
 {-2.0, -5.0, 152},\
 {3.6, -13.9, 152},\
 {27.5, -48.2, 152},\
 {16.7, -19.6, 152},\
 {13.7, -40.7, 152},\
 {-20.0, -50.0, 149.},\
 {5.6, -10.9, 152},\
};
#define WAYPOINTS_LLA { \
 {435645043, 14817909, 15200}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {435641194, 14812805, 15200}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {435641644, 14812794, 15200}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {435640741, 14812569, 15200}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {435639949, 14813282, 15200}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {435636901, 14816318, 15200}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {435639457, 14814917, 15200}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {435637553, 14814593, 15200}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {435636661, 14810443, 15200}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
 {435640222, 14813523, 15200}, /* 1e7deg, 1e7deg, cm (hmsl=51.85m) */ \
};
#define NB_WAYPOINT 10
#define NB_BLOCK 15
#define GROUND_ALT 147.
#define GROUND_ALT_CM 14700
#define SECURITY_HEIGHT 2.
#define SECURITY_ALT 149.
#define HOME_MODE_HEIGHT 2.
#define MAX_DIST_FROM_HOME 400.
#ifdef NAV_C

static inline void auto_nav(void) {
  switch (nav_block) {
    Block(0) // Wait GPS
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (! (NavKillThrottle()))
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
        if (! (NavKillThrottle()))
          NextStageAndBreak();
        break;
      Stage(1)
        if (FALSE) NextStageAndBreak() else {
          NavAttitude(RadOfDeg(0));
          NavVerticalAutoThrottleMode(RadOfDeg(0));
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

    Block(3) // Start Engine
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (! (NavResurrect()))
          NextStageAndBreak();
        break;
      Stage(1)
        if (FALSE) NextStageAndBreak() else {
          NavAttitude(RadOfDeg(0));
          NavVerticalAutoThrottleMode(RadOfDeg(0));
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

    Block(4) // Takeoff
    ; // pre_call
    if ((nav_block != 5) && ((stateGetPositionEnu_f())->z>2.000000)) { GotoBlock(5); return; }
    switch(nav_stage) {
      Stage(0)
        if (! (NavSetWaypointHere(WP_CLIMB)))
          NextStageAndBreak();
        break;
      Stage(1)
        NavGotoWaypoint(2);
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalClimbMode(0.500000);
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(5) // Standby
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        NavGotoWaypoint(3);
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(3), 0.);
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(6) // stay_p1
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        NavGotoWaypoint(4);
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(4), 0.);
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(7) // go_p2
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (NavApproaching(5,CARROT)) NextStageAndBreakFrom(5) else {
          NavGotoWaypoint(5);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode(WaypointAlt(5), 0.);
        }
        break;
      Stage(1)
        GotoBlock(6);
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(8) // line_p1_p2
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (NavApproachingFrom(5,4,CARROT)) NextStageAndBreakFrom(5) else {
          NavSegment(4, 5);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode(WaypointAlt(5), 0.);
        }
        break;
      Stage(1)
        NavGotoWaypoint(5);
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(5), 0.);
        if ((stage_time>10)) NextStageAndBreak();
        break;
      Stage(2)
        if (NavApproachingFrom(4,5,CARROT)) NextStageAndBreakFrom(4) else {
          NavSegment(5, 4);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode(WaypointAlt(4), 0.);
        }
        break;
      Stage(3)
        GotoBlock(6);
        break;
      default:
      Stage(4)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(9) // route
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (NavApproachingFrom(6,4,CARROT)) NextStageAndBreakFrom(6) else {
          NavSegment(4, 6);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode(WaypointAlt(6), 0.);
        }
        break;
      Stage(1)
        if (NavApproachingFrom(7,6,CARROT)) NextStageAndBreakFrom(7) else {
          NavSegment(6, 7);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode(WaypointAlt(7), 0.);
        }
        break;
      Stage(2)
        if (NavApproachingFrom(4,7,CARROT)) NextStageAndBreakFrom(4) else {
          NavSegment(7, 4);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode(WaypointAlt(4), 0.);
        }
        break;
      Stage(3)
        GotoBlock(6);
        break;
      default:
      Stage(4)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(10) // circle
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalAltitudeMode(WaypointAlt(4), 0.);
        NavCircleWaypoint(4, nav_radius);
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(11) // land here
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (! (NavSetWaypointHere(WP_TD)))
          NextStageAndBreak();
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(12) // land
    ; // pre_call
    switch(nav_stage) {
      Stage(0)
        if (NavApproaching(9,CARROT)) NextStageAndBreakFrom(9) else {
          NavGotoWaypoint(9);
          NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
          NavVerticalAltitudeMode(WaypointAlt(9), 0.);
        }
        break;
      default:
      Stage(1)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(13) // flare
    ; // pre_call
    if ((nav_block != 2) && NavDetectGround()) { GotoBlock(2); return; }
    switch(nav_stage) {
      Stage(0)
        if (! (NavStartDetectGround()))
          NextStageAndBreak();
        break;
      Stage(1)
        NavGotoWaypoint(9);
        NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
        NavVerticalClimbMode(-(0.800000));
        break;
      default:
      Stage(2)
        NextBlock();
        break;
    }
    ; // post_call
    break;

    Block(14) // HOME
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
