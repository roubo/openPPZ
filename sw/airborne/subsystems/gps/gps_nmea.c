/*
 *
 * Copyright (C) 2008-2011 The Paparazzi Team
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
 * @file gps_nmea.c
 * Parser for the NMEA protocol. 解析 NMEA 0183 协议  
 *
 * TODO: THIS NMEA-PARSER IS NOT WELL TESTED AND INCOMPLETE!!!
 * Status:
 *  Parsing GGA and RMC is complete, GSA and other records are
 *  incomplete.   完全解析了ＧＧＡ和ＰＭＣ，ＧＳＡ和其他的语句没有完全解析
 */

#include "subsystems/gps.h"

#include "led.h"

#if GPS_USE_LATLONG
/* currently needed to get nav_utm_zone0 */
  //当前需要得到ｕｔｍ区域
#include "subsystems/navigation/common_nav.h"
#endif
#include "math/pprz_geodetic_float.h"

#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#ifdef DEBUG_NMEA
// do debug-output if run on the DEBUG_NMEA-target

#endif


struct GpsNmea gps_nmea;

void parse_nmea_GPGSA(void);
void parse_nmea_GPRMC(void);
void parse_nmea_GPGGA(void);


void gps_impl_init( void ) {
  gps_nmea.msg_available = FALSE;
  gps_nmea.pos_available = FALSE;
  gps_nmea.gps_nb_ovrn = 0;
  gps_nmea.msg_len = 0;
}


/**
 * parse GPGSA-nmea-messages stored in
 * nmea_msg_buf .  解析ＧＰＧＳＡ语句并储存在nmea_msg_buf数组里
 */
void parse_nmea_GPGSA(void) {
  int i = 6;     // current position in the message, start after: GPGSA,
                 //当前位置是第六个字符，在“ＧＰＧＳＡ，”之后
  //      char* endptr;  // end of parsed substrings

  // attempt to reject empty packets right away
   //尝试拒收两个逗号之间是空的字符
  if(gps_nmea.msg_buf[i]==',' && gps_nmea.msg_buf[i+1]==',') {
    NMEA_PRINT("p_GPGSA() - skipping empty message\n\r");
    return;
  }

  // get auto2D/3D
  // ignored
  while(gps_nmea.msg_buf[i++] != ',') {              // next field: fix
    if (i >= gps_nmea.msg_len) {
      NMEA_PRINT("p_GPGSA() - skipping incomplete message\n\r");
      return;
    }
  }

  // get 2D/3D-fix
  // set gps_mode=3=3d, 2=2d, 1=no fix or 0
  gps.fix = atoi(&gps_nmea.msg_buf[i]);
  if (gps.fix == 1)
    gps.fix = 0;
  NMEA_PRINT("p_GPGSA() - gps.fix=%i (3=3D)\n\r", gps.fix);
  while(gps_nmea.msg_buf[i++] != ',') {              // next field:satellite-number-0
    if (i >= gps_nmea.msg_len) {
      NMEA_PRINT("p_GPGSA() - skipping incomplete message\n\r");
      return;
    }
  }

  //int satcount = 0;

  // TODO: get sateline-numbers for gps_svinfos
}

/**
 * parse GPRMC-nmea-messages stored in
 * gps_nmea.msg_buf .   解析ＧＰＲＭＣ语句的内容，并存储在gps_nmea.msg_buf数组里
 */
void parse_nmea_GPRMC(void) {
  int i = 6;     // current position in the message, start after: GPRMC,
                 //  当前位置是第六个
  char* endptr;  // end of parsed substrings

  // attempt to reject empty packets right away   不要两个逗号之间为空的字符
  if(gps_nmea.msg_buf[i]==',' && gps_nmea.msg_buf[i+1]==',') {
    NMEA_PRINT("p_GPRMC() - skipping empty message\n\r");
    return;
  }

  // get time  获得时间
  // ignored
  while(gps_nmea.msg_buf[i++] != ',') {              // next field: warning
    if (i >= gps_nmea.msg_len) {
      NMEA_PRINT("p_GPRMC() - skipping incomplete message\n\r");
      return;
    }
  }

  // get warning   获得警告
  // ignored
  while(gps_nmea.msg_buf[i++] != ',') {              // next field: lat
    if (i >= gps_nmea.msg_len) {
      NMEA_PRINT("p_GPRMC() - skipping incomplete message\n\r");
      return;
    }
  }
  // get lat  获得纬度
  // ignored
  while(gps_nmea.msg_buf[i++] != ',') {              // next field: N/S
    if (i >= gps_nmea.msg_len) {
      NMEA_PRINT("p_GPRMC() - skipping incomplete message\n\r");
      return;
    }
  }
  // get North/South    获得南北信息 Ｓ/Ｎ
  // ignored
  while(gps_nmea.msg_buf[i++] != ',') {              // next field: lon
    if (i >= gps_nmea.msg_len) {
      NMEA_PRINT("p_GPRMC() - skipping incomplete message\n\r");
      return;
    }
  }
  // get lon  获得经度
  // ignored
  while(gps_nmea.msg_buf[i++] != ',') {              // next field: E/W
    if (i >= gps_nmea.msg_len) {
      NMEA_PRINT("p_GPRMC() - skipping incomplete message\n\r");
      return;
    }
  }
  // get eath/west   获得东西Ｗ/Ｅ
  // ignored
  while(gps_nmea.msg_buf[i++] != ',') {              // next field: speed
    if (i >= gps_nmea.msg_len) {
      NMEA_PRINT("p_GPRMC() - skipping incomplete message\n\r");
      return;
    }
  }
  // get speed   获得速度
  double speed = strtod(&gps_nmea.msg_buf[i], &endptr);
  gps.gspeed = speed * 1.852 * 100 / (60*60);
  NMEA_PRINT("p_GPRMC() - ground-speed=%d knot = %d cm/s\n\r", (speed*1000), (gps.gspeed*1000));
  while(gps_nmea.msg_buf[i++] != ',') {              // next field: course
    if (i >= gps_nmea.msg_len) {
      NMEA_PRINT("p_GPRMC() - skipping incomplete message\n\r");
      return;
    }
  }
  double course = strtod(&gps_nmea.msg_buf[i], &endptr);
  gps.course = RadOfDeg(course) * 1e7;
  NMEA_PRINT("COURSE: %d \n\r",gps_course);
}


/**
 * parse GPGGA-nmea-messages stored in    解析ＧＰＧＧＡ信息，并存储在gps_nmea.msg_buf 数组中
 * gps_nmea.msg_buf .
 */
void parse_nmea_GPGGA(void) {
  int i = 6;     // current position in the message, start after: GPGGA,   当前位置是第六个
  char* endptr;  // end of parsed substrings
  double degrees, minutesfrac;
  struct LlaCoor_f lla_f;

  // attempt to reject empty packets right away
    //尝试拒收两个逗号之间是空的字符
  if(gps_nmea.msg_buf[i]==',' && gps_nmea.msg_buf[i+1]==',') {
    NMEA_PRINT("p_GPGGA() - skipping empty message\n\r");
    return;
  }

  // get UTC time [hhmmss.sss]    获得ＵＴＣ时间   格式为[hhmmss.sss]
  // ignored GpsInfo.PosLLA.TimeOfFix.f = strtod(&packet[i], &endptr);
  // FIXME: parse UTC time correctly
  double time = strtod(&gps_nmea.msg_buf[i],&endptr);
  gps.tow = (uint32_t)((time+1)*1000);

  //AD TODO: strtod itow
  while(gps_nmea.msg_buf[i++] != ',') {              // next field: latitude
    if (i >= gps_nmea.msg_len) {
      NMEA_PRINT("p_GPGGA() - skipping incomplete message\n\r");
      return;
    }
  }

  // get latitude [ddmm.mmmmm]   获得高度
  double lat = strtod(&gps_nmea.msg_buf[i], &endptr);
  // convert to pure degrees [dd.dddd] format   转换格式
  minutesfrac = modf(lat/100, &degrees);
  lat = degrees + (minutesfrac*100)/60;
  // convert to radians
  //GpsInfo.PosLLA.lat.f *= (M_PI/180);

  while(gps_nmea.msg_buf[i++] != ',') {              // next field: N/S indicator
    if (i >= gps_nmea.msg_len) {
      NMEA_PRINT("p_GPGGA() - skipping incomplete message\n\r");
      return;
    }
  }

  // correct latitute for N/S  改正纬度
  if(gps_nmea.msg_buf[i] == 'S')
    lat = -lat;

  // convert to radians   变成弧度
  lla_f.lat = RadOfDeg(lat);

  gps.lla_pos.lat = lla_f.lat * 1e7; // convert to fixed-point  转换到定点
  NMEA_PRINT("p_GPGGA() - lat=%d gps_lat=%i\n\r", (lat*1000), lla_f.lat);


  while(gps_nmea.msg_buf[i++] != ',') {              // next field: longitude
    if (i >= gps_nmea.msg_len)
      return;
  }

  // get longitude [ddmm.mmmmm]   获得经度
  double lon = strtod(&gps_nmea.msg_buf[i], &endptr);
  // convert to pure degrees [dd.dddd] format   转化成度分秒格式
  minutesfrac = modf(lon/100, &degrees);
  lon = degrees + (minutesfrac*100)/60;
  // convert to radians    转化到弧度格式
  //GpsInfo.PosLLA.lon.f *= (M_PI/180);
  while(gps_nmea.msg_buf[i++] != ',') {              // next field: E/W indicator
    if (i >= gps_nmea.msg_len)
      return;
  }

  // correct latitute for E/W    
  if(gps_nmea.msg_buf[i] == 'W')
    lon = -lon;

  // convert to radians   转换到弧度格式
  lla_f.lon = RadOfDeg(lon);

  gps.lla_pos.lon = lla_f.lon * 1e7; // convert to fixed-point   转换到定点模式
  NMEA_PRINT("p_GPGGA() - lon=%d gps_lon=%i time=%u\n\r", (lon*1000), lla_f.lon, gps.tow);


  while(gps_nmea.msg_buf[i++] != ',') {              // next field: position fix status
    if (i >= gps_nmea.msg_len)
      return;
  }

  // position fix status   锁定位置标志
  // 0 = Invalid, 1 = Valid SPS, 2 = Valid DGPS, 3 = Valid PPS   
  // 0代表所有均无效  1代表ＧＰＳ有效  2代表ＤＧＰＳ有效  3代表ＰＰＳ有效
  // check for good position fix   检查锁定位置
  if( (gps_nmea.msg_buf[i] != '0') && (gps_nmea.msg_buf[i] != ',') )  {
    gps_nmea.pos_available = TRUE;
    NMEA_PRINT("p_GPGGA() - POS_AVAILABLE == TRUE\n\r");
  } else {
    gps_nmea.pos_available = FALSE;
    NMEA_PRINT("p_GPGGA() - gps_pos_available == false\n\r");
  }

  while(gps_nmea.msg_buf[i++] != ',') {              // next field: satellites used
    if (i >= gps_nmea.msg_len) {
      NMEA_PRINT("p_GPGGA() - skipping incomplete message\n\r\r");
      return;
    }
  }
  // get number of satellites used in GPS solution  获取卫星数目
  gps.num_sv = atoi(&gps_nmea.msg_buf[i]); 
  NMEA_PRINT("p_GPGGA() - gps_numSatlitesUsed=%i\n\r", gps.num_sv);

  while(gps_nmea.msg_buf[i++] != ',') {              // next field: HDOP (horizontal dilution of precision)
    if (i >= gps_nmea.msg_len) {
      NMEA_PRINT("p_GPGGA() - skipping incomplete message\n\r");
      return;
    }
  }
  // we use HDOP here, as the PDOP is not in the message
  // ＨＤＯＰ(Horizontal)：包括经度和纬度等因子，称为水平（平面）位置精度因子
  // PDOP (Positional)：包括经度,纬度和高程等因子，称为三维（空间）位置精度因子
  float hdop = strtof(&gps_nmea.msg_buf[i], &endptr);
  gps.pdop = hdop * 100;

  while(gps_nmea.msg_buf[i++] != ',') {              // next field: altitude
    if (i >= gps_nmea.msg_len) {
      NMEA_PRINT("p_GPGGA() - skipping incomplete message\n\r");
      return;
    }
  }
  // get altitude (in meters) above geoid (MSL)   获得大地水平面上大的高度
  // lla_f.alt should actuall be height above ellipsoid,  lla_f.alt只是地平面上的高度
  // but since we don't get that, use hmsl instead  但是如果我们获取不到它，用ｈｍｓｌ代替
  lla_f.alt = strtof(&gps_nmea.msg_buf[i], &endptr);
  gps.hmsl = lla_f.alt * 1000;
  gps.lla_pos.alt = gps.hmsl;
  NMEA_PRINT("p_GPGGA() - gps_alt=%i\n\r", gps.hmsl);

  while(gps_nmea.msg_buf[i++] != ',') {              // next field: altitude units, always 'M'
    if (i >= gps_nmea.msg_len)
      return;
  }
  while(gps_nmea.msg_buf[i++] != ',') {              // next field: geoid seperation
    if (i >= gps_nmea.msg_len)
      return;
  }
  while(gps_nmea.msg_buf[i++] != ',') {              // next field: seperation units
    if (i >= gps_nmea.msg_len)
      return;
  }
  while(gps_nmea.msg_buf[i++] != ',') {              // next field: DGPS age
    if (i >= gps_nmea.msg_len)
      return;
  }
  while(gps_nmea.msg_buf[i++] != ',') {              // next field: DGPS station ID
    if (i >= gps_nmea.msg_len)
      return;
  }
  //while(gps_nmea.msg_buf[i++] != '*');              // next field: checksum

#if GPS_USE_LATLONG
  /* convert to utm */   //转换为ｕｔｍ时间
  struct UtmCoor_f utm_f;
  utm_f.zone = nav_utm_zone0;
  utm_of_lla_f(&utm_f, &lla_f);

  /* copy results of utm conversion */  //复制ｕｔｍ转化的结果
  gps.utm_pos.east = utm_f.east*100;
  gps.utm_pos.north = utm_f.north*100;
  gps.utm_pos.alt = gps.lla_pos.alt;
  gps.utm_pos.zone = nav_utm_zone0;
#endif

  /* convert to ECEF */  //转换到ｅｃｅｆ坐标
  struct EcefCoor_f ecef_f;
  ecef_of_lla_f(&ecef_f, &lla_f);
  gps.ecef_pos.x = ecef_f.x * 100;
  gps.ecef_pos.y = ecef_f.y * 100;
  gps.ecef_pos.z = ecef_f.z * 100;
}

/**
 * parse_nmea_char() has a complete line.
 * Find out what type of message it is and
 * hand it to the parser for that type.
 */
//parse_nmea_char() 函数是一个完整的流水线。本函数要找出信息的类型并到制定的解析函数解析它。
void nmea_parse_msg( void ) {

  if(gps_nmea.msg_len > 5 && !strncmp(gps_nmea.msg_buf , "GPRMC", 5)) {
    gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
    NMEA_PRINT("parsing RMC: \"%s\" \n\r",gps_nmea.msg_buf);
    NMEA_PRINT("RMC");
    parse_nmea_GPRMC();
  }
  else {
    if(gps_nmea.msg_len > 5 && !strncmp(gps_nmea.msg_buf , "GPGGA", 5)) {
      gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
      NMEA_PRINT("parse_gps_msg() - parsing GGA gps-message \"%s\" \n\r",gps_nmea.msg_buf);
      NMEA_PRINT("GGA");
      parse_nmea_GPGGA();
    }
    else {
      if(gps_nmea.msg_len > 5 && !strncmp(gps_nmea.msg_buf , "GPGSA", 5)) {
        gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
        NMEA_PRINT("GSA: \"%s\" \n\r",gps_nmea.msg_buf);
        NMEA_PRINT("GSA");
        parse_nmea_GPGSA();
      } else {
        gps_nmea.msg_buf[gps_nmea.msg_len] = 0;
        NMEA_PRINT("ignoring: len=%i \n\r \"%s\" \n\r", gps_nmea.msg_len, gps_nmea.msg_buf);
      }
    }
  }

  // reset message-buffer
  gps_nmea.msg_len = 0;
}


/**
 * This is the actual parser.
 * It reads one character at a time
 * setting gps_nmea.msg_available to TRUE
 * after a full line.
 */
// 这是一个实际的解析，它每次只读一个字符，当整包数据读取完毕之后给gps_nmea.msg_available赋值为ＴＲＵＥ
void nmea_parse_char( uint8_t c ) {
  //reject empty lines  不要空数据包
  if (gps_nmea.msg_len == 0) {
    if (c == '\r' || c == '\n' || c == '$')
      return;
  }

  // fill the buffer, unless it's full  将ｇｐｓ发过来的所有数据用gps_nmea.msg_buf数组全部接收
  if (gps_nmea.msg_len < NMEA_MAXLEN - 1) {

    // messages end with a linefeed   数据包最后的终结符为‘\ｒ’和‘\n’
    //AD: TRUNK:       if (c == '\r' || c == '\n')
    if (c == '\r' || c == '\n') {
      gps_nmea.msg_available = TRUE;
    } else {
      gps_nmea.msg_buf[gps_nmea.msg_len] = c;
      gps_nmea.msg_len ++;
    }
  }

  if (gps_nmea.msg_len >= NMEA_MAXLEN - 1)
    gps_nmea.msg_available = TRUE;
}
