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


 /*解析rc数据
  * 函数调用关系为：radio_control_impl_init() 初始化
  ×                   |——>rc_joby_parse(c,*callback) 将读回来的数据存储，并确定高低字节
  ×                          |——>handle_tuple(callback)  确定正常模式和倒置模式是否匹配上
  ×                                    |——>handle_channel(callback)  查询频道数目
  */

#include "stdio.h"
#include "subsystems/radio_control.h"

static struct rc_joby_parser_state parser;
static const int16_t rc_joby_signs[RADIO_CONTROL_NB_CHANNEL] = RC_JOBY_SIGNS;


static void handle_channel(void (* callback)(void))
{
 if (parser.parser_normal_buf == RC_JOBY_MAGIC_START) {
    // got start channel, look for channel 0 next   获得初始频道，接下来找频道0
    parser.current_channel = 0;
  } else if (parser.current_channel == -1) {
    // looking for start channel byte but didn't get it, reset
     //没找到初始频道，复位
    parser.current_byte = READING_HIGH_BYTE;
    parser.current_inverted = READING_NORMAL;
  } else {
    // valid channel, store and look for next
     //有效频道，存储并寻找下一个
    radio_control.values[parser.current_channel] = rc_joby_signs[parser.current_channel] * parser.parser_normal_buf;
    parser.current_channel++;
    if (parser.current_channel == RADIO_CONTROL_NB_CHANNEL) {
      // all channels read, reset parser and handle message
      //所有的频道都读取，复位，解析并处理信息
      parser.current_channel = -1;
      radio_control.frame_cpt++;
      radio_control.status = RC_OK;
      radio_control.time_since_last_frame = 0;
      if (callback != NULL)
    callback();
    }
  }
}

static void handle_tuple(void (* callback)(void))
{
  if (parser.current_inverted == READING_NORMAL) {    //正常
    parser.parser_normal_buf = ((parser.high_byte_buf << 8) | parser.low_byte_buf);
    parser.current_inverted = READING_INVERTED;
  } else if (parser.current_inverted == READING_INVERTED) {    //倒置
    parser.parser_inverted_buf = ((parser.high_byte_buf << 8) | parser.low_byte_buf);
    parser.current_inverted = READING_NORMAL;
    if (parser.parser_normal_buf == ~parser.parser_inverted_buf) {    //正常和倒置匹配
      handle_channel(callback);
    } else {
      // normal didn't match inverted, error, reset   如果没有匹配上，错误，重置
      parser.current_inverted = READING_NORMAL;
      parser.current_byte = READING_HIGH_BYTE;
      parser.current_channel = -1;
      parser.error_counter++;
    }
  }
}

void rc_joby_parse(int8_t c, void (* callback)(void))   //解析函数
{
  if (parser.current_byte == READING_HIGH_BYTE) {   //解析高字节
    parser.high_byte_buf = c;
    if (parser.current_channel >= 0 || parser.high_byte_buf == (RC_JOBY_MAGIC_START >> 8) || parser.current_inverted == READING_INVERTED) {
       // only advance parser state to low byte if we're not looking for a sync byte which we didn't find
        //如果我们没有找到一个同步字节，只提前解析低字节的状态。
      parser.current_byte = READING_LOW_BYTE;
    }
  } else { // READING_LOW_BYTE   解析低字节
    parser.low_byte_buf = c;
    parser.current_byte = READING_HIGH_BYTE;
    handle_tuple(callback);
  }
}

void radio_control_impl_init(void) {
  parser.current_byte = READING_HIGH_BYTE;
  parser.current_inverted = READING_NORMAL;
  parser.current_channel = -1;
}
