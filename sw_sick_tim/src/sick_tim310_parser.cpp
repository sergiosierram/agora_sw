/*
 * Copyright (C) 2013, Osnabrück University
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Osnabrück University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: 21.08.2013
 *
 *      Author: Martin Günther <mguenthe@uos.de>
 *
 */

#include <agora_sw_sick_tim/sick_tim310_parser.h>

#include <ros/ros.h>

namespace sick_tim
{

SickTim310Parser::SickTim310Parser() :
    AbstractParser()
{
}

SickTim310Parser::~SickTim310Parser()
{
}

int SickTim310Parser::parse_datagram(char* datagram, size_t datagram_length, SickTimConfig &config,
                                     sensor_msgs::LaserScan &msg)
{
  static const size_t NUM_FIELDS = 592;
  char* fields[NUM_FIELDS];
  char* cur_field;
  size_t count;

  // ----- only for debug output
  char datagram_copy[datagram_length + 1];
  strncpy(datagram_copy, datagram, datagram_length); // datagram will be changed by strtok
  datagram_copy[datagram_length] = 0;

  // ----- tokenize
  count = 0;
  cur_field = strtok(datagram, " ");

  while (cur_field != NULL)
  {
    if (count < NUM_FIELDS)
      fields[count++] = cur_field;

    // ROS_DEBUG("%zu: %s ", count, cur_field);
    cur_field = strtok(NULL, " ");
  }

  if (count < NUM_FIELDS)
  {
    ROS_WARN(
        "received less fields than expected fields (actual: %zu, expected: %zu), ignoring scan", count, NUM_FIELDS);
    ROS_WARN("are you using the correct node? (124 --> sick_tim310_1130000m01, 306 --> sick_tim551_2050001, 580 --> sick_tim310s01, 592 --> sick_tim310)");
    // ROS_DEBUG("received message was: %s", datagram_copy);
    return ExitError;
  }
  else if (count > NUM_FIELDS)
  {
    ROS_WARN("received more fields than expected (actual: %zu, expected: %zu), ignoring scan", count, NUM_FIELDS);
    ROS_WARN("are you using the correct node? (124 --> sick_tim310_1130000m01, 306 --> sick_tim551_2050001, 580 --> sick_tim310s01, 592 --> sick_tim310)");
    // ROS_DEBUG("received message was: %s", datagram_copy);
    return ExitError;
  }

  // ----- read fields into msg
  msg.header.frame_id = config.frame_id;
  ROS_DEBUG("publishing with frame_id %s", config.frame_id.c_str());

  ros::Time start_time = ros::Time::now(); // will be adjusted in the end

  // <STX> (\x02)
  // 0: Type of command (SN)
  // 1: Command (LMDscandata)
  // 2: Firmware version number (1)
  // 3: Device number (1)
  // 4: Serial number (B96518)
  // 5 + 6: Device Status (0 0 = ok, 0 1 = error)
  // 7: Telegram counter (99)
  // 8: Scan counter (9A)
  // 9: Time since startup (13C8E59)
  // 10: Time of transmission (13C9CBE)
  // 11 + 12: Input status (0 0)
  // 13 + 14: Output status (8 0)
  // 15: Reserved Byte A (0)

  // 16: Scanning Frequency (5DC)
  unsigned short scanning_freq = -1;
  sscanf(fields[16], "%hx", &scanning_freq);
  msg.scan_time = 1.0 / (scanning_freq / 100.0);
  // ROS_DEBUG("hex: %s, scanning_freq: %d, scan_time: %f", fields[16], scanning_freq, msg.scan_time);

  // 17: Measurement Frequency (36)
  unsigned short measurement_freq = 36;
  //sscanf(fields[17], "%hx", &measurement_freq);
  msg.time_increment = 1.0 / (measurement_freq * 100.0);
  // ROS_DEBUG("measurement_freq: %d, time_increment: %f", measurement_freq, msg.time_increment);

  // 18: Number of encoders (0)
  // 19: Number of 16 bit channels (1)
  // 22: Measured data contents (DIST1)

  // 23: Scaling factor (3F800000)
  // ignored for now (is always 1.0):
//      unsigned int scaling_factor_int = -1;
//      sscanf(fields[21], "%x", &scaling_factor_int);
//
//      float scaling_factor = reinterpret_cast<float&>(scaling_factor_int);
//      // ROS_DEBUG("hex: %s, scaling_factor_int: %d, scaling_factor: %f", fields[21], scaling_factor_int, scaling_factor);

  // 24: Scaling offset (00000000) -- always 0
  // 25: Starting angle (FFF92230)
  int starting_angle = -1;
  sscanf(fields[25], "%x", &starting_angle);
  msg.angle_min = (starting_angle / 10000.0) / 180.0 * M_PI - M_PI / 2;
  // ROS_DEBUG("starting_angle: %d, angle_min: %f", starting_angle, msg.angle_min);

  // 26: Angular step width (2710)
  unsigned short angular_step_width = -1;
  sscanf(fields[26], "%hx", &angular_step_width);
  msg.angle_increment = (angular_step_width / 10000.0) / 180.0 * M_PI;
  msg.angle_max = msg.angle_min + 270.0 * msg.angle_increment;

  // adjust angle_min to min_ang config param
  int index_min = 0;
  while (msg.angle_min + msg.angle_increment < config.min_ang)
  {
    msg.angle_min += msg.angle_increment;
    index_min++;
  }

  // adjust angle_max to max_ang config param
  int index_max = 270;
  while (msg.angle_max - msg.angle_increment > config.max_ang)
  {
    msg.angle_max -= msg.angle_increment;
    index_max--;
  }

  ROS_DEBUG("index_min: %d, index_max: %d", index_min, index_max);
  // ROS_DEBUG("angular_step_width: %d, angle_increment: %f, angle_max: %f", angular_step_width, msg.angle_increment, msg.angle_max);

  // 27: Number of data (10F)

  // 28..298: Data_1 .. Data_n
  msg.ranges.resize(index_max - index_min + 1);
  for (int j = index_min; j <= index_max; ++j)
  {
    unsigned short range;
    sscanf(fields[j + 28], "%hx", &range);
    if (range == 0)
      msg.ranges[j - index_min] = std::numeric_limits<float>::infinity();
    else
      msg.ranges[j - index_min] = range / 1000.0;
  }

  // ---297: Number of 8 bit channels (1)---
  // 299: Measured data contents (RSSI1)
  // 300: Scaling factor (3F800000)
  // 301: Scaling offset (00000000)
  // 302: Starting angle (FFF92230)
  // 303: Angular step width (2710)
  // 304: Number of data (10F)
  // 305..575: Data_1 .. Data_n
  if (config.intensity)
  {
    msg.intensities.resize(index_max - index_min + 1);
    for (int j = index_min; j <= index_max; ++j)
    {
      unsigned short intensity;
      sscanf(fields[j + 305], "%hx", &intensity);
      msg.intensities[j - index_min] = intensity;
    }
  }

  // 576: Position (0)
  // 577: Name (0)
  // 578: Comment (0)
  // 579: Time information (0)
  // 579: Event information (0)
  // <ETX> (\x03)

  msg.range_min = 0.05;
  msg.range_max = 4.0;

  // ----- adjust start time
  // - last scan point = now  ==>  first scan point = now - 271 * time increment
  double start_time_adjusted = start_time.toSec()
            - 271 * msg.time_increment              // shift backward to time of first scan point
            + index_min * msg.time_increment        // shift forward to time of first published scan point
            + config.time_offset;                   // add time offset (usually negative) to account for USB latency etc.
  if (start_time_adjusted >= 0.0)   // ensure that ros::Time is not negative (otherwise runtime error)
  {
    msg.header.stamp.fromSec(start_time_adjusted);
  } else {
    ROS_WARN("ROS time is 0! Did you set the parameter use_sim_time to true?");
  }


  return ExitSuccess;
}

} /* namespace sick_tim */
