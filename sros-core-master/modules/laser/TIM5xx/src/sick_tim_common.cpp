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
 *  Created on: 24.05.2012
 *
 *      Authors:
 *         Jochen Sprickerhof <jochen@sprickerhof.de>
 *         Martin Günther <mguenthe@uos.de>
 *
 * Based on the TiM communication example by SICK AG.
 *
 */

#include <sick_tim/sick_tim_common.h>

#include <cstdio>
#include <cstring>
#include "unistd.h"
namespace sick_tim
{

SickTimCommon::SickTimCommon(AbstractParser *parser) :
        expectedFrequency_(15.0), parser_(parser), receiveBuffer(nullptr)
// FIXME All Tims have 15Hz?
{
  buffer_length = 10000;
  receiveBuffer = new unsigned char[buffer_length];
  // datagram publisher (only for debug)

  // scan publisher
//  pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1000);

}

int SickTimCommon::stop_scanner()
{
  /*
   * Stop streaming measurements
   */
  const char requestScanData0[] = {"\x02sEN LMDscandata 0\x03\0\n"};
  int result = sendSOPASCommand(requestScanData0, NULL);
  if (result != 0)
    // use printf because we cannot use ROS_ERROR from the destructor
    printf("\nSOPAS - Error stopping streaming scan data!\n");
  else
    printf("\nSOPAS - Stopped streaming scan data.\n");

  return result;
}

bool SickTimCommon::rebootScanner()
{
  /*
   * Set Maintenance access mode to allow reboot to be sent
   */
  std::vector<unsigned char> access_reply;
  int result = sendSOPASCommand("\x02sMN SetAccessMode 03 F4724744\x03\0", &access_reply);
  if (result != 0)
  {
    printf("SOPAS - Error setting access mode\n");
    return false;
  }
  std::string access_reply_str = replyToString(access_reply);
  if (access_reply_str != "sAN SetAccessMode 1")
  {
    printf("SOPAS - Error setting access mode, unexpected response : %s\n" ,access_reply_str.c_str());
    return false;
  }

  /*
   * Send reboot command
   */
  std::vector<unsigned char> reboot_reply;
  result = sendSOPASCommand("\x02sMN mSCreboot\x03\0", &reboot_reply);
  if (result != 0)
  {
    printf("SOPAS - Error rebooting scanner\n");
    return false;
  }
  std::string reboot_reply_str = replyToString(reboot_reply);
  if (reboot_reply_str != "sAN mSCreboot")
  {
    printf("SOPAS - Error rebooting scanner, unexpected response : %s\n",reboot_reply_str.c_str());
    return false;
  }

  printf("SOPAS - Rebooted scanner\n");
  // Wait a few seconds after rebooting
  sleep(15);

  return true;
}

SickTimCommon::~SickTimCommon()
{
  if(receiveBuffer) {
    delete[] receiveBuffer;
    receiveBuffer = nullptr;
  }
  printf("sick_tim driver exiting.\n");
}


int SickTimCommon::init()
{
  int result = init_device();
  if(result != 0) {
    printf("Failed to init device: %d\n", result);
    return result;
  }
  result = init_scanner();
  if(result != 0) {
      printf("Failed to init scanner: %d\n", result);
  }
  return result;
}

int SickTimCommon::init_scanner()
{
  /*
   * Read the SOPAS variable 'DeviceIdent' by index.
   */
  const char requestDeviceIdent[] = "\x02sRI0\x03\0";
  std::vector<unsigned char> identReply;
  int result = sendSOPASCommand(requestDeviceIdent, &identReply);
  if (result != 0) {
    printf("SOPAS - Error reading variable 'DeviceIdent'.\n");
  }

  /*
   * Read the SOPAS variable 'SerialNumber' by name.
   */
  const char requestSerialNumber[] = "\x02sRN SerialNumber\x03\0";
  std::vector<unsigned char> serialReply;
  result = sendSOPASCommand(requestSerialNumber, &serialReply);
  if (result != 0) {
    printf("SOPAS - Error reading variable 'SerialNumber'.\n");
  }

  // set hardware ID based on DeviceIdent and SerialNumber
  std::string identStr = replyToString(identReply);
  std::string serialStr = replyToString(serialReply);
  if (!isCompatibleDevice(identStr))
    return ExitFatal;

  /*
   * Read the SOPAS variable 'FirmwareVersion' by name.
   */
  const char requestFirmwareVersion[] = {"\x02sRN FirmwareVersion\x03\0"};
  result = sendSOPASCommand(requestFirmwareVersion, NULL);
  if (result != 0)
  {
    printf("SOPAS - Error reading variable 'FirmwareVersion'.\n");
  }

  /*
   * Read Device State
   */
  const char requestDeviceState[] = {"\x02sRN SCdevicestate\x03\0"};
  std::vector<unsigned char> deviceStateReply;
  result = sendSOPASCommand(requestDeviceState, &deviceStateReply);
  if (result != 0)
  {
    printf("SOPAS - Error reading variable 'devicestate'.\n");
  }
  std::string deviceStateReplyStr = replyToString(deviceStateReply);

  /*
   * Process device state, 0=Busy, 1=Ready, 2=Error
   * If configuration parameter is set, try resetting device in error state
   */
  if (deviceStateReplyStr == "sRA SCdevicestate 0")
  {
    printf("Laser is busy\n");
  }
  else if (deviceStateReplyStr == "sRA SCdevicestate 1")
  {
    printf("Laser is ready\n");
  }
  else if (deviceStateReplyStr == "sRA SCdevicestate 2") {
    printf("Laser reports error state :%s",deviceStateReplyStr.c_str());
    if (config_.auto_reboot) {
      rebootScanner();
    }
  }
  else {
    printf("Laser reports unknown devicestate  \n");
  }

  /*
   * Start streaming 'LMDscandata'.
   */
  const char requestScanData[] = {"\x02sEN LMDscandata 1\x03\0"};
  result = sendSOPASCommand(requestScanData, NULL);
  if (result != 0) {
    printf("SOPAS - Error starting to stream 'LMDscandata'.\n");
    return ExitError;
  }

  return ExitSuccess;
}

std::string sick_tim::SickTimCommon::replyToString(const std::vector<unsigned char> &reply)
{
  std::string reply_str;
  for (std::vector<unsigned char>::const_iterator it = reply.begin(); it != reply.end(); it++)
  {
    if (*it > 13) // filter control characters for display
    {
      reply_str.push_back(*it);
    }
  }
  return reply_str;
}

bool sick_tim::SickTimCommon::isCompatibleDevice(const std::string identStr) const
{
  char device_string[7];
  int version_major = -1;
  int version_minor = -1;

  if (sscanf(identStr.c_str(), "sRA 0 6 %6s E V%d.%d", device_string,
             &version_major, &version_minor) == 3
      && strncmp("TiM3", device_string, 4) == 0
      && version_major >= 2 && version_minor >= 50) {
    printf("This scanner model/firmware combination does not support ranging output!\n");
    printf("Supported scanners: TiM5xx: all firmware versions; TiM3xx: firmware versions < V2.50.\n");
    printf("This is a %s, firmware version %d.%d\n", device_string, version_major, version_minor);
    return false;
  }
  return true;
}

int SickTimCommon::loopOnce(sros::core::LaserScan_ptr& msg)
{
  int actual_length = 0;
  static unsigned int iteration_count = 0;

  int result = get_datagram(receiveBuffer, buffer_length, &actual_length);
  if (result != 0)
  {
      printf("Read Error when getting datagram: %i.\n", result);
      return ExitError; // return failure to exit node
  }
  if(actual_length <= 0)
      return ExitSuccess; // return success to continue looping

  // ----- if requested, skip frames
  if (iteration_count++ % (config_.skip + 1) != 0)
    return ExitSuccess;
  /*
   * datagrams are enclosed in <STX> (0x02), <ETX> (0x03) pairs
   */
  char* buffer_pos = (char*)receiveBuffer;
  char *dstart, *dend;
  while( (dstart = strchr(buffer_pos, 0x02)) && (dend = strchr(dstart + 1, 0x03)) )
  {
    size_t dlength = dend - dstart;
    *dend = '\0';
    dstart++;
    int success = parser_->parse_datagram(dstart, dlength, config_, msg);
    if (success == ExitSuccess)
//      diagnosticPub_->publish(msg);
    buffer_pos = dend + 1;
  }

  return ExitSuccess; // return success to continue looping
}

void SickTimCommon::check_angle_range(SickTimConfig &conf)
{
  if (conf.min_ang > conf.max_ang) {
    printf("Minimum angle must be greater than maximum angle. Adjusting min_ang.\n");
    conf.min_ang = conf.max_ang;
  }
}

void SickTimCommon::update_config(sick_tim::SickTimConfig &new_config, uint32_t level)
{
  check_angle_range(new_config);
  config_ = new_config;
}

} /* namespace sick_tim */
