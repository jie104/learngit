/*********************************************************************
 *
 * Software License Agreement ()
 *
 *  Copyright (c) 2020, HINS, Inc.
 *  All rights reserved.
 *
 * Author: Hu Liankui
 * Create Date: 8/9/2020
 *********************************************************************/

/**
 * @brief 兴颂激光雷达驱动
 */

#pragma once
#include "ros/ros.h"
#include "hins/laser_data_receiver.h"
#include <set>
// #include"srv/hins_srv.srv"
namespace hins
{

  class XingSongDriver;
  using XingSongDriverHdr = std::shared_ptr<XingSongDriver>;

  class XingSongDriver
  {
  public:
    XingSongDriver(const ConnectionAddress &conn_info, XingSongLaserParam param);
    XingSongDriver(const ConnectionAddress &conn_info, XingSongLaserParam param, ShadowsFilterParam shadows_filter_param, DisturbFilterParam disturb_filter_param);
    ~XingSongDriver();

    bool StartCapturingTCP();

    ScanData GetFullScan();
    
    bool Connect()
    {
      return data_receiver_ptr_->Connect();
    }

    bool IsConnected()
    {
      return data_receiver_ptr_->IsConnected();
    }

    void Disconnect()
    {
      data_receiver_ptr_->Disconnect();
    }

    std::array<unsigned char, 12> GenerateParamCommand(XingSongLaserParam param)
    {
      return data_receiver_ptr_->GenerateParamCommand(param);
    }

    int SyncWrite(const std::array<unsigned char, 12> command)
    {
      return data_receiver_ptr_->SyncWrite(command);
    }

    short int GetLaserSteadyTime()
    {
      return data_receiver_ptr_->GetLaserSteadyTime();
    }

    int SendAreaCommand()
    {
      return data_receiver_ptr_->SyncWrite(get_area_data_command_);
    }

    void SetAreaCommand(HSGetAreaDataPackage command);
    int GetBlockValue()
    {
      return data_receiver_ptr_->GetBlockValue();
    }

    bool GetBlock1Value()
    {
        return block1_;
    }

    bool GetBlock2Value()
    {
        return block2_;
    }

    bool GetBlock3Value()
    {
        return block3_;
    }

    int GetResponseChannel()
    {
      return now_channel_;
    }

    void RefreshChannel()
    {
      now_channel_ = data_receiver_ptr_->GetResponsChannel();
    }
  private:
    void RunMain();
    void ShadowsFilterInit(int scan_num);
    void ShadowsFilter(ScanData& data, int scan_num);
    bool CheckLaserParam(XingSongLaserParam& laser_param);
    short int CRCVerify(std::array<char,18> *command, int len);
    void DisturbFilter(ScanData& scan_data, DisturbFilterParam disturb_param);
  private:
    int have_block_;
    int now_channel_;
    bool block1_;
    bool block2_;
    bool block3_;
    bool block_enable_;                        // true为发送避障帧，false为不发送
    ShadowsFilterParam shadows_filter_param_;
    ConnectionAddress conn_info_;
    XingSongLaserParam laser_param;
    LaserDataReceiverPtr data_receiver_ptr_;
    std::thread guard_thread_;
    std::vector<float> shadows_filter_threshold_min_;
    std::vector<float> shadows_filter_threshold_max_;
    set<int> shadows_del_index;
    set<int> disturb_del_index_;
    std::array<char, 18> get_area_data_command_;
    DisturbFilterParam disturb_filter_param_;

  };


}
