//
// Created by lhx on 17-12-18.
//

#include "network_monitor.h"

#include "core/util/time.h"

#include <fstream>
#include <sstream>

namespace monitor {

NetworkMonitor::NetworkMonitor()
        : snap_cnt_(0) {

}

NetworkMonitor::~NetworkMonitor() {

}

void NetworkMonitor::snapshot() {
    pre_data_ = cur_data_;

    readAdatperStat(cur_data_);

    auto pre_ts = last_snapshot_timestamp_;
    auto cur_ts = sros::core::util::get_time_in_ms();

    auto time_interval_ms = cur_ts - pre_ts;

    if (time_interval_ms == 0) {
        return;
    }

    last_snapshot_timestamp_ = cur_ts;
    snap_cnt_ += 1;

    if (snap_cnt_ < 2) {
        return;
    }

    for (int i = 0; i < cur_data_.size(); i++) {
        auto pre_item = pre_data_[i];
        auto cur_item = cur_data_[i];

        if (pre_item.name != cur_item.name) {
            continue;
        }

        auto adapter = adapter_stats_map_[cur_item.name];

        adapter.name = cur_item.name;
        adapter.rx_bps = (cur_item.values[A_RX_BYTES] == 0) ? 0 :
                         (cur_item.values[A_RX_BYTES] - pre_item.values[A_RX_BYTES]) * 1000 / time_interval_ms;

        adapter.rx_pps = (cur_item.values[A_RX_PACKETS] == 0) ? 0 :
                         (cur_item.values[A_RX_PACKETS] - pre_item.values[A_RX_PACKETS]) * 1000 / time_interval_ms;

        adapter.tx_bps = (cur_item.values[A_TX_BYTES] == 0) ? 0 :
                         (cur_item.values[A_TX_BYTES] - pre_item.values[A_TX_BYTES]) * 1000 / time_interval_ms;

        adapter.tx_pps = (cur_item.values[A_TX_PACKETS] == 0) ? 0 :
                         (cur_item.values[A_TX_PACKETS] - pre_item.values[A_TX_PACKETS]) * 1000 / time_interval_ms;

        adapter_stats_map_[cur_item.name] = adapter;
    }
}

AdapterStat NetworkMonitor::getAdapterStat(const std::string &name) {
    return adapter_stats_map_[name];
}

void NetworkMonitor::readAdatperStat(std::vector<AdapterData> &entries) {
    entries.clear();

    std::ifstream file("/proc/net/dev");

    std::string line;
    while (std::getline(file, line)) {

        std::istringstream ss(line);

        auto item = AdapterData();

        ss >> item.name;
        item.name.pop_back(); // 去除name最后一个字符(:)

        for (int i = 0; i < NUM_OF_ADAPTER_VALUES; ++i) {
            ss >> item.values[i];
        }

        entries.push_back(item);
    }

}



}

