//
// Created by lhx on 17-12-18.
//

#ifndef SROS_NETWORK_MONITOR_H
#define SROS_NETWORK_MONITOR_H

#include <string>
#include <vector>

#include <memory>
#include <map>

namespace monitor {

const int NUM_OF_ADAPTER_VALUES = 16;

enum AdapterDataValueKey {
    A_RX_BYTES = 0,
    A_RX_PACKETS = 1,
    A_RX_ERRORS = 2,

    A_TX_BYTES = 8,
    A_TX_PACKETS = 9,
    A_TX_ERRORS = 10,
};

typedef struct AdapterData {
    std::string name;
    size_t values[NUM_OF_ADAPTER_VALUES];
} AdapterData;

typedef struct AdapterStat {
    std::string name;

    size_t rx_bps;
    size_t rx_pps;

    size_t tx_bps;
    size_t tx_pps;
} AdapterStat;

typedef std::vector<AdapterData> AdapterData_list;

class NetworkMonitor {
public:
    NetworkMonitor();
    ~NetworkMonitor();

    void snapshot();

    AdapterStat getAdapterStat(const std::string &name);
private:

    void readAdatperStat(std::vector<AdapterData> & entries);

    int snap_cnt_;

    AdapterData_list pre_data_;
    AdapterData_list cur_data_;

    uint64_t last_snapshot_timestamp_;

    std::map<std::string, AdapterStat> adapter_stats_map_;
};

typedef std::shared_ptr<NetworkMonitor> NetworkMonitor_ptr;

}

#endif //SROS_NETWORK_MONITOR_H
