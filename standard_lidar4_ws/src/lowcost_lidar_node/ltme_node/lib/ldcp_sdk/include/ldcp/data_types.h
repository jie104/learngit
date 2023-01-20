#ifndef LDCP_SDK_DATA_TYPES_H_
#define LDCP_SDK_DATA_TYPES_H_

#include <vector>
#include <cstdint>
#include <memory>

namespace ldcp_sdk
{

class ScanBlock
{
public:
  class BlockData
  {
  public:
    std::vector<int> ranges;
    std::vector<int> intensities;
  };

  int block_index;
  int block_count;
  int block_length;
  unsigned int timestamp;
  std::vector<BlockData> layers;
};

#pragma pack(push, 1)
struct OobPacketHeader
{
  uint16_t signature;
  uint16_t frame_index;
  uint8_t block_index;
  uint8_t block_count;
  uint16_t block_length;
  uint32_t timestamp;
  uint16_t checksum;
  struct {
    uint16_t payload_layout : 4;
    uint16_t reserved : 12;
  } flags;
};
#pragma pack(pop)

enum OobPacketPayloadLayout {
  PAYLOAD_LAYOUT_L1E1R16I8,
  PAYLOAD_LAYOUT_L1E1R16I16
};

class ScanFrame
{
public:
  class FrameData
  {
  public:
    std::vector<int> ranges;
    std::vector<int> intensities;
  };

  unsigned int timestamp;
  std::vector<FrameData> layers;
};

}

#endif
