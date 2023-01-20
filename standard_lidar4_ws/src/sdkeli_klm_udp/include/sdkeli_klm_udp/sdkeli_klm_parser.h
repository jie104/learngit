#ifndef SDKELI_KLMXX_PARSER__
#define SDKELI_KLMXX_PARSER__

#include <sdkeli_klm_udp/parser_base.h>

namespace sdkeli_klm_udp
{
class CSDKeliKlmParser : public CParserBase
{
public:
    CSDKeliKlmParser();
    virtual ~CSDKeliKlmParser();

    virtual int Parse(unsigned short *data, size_t data_length, unsigned char *info, size_t info_length, SDKeliKlmConfig &config, sensor_msgs::LaserScan &msg);

    void SetRangeMin(float minRange);
    void SetRangeMax(float maxRange);
    void SetTimeIncrement(float time);
    void SetFrameId(std::string str);

private:
    float fRangeMin;
    float fRangeMax;
    float fTimeIncrement;
    std::string fFrame_id;
};
} /*namespace sdkeli_klm_udp*/

#endif /*SDKELI_KLMXX_PARSER__*/
