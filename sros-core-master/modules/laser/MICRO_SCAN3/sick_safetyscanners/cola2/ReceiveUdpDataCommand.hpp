//
// Created by lfc on 19-10-10.
//

#ifndef SICK_SAFETYSCANNERS_RECEIVEUDPDATACOMMAND_HPP
#define SICK_SAFETYSCANNERS_RECEIVEUDPDATACOMMAND_HPP

#include "VariableCommand.h"
#include "../data_processing/ParseTypeCodeData.h"
#include "../datastructure/CommSettings.h"
#include "Cola2Session.h"
#include "glog/logging.h"

namespace sick {
namespace cola2 {
class ReceiveUdpDataCommand :public VariableCommand {
public:
    ReceiveUdpDataCommand(Cola2Session &session, sick::datastructure::ConfigData &type_code) :
            VariableCommand(session, 0x00b6), m_config_data(type_code) {
        LOG(INFO) << "id :" << getVariableIndex();
        m_measurement_current_config_parser_ptr = std::make_shared<sick::data_processing::ParseMeasurementCurrentConfigData>();
    }

    bool canBeExecutedWithoutSessionID() const {
        return true;
    }

    bool processReply() {
        if (!VariableCommand::processReply()) {
            return false;
        }
        auto datas = Command::getDataVector();
        printf("\n");
        for(auto& data:datas){
            printf("%c",(char)data);
        }
        printf("\n");
        m_measurement_current_config_parser_ptr->parseTCPSequence(Command::getDataVector(), m_config_data);
        return true;
    }

private:
    std::shared_ptr<sick::data_processing::ParseMeasurementCurrentConfigData> m_measurement_current_config_parser_ptr;

    sick::datastructure::ConfigData& m_config_data;
};
}
}

#endif //SICK_SAFETYSCANNERS_RECEIVEUDPDATACOMMAND_HPP
