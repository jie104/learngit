cmake_minimum_required(VERSION 2.8.3)
project(micro_scan3)
set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -Wfatal-errors")
aux_source_directory(. MODULE_SRCS)
include_directories(sick_safetyscanners)

## Declare a cpp library
sr_library(module-laser_MICROSCAN3
        SRCS
            sick_safetyscanners/SickSafetyscanners.cpp
            sick_safetyscanners/cola2/ChangeCommSettingsCommand.cpp
            sick_safetyscanners/cola2/CloseSession.cpp
            sick_safetyscanners/cola2/Cola2Session.cpp
            sick_safetyscanners/cola2/Command.cpp
            sick_safetyscanners/cola2/CreateSession.cpp
            sick_safetyscanners/cola2/DeviceNameVariableCommand.cpp
            sick_safetyscanners/cola2/FieldGeometryVariableCommand.cpp
            sick_safetyscanners/cola2/FieldHeaderVariableCommand.cpp
            sick_safetyscanners/cola2/MeasurementCurrentConfigVariableCommand.cpp
            sick_safetyscanners/cola2/MeasurementPersistentConfigVariableCommand.cpp
            sick_safetyscanners/cola2/MethodCommand.cpp
            sick_safetyscanners/cola2/MonitoringCaseTableHeaderVariableCommand.cpp
            sick_safetyscanners/cola2/MonitoringCaseVariableCommand.cpp
            sick_safetyscanners/cola2/TypeCodeVariableCommand.cpp
            sick_safetyscanners/cola2/VariableCommand.cpp
            sick_safetyscanners/communication/AsyncTCPClient.cpp
            sick_safetyscanners/communication/AsyncUDPClient.cpp
            sick_safetyscanners/data_processing/ParseApplicationData.cpp
            sick_safetyscanners/data_processing/ParseData.cpp
            sick_safetyscanners/data_processing/ParseDataHeader.cpp
            sick_safetyscanners/data_processing/ParseDatagramHeader.cpp
            sick_safetyscanners/data_processing/ParseDerivedValues.cpp
            sick_safetyscanners/data_processing/ParseDeviceName.cpp
            sick_safetyscanners/data_processing/ParseFieldGeometryData.cpp
            sick_safetyscanners/data_processing/ParseFieldHeaderData.cpp
            sick_safetyscanners/data_processing/ParseGeneralSystemState.cpp
            sick_safetyscanners/data_processing/ParseIntrusionData.cpp
            sick_safetyscanners/data_processing/ParseMeasurementCurrentConfigData.cpp
            sick_safetyscanners/data_processing/ParseMeasurementData.cpp
            sick_safetyscanners/data_processing/ParseMeasurementPersistentConfigData.cpp
            sick_safetyscanners/data_processing/ParseMonitoringCaseData.cpp
            sick_safetyscanners/data_processing/ParseTCPPacket.cpp
            sick_safetyscanners/data_processing/ParseTypeCodeData.cpp
            sick_safetyscanners/data_processing/TCPPacketMerger.cpp
            sick_safetyscanners/data_processing/UDPPacketMerger.cpp
            sick_safetyscanners/datastructure/ApplicationData.cpp
            sick_safetyscanners/datastructure/ApplicationInputs.cpp
            sick_safetyscanners/datastructure/ApplicationOutputs.cpp
            sick_safetyscanners/datastructure/CommSettings.cpp
            sick_safetyscanners/datastructure/ConfigData.cpp
            sick_safetyscanners/datastructure/Data.cpp
            sick_safetyscanners/datastructure/DataHeader.cpp
            sick_safetyscanners/datastructure/DatagramHeader.cpp
            sick_safetyscanners/datastructure/DerivedValues.cpp
            sick_safetyscanners/datastructure/FieldData.cpp
            sick_safetyscanners/datastructure/GeneralSystemState.cpp
            sick_safetyscanners/datastructure/IntrusionData.cpp
            sick_safetyscanners/datastructure/IntrusionDatum.cpp
            sick_safetyscanners/datastructure/MeasurementData.cpp
            sick_safetyscanners/datastructure/MonitoringCaseData.cpp
            sick_safetyscanners/datastructure/PacketBuffer.cpp
            sick_safetyscanners/datastructure/ParsedPacketBuffer.cpp
            sick_safetyscanners/datastructure/ScanPoint.cpp
            sick_safetyscanners/datastructure/TypeCode.cpp
            micro_scan3_module.cpp
        HDRS
            micro_scan3_module.h
        )
#add_executable(micro_scan3_node micro_scan3_node.cpp)
#target_link_libraries(micro_scan3_node
#        module-laser_MICROSCAN3
#        glog
#        boost_system
#        boost_thread)