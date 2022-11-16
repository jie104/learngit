#-------------------------------------------------
#
# Project created by QtCreator 2022-11-13T23:42:12
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = LidarDetect
TEMPLATE = app

#添加ros头文件路径和动态链接库
INCLUDEPATH += /opt/ros/noetic/include
DEPENDPATH += /opt/ros/noetic/include
LIBS += -L/opt/ros/noetic/lib -lroscpp -lroslib -lrosconsole -lroscpp_serialization -lrostime

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

SOURCES += \
    http_command_interface.cpp \
        main.cpp \
    r2000_driver.cpp \
    r2000_node.cpp \
    scan_data_receiver.cpp \
        widget.cpp \
#    standard_lidar_protocol/UST10LX/urg_c_wrapper.cpp \
#    standard_lidar_protocol/UST10LX/urg_c/src/urg_connection.c \
#    standard_lidar_protocol/UST10LX/urg_c/src/urg_debug.c \
#    standard_lidar_protocol/UST10LX/urg_c/src/urg_ring_buffer.c \
#    standard_lidar_protocol/UST10LX/urg_c/src/urg_sensor.c \
#    standard_lidar_protocol/UST10LX/urg_c/src/urg_serial_linux.c \
#    standard_lidar_protocol/UST10LX/urg_c/src/urg_serial_utils_linux.c \
#    standard_lidar_protocol/UST10LX/urg_c/src/urg_tcpclient.c \
#    standard_lidar_protocol/UST10LX/urg_c/src/urg_time.c \
#    standard_lidar_protocol/UST10LX/urg_c/src/urg_utils.c

HEADERS += \
    pepperl_fuchs_r2000/http_command_interface.h \
    pepperl_fuchs_r2000/packet_structure.h \
    pepperl_fuchs_r2000/protocol_info.h \
    pepperl_fuchs_r2000/r2000_driver.h \
    pepperl_fuchs_r2000/r2000_node.h \
    pepperl_fuchs_r2000/scan_data_receiver.h \
        widget.h \
#    ../workspace/obstacle_and_filter/standard_lidar4_ws/src/standard_lidar_driver/src/standard_lidar_protocol/oradar_laser_protocol.hpp \
#    ../workspace/obstacle_and_filter/standard_lidar4_ws/src/standard_lidar_driver/src/standard_lidar_protocol/laser_protocol_interface.hpp \
#    include/oradar_laser_protocol.hpp \
#    include/oradar_laser_protocol.hpp \
#    standard_lidar_protocol/UST10LX/urg_c/include/urg_c/urg_connection.h \
#    standard_lidar_protocol/UST10LX/urg_c/include/urg_c/urg_debug.h \
#    standard_lidar_protocol/UST10LX/urg_c/include/urg_c/urg_detect_os.h \
#    standard_lidar_protocol/UST10LX/urg_c/include/urg_c/urg_errno.h \
#    standard_lidar_protocol/UST10LX/urg_c/include/urg_c/urg_ring_buffer.h \
#    standard_lidar_protocol/UST10LX/urg_c/include/urg_c/urg_sensor.h \
#    standard_lidar_protocol/UST10LX/urg_c/include/urg_c/urg_serial.h \
#    standard_lidar_protocol/UST10LX/urg_c/include/urg_c/urg_serial_utils.h \
#    standard_lidar_protocol/UST10LX/urg_c/include/urg_c/urg_tcpclient.h \
#    standard_lidar_protocol/UST10LX/urg_c/include/urg_c/urg_time.h \
#    standard_lidar_protocol/UST10LX/urg_c/include/urg_c/urg_utils.h \
    standard_lidar_protocol/UST10LX/laser_scan_msg.hpp \
    standard_lidar_protocol/UST10LX/urg_c_wrapper.h \
    standard_lidar_protocol/data_receiver_interface.hpp \
    standard_lidar_protocol/keli_laser_protocol.hpp \
    standard_lidar_protocol/keli_obstacle_laser_protocol.hpp \
    standard_lidar_protocol/laser_protocol_interface.hpp \
    standard_lidar_protocol/laser_scan_msg.hpp \
    standard_lidar_protocol/oradar_laser_protocol.hpp \
    standard_lidar_protocol/siminics_laser_protocol.hpp \
    standard_lidar_protocol/synchronize_time_manager.hpp \
    standard_lidar_protocol/tcp_scan_data_receiver.hpp \
    standard_lidar_protocol/udp_scan_data_receiver.hpp \
    standard_lidar_protocol/wanji_laser_protocol.hpp \
    standard_lidar_protocol/wanji_laser_protocol_udp.hpp \
    lidar_detect/base_data_type.hpp \
    lidar_detect/lidar_interpara_calibra.hpp \
    lidar_detect/recognize.hpp \
    lidar_detect/solveCloudPoint.hpp

FORMS += \
        widget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

#添加boost库
LIBS +=/usr/lib/x86_64-linux-gnu/libboost_thread.a
LIBS +=/usr/lib/x86_64-linux-gnu/libboost_system.a
DEFINES +=BOOST_USE_LIB

#添加ros相关
INCLUDEPATH += /opt/ros/noetic

/include
DEPENDPATH += /opt/ros/noetic/lib
LIBS += -L/opt/ros/noetic/lib
LIBS += /opt/ros/noetic/lib/librosbag.so
LIBS += /opt/ros/noetic/lib/libroscpp.so


LIBS += /opt/ros/noetic/lib/libroslib.so
LIBS += /opt/ros/noetic/lib/libroslz4.so
LIBS += /opt/ros/noetic/lib/librostime.so
LIBS += /opt/ros/noetic/lib/libroscpp_serialization.so
LIBS += /opt/ros/noetic/lib/librospack.so
LIBS += /opt/ros/noetic/lib/libcpp_common.so
LIBS += /opt/ros/noetic/lib/librosbag_storage.so
LIBS += /opt/ros/noetic/lib/librosconsole.so
LIBS += /opt/ros/noetic/lib/libxmlrpcpp.so
LIBS += /opt/ros/noetic/lib/librosconsole_backend_interface.so
LIBS += /opt/ros/noetic/lib/librosconsole_log4cxx.so
