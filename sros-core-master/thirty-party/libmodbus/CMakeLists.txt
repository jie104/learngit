# add sources of the "libmodbus" static library
add_library(libmodbus
        modbus.c 
        modbus.h
        modbus-data.c
        modbus-private.h
        modbus-rtu.c
        modbus-rtu.h
        modbus-rtu-private.h
        modbus-tcp.c
        modbus-tcp.h
        modbus-tcp-private.h
        modbus-version.h
        config.h
        )