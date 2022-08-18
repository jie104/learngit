
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


add_library(dmtx_lib STATIC
        dmtx.h
        config.h
        dmtxstatic.h
        dmtx.cpp
        dmtxencodescheme.cpp
        dmtxbytelist.cpp
        dmtxdecode.cpp
        dmtxencode.cpp
        dmtxdecodescheme.cpp
        dmtxencodeascii.cpp
        dmtxencodebase256.cpp
        dmtxencodec40textx12.cpp
        dmtxencodeedifact.cpp
        dmtxencodeoptimize.cpp
        dmtxencodestream.cpp
        dmtximage.cpp
        dmtxmatrix3.cpp
        dmtxmessage.cpp
        dmtxplacemod.cpp
        dmtxreedsol.cpp
        dmtxregion.cpp
        dmtxscangrid.cpp
        dmtxsymbol.cpp
        dmtxtime.cpp
        dmtxvector2.cpp
)

#target_link_libraries(dmtx_lib )

                         
