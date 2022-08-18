
set(BAG_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)

add_subdirectory(src)

if(${NONE_SROS})
    add_subdirectory(test)
endif()