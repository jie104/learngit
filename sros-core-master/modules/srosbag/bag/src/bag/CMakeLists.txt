
# set(Boost_NO_SYSTEM_PATHS TRUE) 
# if (Boost_NO_SYSTEM_PATHS)
#   set(BOOST_ROOT "/usr/local/boost_1_68")
#   set(BOOST_INCLUDE_DIRS "${BOOST_ROOT}/include")
#   set(BOOST_LIBRARY_DIRS "${BOOST_ROOT}/lib")
# endif (Boost_NO_SYSTEM_PATHS)

# find_package(Boost 1.68 EXACT REQUIRED COMPONENTS
#             filesystem
#             serialization
#             iostreams
#             )

find_package(Boost REQUIRED COMPONENTS
            filesystem
            serialization
            iostreams
            )


message(STATUS ${Boost_VERSION})

add_library(msg_bag STATIC "")

target_sources(msg_bag
    PUBLIC
    # PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/message_bag.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../thread_pool/bag_thread_pool.cpp
)

target_include_directories(msg_bag
    
    # PRIVATE
    PUBLIC
    $<BUILD_INTERFACE:${BAG_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${BAG_INCLUDE_DIR}/bag>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${Boost_INCLUDE_DIR}>    
)

target_link_libraries(msg_bag
  PUBLIC
    Boost::filesystem
    Boost::serialization
    Boost::iostreams
    #   PRIVATE
    rt
    my_factory
    products

)

add_library(play_window SHARED "")

target_sources(play_window
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/play_window.cpp
)

target_include_directories(play_window
    PUBLIC
    $<BUILD_INTERFACE:${INCLUDE_DIR}/bag>
    $<BUILD_INTERFACE:${Boost_INCLUDE_DIR}>
)

target_link_libraries(play_window
    PUBLIC
    rt
)

set(PLAY_WINDOW_PATH ${PROJECT_BINARY_DIR})
configure_file("${INCLUDE_DIR}/bag/config.h.in" "${PROJECT_BINARY_DIR}/play_window/config.h")

add_executable(terminal_play 
    ${CMAKE_CURRENT_LIST_DIR}/terminal_play.cpp
)


target_include_directories(terminal_play
    PUBLIC

    $<BUILD_INTERFACE:${INCLUDE_DIR}/bag>
)

target_link_libraries(terminal_play
    PUBLIC
    play_window
)

set_target_properties(terminal_play PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})



