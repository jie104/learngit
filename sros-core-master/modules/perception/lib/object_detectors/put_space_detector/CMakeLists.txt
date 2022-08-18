set(SUB_DETECTOR_NAME libput_space_detector)
set(SUB_DETECTOR_DESC "Perception library headup detect module.")


add_library(${SUB_DETECTOR_NAME} STATIC
    put_space_detector.cpp
    cube_pile_target.cpp
    ../base/detector.cpp
    ../base/cluster.cpp
    ../../common/algorithm/euclidean_cluster_extraction.cpp
    ../../common/algorithm/estimated_normal.cpp
    ../../common/algorithm/fitting_plane.cpp
    ../../common/algorithm/statistical_outlier_removal.cpp
    ../../../common/log.cpp
    cube.cpp
    )

target_link_libraries(${SUB_DETECTOR_NAME} )

set_target_properties(${SUB_DETECTOR_NAME}
    PROPERTIES VERSION ${PERCEPTION_VERSION}
    SOVERSION ${ver_major}.${ver_minor}
    )
