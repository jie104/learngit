
set(SUB_DETECTOR_NAME libhandcart_detector)
set(SUB_DETECTOR_DESC "Perception library handcart detect module.")

add_library(${SUB_DETECTOR_NAME} STATIC
    cube.cpp
    handcart_detector.cpp
    handcart_target.cpp
    ../base/detector.cpp
    ../base/cluster.cpp
    ../../common/algorithm/euclidean_cluster_extraction.cpp
    ../../common/algorithm/estimated_normal.cpp
    ../../common/algorithm/fitting_plane.cpp
    ../../common/algorithm/statistical_outlier_removal.cpp
    ../../../common/log.cpp
    )

set_target_properties(${SUB_DETECTOR_NAME}
    PROPERTIES VERSION ${PERCEPTION_VERSION}
    SOVERSION ${ver_major}.${ver_minor}
    )
