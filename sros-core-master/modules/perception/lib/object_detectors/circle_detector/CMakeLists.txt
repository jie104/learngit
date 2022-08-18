
set(SUB_DETECTOR_NAME libcircle_detector)
set(SUB_DETECTOR_DESC "Perception library circle detect module.")

add_library(${SUB_DETECTOR_NAME} STATIC
    circle_detector.cpp
    circle_target.cpp
    ../base/detector.cpp
    ../base/cluster.cpp
    ../../common/algorithm/euclidean_cluster_extraction.cpp
    ../../common/algorithm/estimated_normal.cpp
    ../../common/algorithm/fitting_plane.cpp
    ../../common/algorithm/fitting_circle.cpp
    ../../common/algorithm/statistical_outlier_removal.cpp
    ../../../common/log.cpp
    )

set_target_properties(${SUB_DETECTOR_NAME}
    PROPERTIES VERSION ${PERCEPTION_VERSION}
    SOVERSION ${ver_major}.${ver_minor}
    )
