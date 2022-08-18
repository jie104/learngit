
set(SUB_DETECTOR_NAME libdecards_detector)
set(SUB_DETECTOR_DESC "Perception library decards detect module.")

add_library(${SUB_DETECTOR_NAME} STATIC
    decards_detector.cpp
    ../card_detector/card_target.cpp
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
