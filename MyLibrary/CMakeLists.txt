set(MYNOID_HEADERS
    "Trajectory_planner.h"
    "Footprint.h"
    "Footprint_planner.h"
    "IK.h"
    "Sensing_Unit.h"
    "Stabilizer.h"
    )

set(MYNOID_SOURCES
     "Trajectory_planner.cpp"
     "Footprint.cpp"
     "Footprint_planner.cpp"
     "IK.cpp"
     "Sensing_Unit.cpp"
     "Stabilizer.cpp"
     )

choreonoid_add_library(mynoid_lib STATIC ${MYNOID_HEADERS} ${MYNOID_SOURCES})

target_link_libraries(mynoid_lib CnoidBody)
