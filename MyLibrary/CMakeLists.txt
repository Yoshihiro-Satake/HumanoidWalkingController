set(MYNOID_HEADERS
    "Trajectory_planner.h"
    "Footprint.h"
    )

set(MYNOID_SOURCES
     "Trajectory_planner.cpp"
     "Footprint.cpp"
     )

choreonoid_add_library(mynoid_lib STATIC ${MYNOID_HEADERS} ${MYNOID_SOURCES})

target_link_libraries(mynoid_lib CnoidBody)
