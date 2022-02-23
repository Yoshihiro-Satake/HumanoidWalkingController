include_directories("/home/yoshihiro/choreonoid/ext/Humanoid/MyLibrary")

add_cnoid_simple_controller(Biped_Online_Controller_test controller.cpp ${MYNOID_SOURCES} ${MYNOID_HEADERS})

target_link_libraries(Biped_Online_Controller_test mynoid_lib)
