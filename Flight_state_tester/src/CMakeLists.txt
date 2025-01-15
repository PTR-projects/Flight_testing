add_executable(Banana_Ballistics main.cpp) # The main executable
#add_library(Project-Name-lib hello_world.cpp factorial.cpp) # A library for tests

#SET_TARGET_PROPERTIES(Project-Name-lib PROPERTIES PREFIX "") # Remove the lib prefix

#target_link_libraries(Project-Name Project-Name-lib) # Link our sources to the executable

install(Banana_Ballistics DESTINATION bin)
