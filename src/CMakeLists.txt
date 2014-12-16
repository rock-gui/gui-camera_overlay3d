rock_library(camera_overlay3d
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(camera_overlay3d_bin Main.cpp
    DEPS camera_overlay3d)

