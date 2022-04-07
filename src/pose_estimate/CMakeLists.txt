#find_package()

include_directories(
        include
        ../utils/include
        /usr/include/eigen3
        ${Sophus_INCLUDE_DIRS}
        )

add_library(pose_estimate
        src/PoseSolver.cpp
        src/Predictor.cpp
        src/Kalman.cpp
        )

target_link_libraries(pose_estimate
        utils
        /usr/local/lib/libSophus.so
        )
