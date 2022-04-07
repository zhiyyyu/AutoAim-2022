find_package(OpenCV 4 REQUIRED)
#find_package(PythonLibs 3.8 REQUIRED)
# set(PYTHON_LIBRARY "/usr/lib/python3.5/config-3.5m-aarch64-linux-gnu/libpython3.5.so")
# set(PYTHON_INCLUDE_DIR "/usr/lib/python3.5")
include_directories(
        include
        ../utils/include
        ../driver/include
        ../pose_estimate/include
        ${PYTHON_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_library(armor_detector
        src/Detector.cpp
        src/Classifier.cpp
        src/LightBarFinder.cpp
        src/ArmorFinder.cpp
        src/Tracker.cpp
        src/Score.cpp
        )

target_link_libraries(armor_detector
        utils
        pose_estimate
        -lpthread
        ${PYTHON_LIBRARIES}
        ${OpenCV_LIBS}
)
