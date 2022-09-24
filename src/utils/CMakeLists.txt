#find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
	    /usr/local/include/glog
)

add_library(utils
        src/Config.cpp
        src/Log.cpp
        src/Params.cpp
        src/Thread.cpp
        )

target_link_libraries(utils
        #-lglog
	    /usr/local/lib/libglog.so
        -ljsoncpp
        ${OpenCV_LIBS}
        )
