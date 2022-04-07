//
// Created by zhiyu on 2021/8/21.
//

#ifndef AUTOAIM_THREAD_H
#define AUTOAIM_THREAD_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

using namespace std;

namespace ly
{

    class Thread
    {
    public:
        static bool image_is_update;
 
        static condition_variable cond_is_update;
        static condition_variable cond_not_update;

        static mutex mtx_image;
        static mutex mtx_video;
    };
}

#endif //AUTOAIM_THREAD_H
