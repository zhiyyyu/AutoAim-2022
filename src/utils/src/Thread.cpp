//
// Created by zhiyu on 2021/8/21.
//

#include "Thread.h"

namespace ly{
    bool Thread::image_is_update;

    condition_variable Thread::cond_is_update;

    mutex Thread::mtx_image;
    mutex Thread::mtx_video;
}