# AutoAim
华中科技大学狼牙战队2022年英雄视觉代码

# Dependence

* Ceres
* Eigen
* OpenCV
* Sophus
* jsoncpp

# Build

```sh
mkdir build
cmake ..
make -j4
./AutoAim
```

# 功能清单

* init.json配置文件初始化
* c++11 thread多线程 + 条件变量、互斥锁同步线程
* 神经网络分类器、HOG+SVM分类器
* 卡尔曼滤波可视化调参
* 前哨站模式（瞄准圆心、计算下一块装甲到达时间）
* glog日志
