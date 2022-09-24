//
// Created by zhiyu on 2022/7/30.
//

#ifndef AUTOAIM_ARRAY_H
#define AUTOAIM_ARRAY_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;

namespace ly{
    template <typename T>
    class RollingArray
    {
    public:
        RollingArray():
            _size(100), id(0), metric(0){}
        
        RollingArray(int _size):
            _size(_size), id(0), metric(0){}

        void update(T elem){
            if(array.size() < _size){            
                metric += elem;
                array.push_back(elem);
                return;
            }
            metric -= array[id];
            array[id] = elem;
            metric += array[id];
            id = ((id + 1) % _size + _size) % _size;
        }

        T getMetric(){ return metric; }
        int size(){ return array.size(); }
        int getSize(){ return _size; }
        void clear(){ 
            array.clear();
            metric = {};
        }
    private:
        int id;
        int _size;
        vector<T> array;
        T metric;
    };
}

#endif