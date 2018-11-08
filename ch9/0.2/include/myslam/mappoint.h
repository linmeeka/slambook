#ifndef MAPPOINT_H
#define MAPPOINT_H
#include"myslam/common_include.h"
namespace myslam
{
class Frame;
class MapPoint;
// 单个地图中点的类，不是地图
class MapPoint
{
public:
    typedef shared_ptr<MapPoint>Ptr;
    unsigned long id_;
    Vector3d pos_;
    Vector3d norm_;
    Mat descriptor_;
    int observed_times_;
    int correct_times_;

    MapPoint();
    MapPoint(long id,Vector3d position,Vector3d norm);
    //类的静态成员函数，属于类本身，不属于对象。    只能访问类的静态成员变量和静态成员函数
    static MapPoint::Ptr creatMapPoint();
};
}
#endif