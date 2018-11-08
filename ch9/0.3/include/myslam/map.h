#ifndef MAP_H
#define MAP_H
#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"
namespace myslam
{
//用来管理所有关键帧和路标点
class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    // hash map
    unordered_map<unsigned long, MapPoint::Ptr> map_points_;
    unordered_map<unsigned long, Frame::Ptr> keyframes_;

    Map() {}

    void insertKeyFrame(Frame::Ptr frame);
    void insertMapPoint(MapPoint::Ptr map_point);
};
} // namespace myslam
#endif