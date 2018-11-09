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
    bool good_;
    Vector3d pos_;
    Vector3d norm_;
    Mat descriptor_;
    int visible_times_;
    int matched_times_;
    list<Frame*> observed_frames_; // keyframes that can observe this point
    static unsigned long factory_id_;
    MapPoint();
    MapPoint(
        unsigned long id,
        const Vector3d pos,
        const Vector3d norm,
        Frame* frame=nullptr,
        const Mat& descriptor=Mat()
    );
    
    //const修饰函数：函数不能改变类的成员变量
    inline cv::Point3f getPositionCV() const{
        return cv::Point3f(pos_(0,0),pos_(1,0),pos_(2,0));
    }

    //类的静态成员函数，属于类本身，不属于对象。    只能访问类的静态成员变量和静态成员函数
    static MapPoint::Ptr createMapPoint();
    static MapPoint::Ptr createMapPoint
    (
        const Vector3d& pos_world,
        const Vector3d& norm,
        const Mat& descriptor,
        Frame *frame
    );
};
}
#endif