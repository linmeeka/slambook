#ifndef FRAME_H
#define FRAME_H
#include "myslam/common_include.h"
#include "myslam/camera.h"
namespace myslam
{
class MapPoint;
class Frame
{
  public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long id_;   // id
    double time_stamp_;  // 时间戳
    SE3 T_c_w_;          //T 位姿
    Camera::Ptr camera_; //相机
    Mat color_, depth_;

  public:
    Frame();
    Frame(long id, double time_stamp = 0, SE3 T_c_w = SE3(), Camera::Ptr camera = nullptr, Mat color = Mat(), Mat depth = Mat());
    ~Frame();

    //factory function 创建frame
    static Frame::Ptr createFrame();

    // 找给定点的深度
    double findDepth(const cv::KeyPoint &kp);

    // 相机光心
    Vector3d getCamCenter() const;

    //看给定点是否在图像里
    bool isInFrame(const Vector3d &pt_world);
};
} // namespace myslam
#endif