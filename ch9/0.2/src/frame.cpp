#include "myslam/frame.h"
namespace myslam
{

Frame::Frame() : id_(-1), time_stamp_(-1), camera_(nullptr)
{
}

Frame::Frame(long id, double time_stamp, SE3 T_c_w , Camera::Ptr camera , Mat color , Mat depth ) 
: id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth)
{
}

Frame::~Frame()
{
}

//factory function 创建frame
Frame::Ptr Frame::createFrame()
{
    // 静态变量，只在第一次进入这个函数的时候初始化为0，其余不执行初始化，只++。
    static long factory_id = 0;
    return Frame::Ptr(new Frame(factory_id++));
}

// 找给定点的深度
double Frame::findDepth(const cv::KeyPoint &kp)
{
    // 最接近的整数值
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    ushort d = depth_.ptr<ushort>(y)[x];
    // 如果这点有深度
    if (d != 0)
    {
        //cout<<camera_->depth_scale_<<endl;
        //cout<<"a"<<double(d) / camera_->depth_scale_<<endl;
        return double(d) / camera_->depth_scale_;
    }
    // 莫得的话看周围四国点
    else
    {
        int dx[4] = {-1, 0, 1, 0};
        int dy[4] = {0, -1, 0, 1};
        for (int i = 0; i < 4; i++)
        {
            d = depth_.ptr<ushort>(y + dy[i])[x + dx[i]];
            if (d != 0)
            {
                //cout<<"b"<<double(d) / camera_->depth_scale_<<endl;
                return double(d) / camera_->depth_scale_;
            }
        }
    }
    // 还是莫得，只好返回莫得
    
    return -1.0;
}

// 相机光心
Vector3d Frame::getCamCenter() const
{
    // 世界坐标系的原点乘T'，就是相机坐标系的原点。
    return T_c_w_.inverse().translation();
}

//看给定点是否在图像里
bool Frame::isInFrame(const Vector3d &pt_world)
{
    Vector3d p_cam = camera_->world2camera(pt_world, T_c_w_);
    if (p_cam(2, 0) < 0)
        return 0;
    Vector2d pixel=camera_->world2pixel(pt_world,T_c_w_);
    return pixel(0,0)>0&&pixel(0,0)<color_.cols&&pixel(1,0)>0&&pixel(1,0)<color_.rows;
}

} // namespace myslam