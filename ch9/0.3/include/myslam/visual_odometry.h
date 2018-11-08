#ifndef  VISUALODOMETRY_H
#define VISUALODOMETRY_H
#include "myslam/common_include.h"
#include "myslam/map.h"

#include <opencv2/features2d/features2d.hpp>

namespace myslam
{

class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr;
    // 前端状态
    enum VOState{
        INITIALIZING=-1,
        OK=0,
        LOST
    };

    VOState state_;
    Map::Ptr map_;
    Frame::Ptr ref_;
    Frame::Ptr curr_;

    cv::Ptr<cv::ORB> orb_; // 特征点
    vector<cv::Point3f> pts_3d_ref_;
    vector<cv::KeyPoint> keypoints_curr_;
    Mat descriptors_curr_;
    Mat descriptors_ref_;
    vector<cv::DMatch> feature_matches_;

    SE3 T_c_r_estimated_; // T_cr   T_cw=T_cr*T_rw T_rw is known
    int num_inliers_;
    int num_lost_;

    int num_of_features_;
    double scale_factor_;
    int level_pyramid_;
    float match_ratio_;
    int max_num_lost_;
    int min_inliers_;

    double key_frame_min_rot; // min rotation of two key-frames
    double key_frame_min_trans; // translation

public:
    VisualOdometry();
    ~VisualOdometry();

    bool addFrame(Frame::Ptr frame);

protected:
    void extractKeyPoints();
    void computeDescriptors();
    void featureMatching();
    void poseEstimationPnP();
    void setRef3DPoints();

    void addKeyFrame();
    bool checkEstimatedPose();
    bool checkKeyFrame();

};

}
    
#endif // VISUALODOMETRY_H
