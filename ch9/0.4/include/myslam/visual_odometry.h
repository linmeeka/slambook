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
    vector<cv::KeyPoint> keypoints_curr_;
    Mat descriptors_curr_;
    
    cv::FlannBasedMatcher matcher_flann_;
    vector<MapPoint::Ptr> match_3dpts_;
    vector<int> match_2dkp_index_;


    SE3 T_c_w_estimated_; // T_cr   T_cw=T_cr*T_rw T_rw is known
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
    double map_point_erase_ratio_;

public:
    VisualOdometry();
    ~VisualOdometry();

    bool addFrame(Frame::Ptr frame);

protected:
    void extractKeyPoints();
    void computeDescriptors();
    void featureMatching();
    void poseEstimationPnP();
    void optimizeMap();

    void addKeyFrame();
    void addMapPoints();
    bool checkEstimatedPose();
    bool checkKeyFrame();

    double getViewAngle(Frame::Ptr frame,MapPoint::Ptr point);
};

}
    
#endif // VISUALODOMETRY_H
