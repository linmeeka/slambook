#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include "myslam/g2o_types.h"
namespace myslam
{

VisualOdometry::VisualOdometry()
    : state_(INITIALIZING), ref_(nullptr), curr_(nullptr), map_(new Map), num_lost_(0), num_inliers_(0), matcher_flann_(new cv::flann::LshIndexParams(5, 10, 2))
{
    num_of_features_ = Config::get<int>("number_of_features");
    scale_factor_ = Config::get<double>("scale_factor");
    level_pyramid_ = Config::get<int>("level_pyramid");
    match_ratio_ = Config::get<float>("match_ratio");
    max_num_lost_ = Config::get<float>("max_num_lost");
    min_inliers_ = Config::get<int>("min_inliers");
    key_frame_min_rot = Config::get<double>("keyframe_rotation");
    key_frame_min_trans = Config::get<double>("keyframe_translation");
    map_point_erase_ratio_ = Config::get<double>("map_point_erase_ratio");
    orb_ = cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_);
}

VisualOdometry::~VisualOdometry()
{
}

bool VisualOdometry::addFrame(Frame::Ptr frame)
{
    switch (state_)
    {
    case INITIALIZING:
    {
        state_ = OK;
        ref_ = curr_ = frame;
        //map_->insertKeyFrame(frame);
        extractKeyPoints();
        computeDescriptors();
        addKeyFrame();
        break;
    }
    case OK:
    {
        curr_ = frame;
        curr_->T_c_w_ = ref_->T_c_w_;
        extractKeyPoints();
        computeDescriptors();
        featureMatching();
        poseEstimationPnP();
        if (checkEstimatedPose())
        {
            curr_->T_c_w_ = T_c_w_estimated_; //直接估计T_cw
            optimizeMap();
            num_lost_ = 0;
            if (checkKeyFrame())
                addKeyFrame();
        }
        else
        {
            num_lost_++;
            if (num_lost_ > max_num_lost_)
                state_ = LOST;
            return false;
        }
        break;
    }
    case LOST:
    {
        cout << "vo has lost" << endl;
        break;
    }
    }
    return true;
}

void VisualOdometry::extractKeyPoints()
{
    boost::timer timer;
    orb_->detect(curr_->color_, keypoints_curr_);
    cout << "Extract keypoitns cost time: " << timer.elapsed() << endl;
}

void VisualOdometry::computeDescriptors()
{
    boost::timer timer;
    orb_->compute(curr_->color_, keypoints_curr_, descriptors_curr_);
    cout << "Compute descriptor cost time: " << timer.elapsed() << endl;
}

void VisualOdometry::featureMatching()
{
    boost::timer timer;
    vector<cv::DMatch> matches;
    Mat desp_map;
    vector<MapPoint::Ptr> candidate;
    for (auto &allpoints : map_->map_points_)
    {
        // <ID,map_point::Ptr
        MapPoint::Ptr &p = allpoints.second;
        // pos：特征点p在世界坐标系中的坐标，看它在不在当前帧视野里
        if (curr_->isInFrame(p->pos_))
        {
            // 取的是地图中特征点的引用，直接++
            p->visible_times_++;
            // 存在于当前帧中的地图特征点和其描述子集合
            candidate.push_back(p);
            desp_map.push_back(p->descriptor_);
        }
    }

    // 当前帧特征点的描述子，与地图特征点的描述子匹配
    matche_flann_.match(desp_map, descriptors_curr_, matches);

    //定义lambda函数，从matches里，按照其中元素的cv::DMatch.distance找到最小距离的元素
    //得到它的最小距离
    float min_dis = std::min_element(
                        matches.begin(), matches.end(),
                        [](const cv::DMatch &m1, const cv::DMatch &m2) {
                            return m1.distance < m2.distance;
                        })
                        ->distance;
    match_3dpts_.clear();
    match_2dkp_index_.clear();

    // 找到距离合适的匹配
    // 距离小于30或min_dis的match_ratio倍，算匹配上
    for (cv::DMatch &m : matches)
    {
        if (m.distance < max<float>(min_dis * match_ratio_, 30.0))
        {
            // 当前帧在地图中匹配到的特征点（地图中的特征点的三维坐标）
            match_3dpts_.push_back(candidate[m.queryIdx]);
            // 当前帧在地图中匹配到的特征点的id（当前帧上2d点的id）
            match_2dkp_index_.push_back(m.trainIdx);
        }
    }
    cout << "good matches:" << match_3dpts_.size() << endl;
    cout << "match cost time: " << timer.elapsed() << endl;
}

void VisualOdometry::poseEstimationPnP()
{
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;
    // 2d特征点
    for (int index : match_2dkp_index_)
    {
        pts2d.push_back(keypoints_curr_[index].pt);
    }
    //3d坐标点
    for (MapPoint::Ptr pt : match_3dpts_)
    {
        pts3d.push_back(pt->getPositionCV());
    }
    // 内参
    Mat K = (cv::Mat_<double>(3, 3) << ref_->camera_->fx_, 0, ref_->camera_->cx_,
             0, ref_->camera_->fy_, ref_->camera_->cy_,
             0, 0, 1);
    //输出
    // rvec ，tvec 估计的旋转矩阵和平移矩阵
    // inliers ： 内点。 feature_matches_里存在误匹配，内点是正确匹配的点
    Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
    num_inliers_ = inliers.rows;
    cout << "pnp inliers:" << num_inliers_ << endl;
    T_c_w_estimated_ = SE3(
        SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
        Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));
    // 使用bundle adjestment优化位姿
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 2>> Block;
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block *solver_ptr = new Block(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // 优化变量，一个顶点，位姿
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    // 设定初值
    pose->setEstimate(g2o::SE3Quat(
        T_c_w_estimated_.rotation_matrix(),
        T_c_w_estimated_.translation()));
    optimizer.addVertex(pose);

    // 边
    for (int i = 0; i < inliers.rows; i++)
    {
        // 对每一个内点
        int index = inliers.at<int>(i, 0);
        EdgeProjectXYZ2UVPoseOnly *edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setId(i);
        // 一元边，只连接位姿顶点
        edge->setVertex(0, pose);
        edge->camera_ = curr_->camera_.get();
        // 三维点坐标，把它重投影和下面的measurement计算误差
        edge->point_ = Vector3d(pts3d[index].x, pts3d[index].y, pts3d[index].z);
        // 观测到的值
        edge->setMeasurement(Vector2d(pts2d[index].x, pts2d[index].y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        // 内点才算匹配上了
        match_3dpts_[index]->matched_times_++;
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    T_c_w_estimated_ = SE3(
        pose->estimate().rotation(),
        pose->estimate().translation());
}

bool VisualOdometry::checkEstimatedPose()
{
    if (num_inliers_ < min_inliers_)
    {
        cout << "reject because inlier is too small:" << num_inliers_ << endl;
        return false;
    }
    // 估计的是T_c_w，ref_->T_c_w_是参考帧的位姿(T_r_w)，T_r_c*T_c_w=T_r_w。
    // 要计算参考帧到当前帧的转换
    // 参考帧转到当前，当前在转到世界，等于参考帧到世界坐标系的转换
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    if (d.norm() > 5.0)
    {
        cout << "reject because motion is too large:" << d.norm() << endl;
        return false;
    }
    return true;
}

bool VisualOdometry::checkKeyFrame()
{
    SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();
    Sophus::Vector6d d = T_c_w_estimated_.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if (rot.norm() > key_frame_min_rot || trans.norm() > key_frame_min_trans)
        return true;
    return false;
}

void VisualOdometry::addKeyFrame()
{
    // 对于第一帧，插入所有3d点
    if (map_->keyframes_.empty())
    {
        for (size_t i = 0; i < keypoints_curr_.size(); i++)
        {
            double d = curr_->findDepth(keypoints_curr_[i]);
            if (d < 0)
                continue;
            Vector3d p_world = ref_->camera_->pixel2world(
                Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), curr_->T_c_w_, d);
            // 可是上面得到的已经是世界坐标了啊？
            // 把坐标点挪到世界坐标系下。减去相机坐标系光心的平移，不是以第一帧为世界坐标。
            Vector3d n = p_world - ref_->getCamCenter();
            n.normalize();
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                p_world, n, descriptors_curr_.row(i).clone(), curr_.get());
            map_->insertMapPoint(map_point);
        }
    }
    map_->insertKeyFrame(curr_);
    ref_ = curr_;
}

// 向地图中添加新的特征点
// 只有地图中的特征点太少了才调用
void VisualOdometry::addMapPoints()
{
    vector<bool> matched(keypoints_curr_.size(), false);
    for (int index : match_2dkp_index_)
        matched[index] = true;
    for (int i = 0; i < keypoints_curr_.size(); i++)
    {
        // 当前帧的特征点，已经和地图中的特征点匹配上，说明已经在地图里，不需要添加
        if (matched[i] == true)
            continue;
        // 对于没在地图里的特征点，都加进去
        // 仍然是上一帧到世界的T已经估计好，从上一帧里面找深度
        double d = ref_->findDepth(keypoints_curr_[i]);
        if (d < 0)
            continue;
        Vector3d p_world = ref_->camera->pixel2world(
            Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), curr_->T_c_w_, d);
        // 挪到世界坐标系下
        Vector3d n = p_world - ref->getCamCenter();
        n.normalize();
        MapPoint::Ptr map_point = MapPoint::createMapPoint(
            p_world, n, descriptors_curr_.row(i).clone(), curr_.get());
        map_->insertMapPoint(map_point);
    }
}

void VisualOdometry::optimizeMap()
{
    for (auto iter = map_->map_points_.begin(); iter != map_->map_points_.end();)
    {
        // 删除当前帧看不见的点
        if (!curr_->isInFrame(iter->second->pos_))
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        // 删除总被看见却总匹配不上的点
        float match_ratio = float(iter->second->matched_times_) / iter->second->visible_times_;
        if (match_ratio < map_point_erase_ratio_)
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        // 如果太偏了
        double angle = getViewAngle(curr_, iter->second);
        if (angle > M_PI / 6.)
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        if (iter->second->good_ == false)
        {
            // ?? todo try triangulate this map point
        }
        iter++;
    }
    if (match_2dkp_index_.size() < 100)
        addMapPoints();
    if (map_->map_points_.size() > 1000)
    {
        map_point_erase_ratio_ += 0.05;
    }
    else
        map_point_erase_ratio_ = 0.1;
    cout << "map points: " << map_points_.size() << endl;
}

double VisualOdometry::getViewAngle(Frame::Ptr frame, MapPoint::Ptr point)
{
    vector3d n = point->pos_ - frame->getCamCenter();
    n.normalize();
    return acos(n.transpose() * point->norm_);
}

} // namespace myslam
