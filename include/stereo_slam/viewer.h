//
// Created by gaoxiang on 19-5-4.
//

#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <thread>

#include "stereo_slam/comm_include.h"
#include "stereo_slam/struct_def.h"
#include "stereo_slam/map.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>


/**
 * 可视化
 */
class Viewer {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer(ros::NodeHandle& nh);

    void SetMap(Map::Ptr map) { map_ = map; }

    void Close();

    // 增加一个当前帧
    void AddCurrentFrame(Frame::Ptr current_frame);

    // 更新地图
    void UpdateMap();

   private:
    void ThreadLoop();

    // void DrawFrame(Frame::Ptr frame, const float* color);

    // void DrawMapPoints();

    // camera follow the current frame 
    // void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

    // plot the features in current frame into an image
    cv::Mat PlotFrameImage();

    void publish_fixed_pose(const Frame::Ptr frame);

    int MapPoints_to_PointCloud2(const std::unordered_map<unsigned long, MapPoint::Ptr> &active_landmarks_);

    Frame::Ptr current_frame_ = nullptr;
    Map::Ptr map_ = nullptr;

    std::thread viewer_thread_;
    bool viewer_running_ = true;

    std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
    std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex_;

    ros::NodeHandle nh_;
    ros::Publisher pub_frame;
    ros::Publisher pub_pose;
    ros::Publisher pub_mappoint;
    sensor_msgs::PointCloud2 feature_map_;
};

#endif  // MYSLAM_VIEWER_H
