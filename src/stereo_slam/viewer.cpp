//
// Created by gaoxiang on 19-5-4.
//
#include "stereo_slam/viewer.h"
#include "stereo_slam/struct_def.h"

#include <opencv2/opencv.hpp>

// Viewer(ros::NodeHandle& nh) : nh_(nh)
Viewer::Viewer(ros::NodeHandle& nh) : nh_(nh) {
    pub_frame = nh_.advertise<sensor_msgs::Image>("/frame", 100);
    pub_pose = nh_.advertise<visualization_msgs::Marker>("pose", 100);
    pub_mappoint = nh_.advertise<sensor_msgs::PointCloud2>("map_points", 1);
    // viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
}

void Viewer::Close() {
    viewer_running_ = false;
    viewer_thread_.join();
}

void Viewer::AddCurrentFrame(Frame::Ptr current_frame) {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    current_frame_ = current_frame;

    cv::Mat frame = PlotFrameImage();
    sensor_msgs::ImagePtr frameMsg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", frame).toImageMsg();
    frameMsg->header.stamp = ros::Time::now();
    pub_frame.publish(frameMsg);
}

void Viewer::UpdateMap() {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    assert(map_ != nullptr);
    active_keyframes_ = map_->GetActiveKeyFrames();
    active_landmarks_ = map_->GetActiveMapPoints();
    map_updated_ = true;
    if (active_keyframes_.size() > 0) {
        unsigned long maxKey = 0;
        Frame::Ptr maxFrame = nullptr;
        for (auto& pair : active_keyframes_) {
            if (pair.first >= maxKey) {
                maxKey = pair.first;
                maxFrame = pair.second;
            }
        }
        publish_fixed_pose(maxFrame);
        // if (maxFrame != nullptr) {
        //     publish_fixed_pose(maxFrame);
        // }
    }
    std::cout << "KF after" << std::endl;
    if (map_ && map_updated_) {
        // unsigned long maxKey = 0;
        // Frame::Ptr maxFrame = nullptr;

        // for (const auto& pair : active_keyframes_) {
        //     if (pair.first > maxKey) {
        //         maxKey = pair.first;
        //         maxFrame = pair.second;
        //     }
        // }
        // publish_fixed_pose(maxFrame);

        MapPoints_to_PointCloud2(active_landmarks_);
        pub_mappoint.publish(feature_map_);
    }
}

cv::Mat Viewer::PlotFrameImage() {
    cv::Mat img_out;
    cv::cvtColor(current_frame_->left_img_, img_out, cv::COLOR_GRAY2BGR);
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_left_[i]->map_point_.lock()) {
            auto feat = current_frame_->features_left_[i];
            cv::circle(img_out, feat->position_.pt, 3, cv::Scalar(0, 250, 0), 3);
        }
    }
    return img_out;
}


void Viewer::publish_fixed_pose(const Frame::Ptr frame)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time(0);

    marker.ns = "fixed_pose";
    marker.id = frame->id_;

    uint32_t shape = visualization_msgs::Marker::CUBE;
    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    SE3 T_w_c = frame->pose_.inverse();

    marker.pose.position.x = T_w_c.translation()(0);
    marker.pose.position.y = T_w_c.translation()(1);
    marker.pose.position.z = T_w_c.translation()(2);
    marker.pose.orientation.x = T_w_c.unit_quaternion().x();
    marker.pose.orientation.y = T_w_c.unit_quaternion().y();
    marker.pose.orientation.z = T_w_c.unit_quaternion().z();
    marker.pose.orientation.w = T_w_c.unit_quaternion().w();

    marker.scale.x = 5;
    marker.scale.y = 5;
    marker.scale.z = 5;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    pub_pose.publish(marker);

    ros::spinOnce();
}


int Viewer::MapPoints_to_PointCloud2(const std::unordered_map<unsigned long, MapPoint::Ptr> &active_landmarks_)
{
    if (active_landmarks_.empty())
    {
        std::cout << "Invalid input: empty active landmarks" << std::endl;
        return -1; // 返回一个错误代码表示执行失败
    }

    // 设置feature_map_的基本属性
    const int num_channels = 3; // x y z

    feature_map_ = sensor_msgs::PointCloud2();
    feature_map_.header.stamp = ros::Time::now();
    feature_map_.header.frame_id = "map";
    feature_map_.height = 1;
    feature_map_.width = active_landmarks_.size();
    feature_map_.is_bigendian = false;
    feature_map_.is_dense = true;
    feature_map_.point_step = num_channels * sizeof(float);
    feature_map_.row_step = feature_map_.point_step * feature_map_.width;
    feature_map_.fields.resize(num_channels);

    // 配置feature_map_的字段属性
    std::string channel_id[] = {"x", "y", "z"};
    for (int i = 0; i < num_channels; i++)
    {
        feature_map_.fields[i].name = channel_id[i];
        feature_map_.fields[i].offset = i * sizeof(float);
        feature_map_.fields[i].count = 1;
        feature_map_.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    feature_map_.data.resize(feature_map_.row_step * feature_map_.height);

    unsigned char *feature_map_data_ptr = &(feature_map_.data[0]);

    // 将3D点的数据填充到feature_map_.data中
    float data_array[num_channels];

    int index = 0;
    for (const auto &landmark : active_landmarks_)
    {
        data_array[0] = landmark.second->Pos()[0]; // x
        data_array[1] = landmark.second->Pos()[1]; // y
        data_array[2] = landmark.second->Pos()[2]; // z
        // std::cout << "Update Map data_array: " << data_array[0] << data_array[1] << data_array[2] << std::endl;
        memcpy(feature_map_data_ptr + (index * feature_map_.point_step), data_array, num_channels * sizeof(float));
        index++;
    }

    return 0; // 返回执行成功的代码
}
