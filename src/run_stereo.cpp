#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "stereo_slam/comm_include.h"
#include "stereo_slam/camera.h"
#include "stereo_slam/struct_def.h"
#include "stereo_slam/map.h"
#include "stereo_slam/odometry.h"
#include "stereo_slam/optimization.h"
#include "stereo_slam/viewer.h"

using namespace std;

bool CameraInit(string calibPath, std::vector<Camera::Ptr>& cameras_) {
    // read camera intrinsics and extrinsics
    ifstream fin(calibPath);
    if (!fin) {
        ROS_INFO("cannot find calib.txt!"); 
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        char camera_name[3];
        for (int k = 0; k < 3; ++k) {
            fin >> camera_name[k];
        }
        double projection_data[12];
        for (int k = 0; k < 12; ++k) {
            fin >> projection_data[k];
        }
        Mat33 K;
        K << projection_data[0], projection_data[1], projection_data[2],
            projection_data[4], projection_data[5], projection_data[6],
            projection_data[8], projection_data[9], projection_data[10];
        Vec3 t;
        t << projection_data[3], projection_data[7], projection_data[11];
        t = K.inverse() * t;
        // K = K * 0.5;
        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                          t.norm(), SE3(SO3(), t)));
        cameras_.push_back(new_camera);
        cout << "Camera " << i << " extrinsics: " << t.transpose() << endl;
    }
    fin.close();
    return true;
}


int main(int argc, char *argv[])
{
    //执行 ros 节点初始化
    ros::init(argc, argv,"run_stereo");
    //创建 ros 节点句柄(非必须)
    ros::NodeHandle nh;
    //控制台输出 hello world
    ROS_INFO("hello world!");

    string datasetPath;
    nh.getParam("/dataset_dir", datasetPath);
    ROS_INFO("dataset: %s", datasetPath.c_str());

	string calibPath = datasetPath + "calib.txt";
	std::vector<Camera::Ptr> cameras;
	CameraInit(calibPath, cameras);
	Camera::Ptr leftCamera = cameras.at(0);
	Camera::Ptr rightCamera = cameras.at(1);

	// //然后可以直接访问其成员属性，例如：
	// double fx = firstCamera->fx_;
	// double fy = firstCamera->fy_;
	// double cx = firstCamera->cx_;
	// double cy = firstCamera->cy_;
	// double baseline = firstCamera->baseline_;
	// SE3 pose = firstCamera->pose_;
	// SE3 poseInv = firstCamera->pose_inv_;
	// cout << "fx " << fx << endl;
	// cout << "fy " << fy << endl;
	// cout << "cx " << cx << endl;
	// cout << "cy " << cy << endl;
	// cout << "baseline " << baseline << endl;
	// cout << "pose " << pose.matrix() << endl;
	// cout << "poseInv " << poseInv.matrix()  << endl;

    bool output_pose;
    nh.getParam("/output_pose", output_pose);

    bool use_rviz;
    nh.getParam("/use_rviz", use_rviz);

	// // create components and links
    Odometry::Ptr odometry = Odometry::Ptr(new Odometry);
    Optimization::Ptr optimization = Optimization::Ptr(new Optimization);
    Map::Ptr map = Map::Ptr(new Map);
    // Viewer::Ptr viewer = Viewer::Ptr(new Viewer);
    Viewer::Ptr viewer = Viewer::Ptr(new Viewer(nh));
    // Viewer::Ptr viewer = std::make_shared<Viewer>(nh);

    odometry->SetBackend(optimization);
    odometry->SetMap(map);
    odometry->SetCameras(leftCamera, rightCamera);
    odometry->SetViewer(viewer);

    optimization->SetMap(map);
    optimization->SetCameras(leftCamera, rightCamera);

    viewer->SetMap(map);

	// load params
    ros::Publisher pubLeftImage = nh.advertise<sensor_msgs::Image>("/leftImage",1000);
	ros::Publisher pubRightImage = nh.advertise<sensor_msgs::Image>("/rightImage",1000);

	// load image list
	FILE* file = std::fopen((datasetPath + "times.txt").c_str() , "r");
	if(file == NULL){
	    printf("cannot find file: %stimes.txt\n", datasetPath.c_str());
	    ROS_BREAK();
	    return 0;          
	}

    double imageTime;
	vector<double> imageTimeList;
	while (fscanf(file, "%lf", &imageTime) != EOF)
	{
	    imageTimeList.push_back(imageTime);
	}
	std::fclose(file);

    // load stereo images
	string leftImagePath, rightImagePath;
	cv::Mat leftImage, rightImage;

    for (size_t i = 0; i < imageTimeList.size(); i++)
	{	
		if(ros::ok())
		{
			printf("\nprocess image %d\n", (int)i);
			stringstream ss;
			ss << setfill('0') << setw(6) << i;
			leftImagePath = datasetPath + "image_0/" + ss.str() + ".png";
			rightImagePath = datasetPath + "image_1/" + ss.str() + ".png";

			leftImage = cv::imread(leftImagePath, cv::IMREAD_GRAYSCALE);
			sensor_msgs::ImagePtr imLeftMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", leftImage).toImageMsg();
			imLeftMsg->header.stamp = ros::Time(imageTimeList[i]);
			pubLeftImage.publish(imLeftMsg);

			rightImage = cv::imread(rightImagePath, cv::IMREAD_GRAYSCALE);
			sensor_msgs::ImagePtr imRightMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", rightImage).toImageMsg();
			imRightMsg->header.stamp = ros::Time(imageTimeList[i]);
			pubRightImage.publish(imRightMsg);

            auto new_frame = Frame::CreateFrame();
            new_frame->left_img_ = leftImage;
            new_frame->right_img_ = rightImage;

            // cv::Mat image_left_resized, image_right_resized;
            // cv::resize(leftImage, image_left_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
            // cv::resize(rightImage, image_right_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

            // auto new_frame = Frame::CreateFrame();
            // new_frame->left_img_ = image_left_resized;
            // new_frame->right_img_ = image_right_resized;

            auto t1 = std::chrono::steady_clock::now();
            bool success = odometry->AddFrame(new_frame);
            auto t2 = std::chrono::steady_clock::now();
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            std::cout << "VO cost time: " << time_used.count() << " seconds." << std::endl;
        }
    }

    return 0;
}