//
// Created by huws on 23-03-21.
//

#include <ros/ros.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <queue>
#include <mutex>

#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include "vo_system.h"

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

//config
myslam::VisualOdometry::Ptr vo_inited = nullptr;
ros::Publisher pub_pose_out;
//image
std::mutex mImgBufMutex;
queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
//imu
queue<sensor_msgs::ImuConstPtr> imuBuf;
std::mutex mImuBufMutex;
// other function
cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);

void left_img_callback(const sensor_msgs::ImageConstPtr &img0_msg)
{
    mImgBufMutex.lock();
    // if (imgLeftBuf.size()>30)
    //     imgLeftBuf.pop();
    imgLeftBuf.push(img0_msg);
    mImgBufMutex.unlock();
}

void right_img_callback(const sensor_msgs::ImageConstPtr &img1_msg)
{
    mImgBufMutex.lock();
    // if (imgRightBuf.size()>30)
    //     imgRightBuf.pop();
    imgRightBuf.push(img1_msg);
    mImgBufMutex.unlock();
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mImuBufMutex.lock();
    imuBuf.push(imu_msg);
    mImuBufMutex.unlock();
}

void SyncFunc()
{
    static const double maxTimeDiff = 0.005; //5ms
    while(1)
    {
        // cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        myslam::Multi_Sensor_Data data_input;
        //
        // if (vo_inited == nullptr) {
        //     printf("waiting for vo inited!\n");
        //     usleep(10*1000); //10ms
        //     continue;
        // }
        mImgBufMutex.lock();
        if (!imgLeftBuf.empty()&&!imgRightBuf.empty()) {
            tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            tImRight = imgRightBuf.front()->header.stamp.toSec();

            if (tImLeft < tImRight - maxTimeDiff)
            {
                imgLeftBuf.pop();
                printf("throw left img\n");
            }
            else if(tImLeft > tImRight + maxTimeDiff)
            {
                imgRightBuf.pop();
                printf("throw right img\n");
            }
            else{
                double time = imgLeftBuf.front()->header.stamp.toSec();
                //
                data_input.time_stamp_ = time / 1000.0; //ms
                data_input.left_img = GetImage(imgLeftBuf.front());
                imgLeftBuf.pop();
                data_input.right_img = GetImage(imgRightBuf.front());  
                imgRightBuf.pop();
            }
        }
        mImgBufMutex.unlock();

        if (!data_input.left_img.empty())
        {
            geometry_msgs::PoseStamped zzslam_pose;

            SE3 tcw_now = vo_inited->TrackStereo(data_input);
            cv::waitKey(1);

            Eigen::Matrix3d Rc0ci_eigen;
            Eigen::Matrix3d Rwc0_eigen;
            Eigen::Matrix3d Rwci_eigen;
            Eigen::Vector3d twc_eigen;
            Rwci_eigen << 
            0, 0, 1,
            -1, 0, 0,
            0, -1, 0;
            Rc0ci_eigen = tcw_now.inverse().rotationMatrix();
            Rwci_eigen = Rwc0_eigen * Rc0ci_eigen;
            twc_eigen = tcw_now.inverse().translation();

            Eigen::Quaterniond qwc;
            qwc = Rwci_eigen;
            zzslam_pose.header.frame_id = "map";
            zzslam_pose.pose.orientation.x = qwc.x();
            zzslam_pose.pose.orientation.y = qwc.y();
            zzslam_pose.pose.orientation.z = qwc.z();
            zzslam_pose.pose.orientation.w = qwc.w();
            zzslam_pose.pose.position.x = twc_eigen.x();
            zzslam_pose.pose.position.y = twc_eigen.y();
            zzslam_pose.pose.position.z = twc_eigen.z();

            pub_pose_out.publish(zzslam_pose);
        }
        
        
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "zzslam_ros");
    ros::NodeHandle node_handler("~"); 
    std::string node_name = ros::this_node::getName();
    //
    std::string config_file_path = "../config/default.yaml";
    node_handler.param<std::string>(node_name + "/settings_file", config_file_path, "file_not_set");
    //VO初始化
    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry(config_file_path));
    bool init_success = vo->Init();
    if(!init_success) {
        std::cout << "VO init failed!,please check." << std::endl;
        return -1;
    }
    vo_inited = vo;

    // sub topic
    ros::Subscriber sub_img_left = node_handler.subscribe("/camera/left/image_raw", 30, left_img_callback);
    ros::Subscriber sub_img_rifht = node_handler.subscribe("/camera/right/image_raw", 30, right_img_callback);
    // ros::Subscriber sub_imu = node_handler.subscribe("/camera/imu", 200, imu_callback);
    // ros::Subscriber sub_gnss = node_handler.subscribe("/gnss", 200, gnss_callback);

    // pub topic
    pub_pose_out = node_handler.advertise<geometry_msgs::PoseStamped> ("/zzslam/out_pose", 30);

    // sync tread
    std::thread sync_thread(SyncFunc);

    ros::spin();

    //
    // todo: save trajectory
    // vo_inited->exit();
    ros::shutdown();
    return 0;
}

cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    if(cv_ptr->image.type()==0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}
