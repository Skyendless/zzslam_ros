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
#include <sensor_msgs/NavSatFix.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

using namespace std;

//config
myslam::VisualOdometry::Ptr vo_inited = nullptr;
ros::Publisher pub_pose_out;
ros::Publisher pub_path;
ros::Publisher pub_odometry;
//image
std::mutex mImgBufMutex;
queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
//imu
queue<sensor_msgs::ImuConstPtr> imuBuf;
std::mutex mImuBufMutex;
//
SE3 Twc0;
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
                data_input.time_stamp_ = time; //s
                data_input.left_img = GetImage(imgLeftBuf.front());
                imgLeftBuf.pop();
                data_input.right_img = GetImage(imgRightBuf.front());  
                imgRightBuf.pop();
            }
        }
        mImgBufMutex.unlock();

        if (!data_input.left_img.empty())
        {
            SE3 tcc0_now = vo_inited->TrackStereo(data_input);
            cv::waitKey(1);

            Eigen::Vector3d global_t;
            Eigen:: Quaterniond global_q;

            SE3 Twci = Twc0 * tcc0_now.inverse();
            global_q = Twci.rotationMatrix();
            global_t = Twci.translation();

            // nav_msgs::Odometry odometry;
            // // odometry.header = pose_msg->header;
            // odometry.header.frame_id = "world";
            // odometry.child_frame_id = "world";
            // odometry.pose.pose.position.x = global_t.x();
            // odometry.pose.pose.position.y = global_t.y();
            // odometry.pose.pose.position.z = global_t.z();
            // odometry.pose.pose.orientation.x = global_q.x();
            // odometry.pose.pose.orientation.y = global_q.y();
            // odometry.pose.pose.orientation.z = global_q.z();
            // odometry.pose.pose.orientation.w = global_q.w();
            // pub_odometry.publish(odometry);
            // pub_global_path.publish(*global_path);
            // publish_car_model(t, global_t, global_q);

            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time(0);
            pose.header.frame_id = "world";
            pose.pose.position.x = global_t.x();
            pose.pose.position.y = global_t.y();
            pose.pose.position.z = global_t.z();
            pose.pose.orientation.x = global_q.x();
            pose.pose.orientation.y = global_q.y();
            pose.pose.orientation.z = global_q.z();
            pose.pose.orientation.w = global_q.w();
            pub_pose_out.publish(pose);

            // tf::Transform transform_w_c;
            // transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
            // tf::Quaternion q;
            // q = global_q;

            static tf::TransformBroadcaster tf_broadcaster;
            tf::Transform transform_w_c;
            tf::Quaternion tmp_q(global_q.x(), global_q.y(), global_q.z(), global_q.w());
            transform_w_c.setRotation(tmp_q);
            transform_w_c.setOrigin(tf::Vector3(global_t.x(), global_t.y(), global_t.z() ));
            tf_broadcaster.sendTransform(tf::StampedTransform(transform_w_c,  ros::Time::now(), "world", "camera")); 


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
    //
    Eigen::Matrix3d Rwc0_eigen;
    Eigen::Vector3d twc0_eigen;
    Rwc0_eigen << 
    0, 0, 1,
    -1, 0, 0,
    0, -1, 0;
    // Rwc0_eigen << 
    // 0,  1,  0,
    // -1,  0,  0,
    // 0,  0,  1;
    twc0_eigen << 0, 0, 0;
    Twc0 = SE3(Rwc0_eigen, twc0_eigen);
    // sub topic
    ros::Subscriber sub_img_left = node_handler.subscribe("/camera/left/image_raw", 30, left_img_callback);
    ros::Subscriber sub_img_rifht = node_handler.subscribe("/camera/right/image_raw", 30, right_img_callback);
    // ros::Subscriber sub_imu = node_handler.subscribe("/camera/imu", 200, imu_callback);
    // ros::Subscriber sub_gnss = node_handler.subscribe("/gnss", 200, gnss_callback);

    // pub topic
    pub_pose_out = node_handler.advertise<geometry_msgs::PoseStamped> ("/zzslam/out_pose", 30);
    // pub_path = node_handler.advertise<nav_msgs::Path>("/zzslam/path", 100);
    // pub_odometry = node_handler.advertise<nav_msgs::Odometry>("/zzslam/odometry", 100);
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
