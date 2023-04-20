//
// Created by Huws on 2022/12/09.
//

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <dirent.h>

using namespace std;
using namespace cv;

// string file_1 = "../data/LK1.png";
// string file_2 = "../data/LK2.png";

// const cv::Mat cv_K = ( cv::Mat_<double> ( 3,3 ) <<
// 364.860000, 0.0, 317.437333,
// 0.0, 486.986667, 246.824000,
// 0.0, 0.0, 1.0 );
// const cv::Mat cv_D = ( cv::Mat_<double> ( 5,1 ) <<
// -0.419666, 0.195211, -0.000372, -0.000372, -0.045762);
// static cv::Mat NewCameraMatrix;
// static cv::Mat map1, map2;
// static const double alpha = 0;

bool is_vedio = false;
ros::Publisher pub_img;
// string vedio_path = string("/home/efsz/Videos/iphone_calib2.mp4"); // first image
void video_pub(const string &vedio_path_input);
void picture_stream_pub(const string &vedio_path_input);
bool compare(std::string a, std::string b);

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "usage: rosrun vedio_pub_node your_data_path" << endl;
        return -1;
    }
    
    ros::init(argc, argv, "video_pub_node");
    ros::NodeHandle n("~");

    pub_img = n.advertise<sensor_msgs::Image>("video_image", 100);

    string vedio_path = string(argv[1]);
    if (is_vedio)
    {
        video_pub(vedio_path);
    }
    else{
        picture_stream_pub(vedio_path);
    }

    // ros::spin();

    return 0;
}


void video_pub(const string &vedio_path_input)
{
    cv::VideoCapture video_capture(vedio_path_input);

    int rate_sleep = 0 ;
    if (!video_capture.isOpened()) {
        cout << "Error opening video stream or file" << endl;
        return ;
    }
    else {
        // Obtain fps and frame count by get() method and print
        int fps = video_capture.get(CAP_PROP_FPS);
        cout << "Frames per second :" << fps << endl;
        rate_sleep = (int)1000/fps ; //ms
        // Obtain frame_count using opencv built in frame count reading method
        int frame_count = video_capture.get(CAP_PROP_FRAME_COUNT);
        cout << "Frame count :" << frame_count << endl;
    }

    cv::Mat frame,gray;
    while (video_capture.isOpened())
    {
        bool isSuccess = video_capture.read(frame);

        if(isSuccess) {
            // cout << "frame.rows is :" << frame.rows << "frame.cols is :" << frame.cols <<endl;
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            msg->header.stamp = ros::Time::now();
            pub_img.publish(msg);
            // cv::imshow("Frame", frame);
        }
        else{
            break;
        }
        int key = cv::waitKey(rate_sleep);
        if (key == 'q'|| key == 27) {
            cout << "q or esc key is pressed by the user. Stopping the video" << endl;
            break;
        }
    }
}


//
bool GetFileName(std::string path, std::vector<std::string> &files) {
    DIR *pDir; // 是头文件<dirent.h>的类型
    struct dirent *ptr; // opendir、readdir这些都是头文件dirent.h
    if (!(pDir = opendir(path.c_str()))) return false;
    while ((ptr = readdir(pDir)) != 0) {
    // strcmp是C语言里的，只导入string,然后std::strcmp都是没有的
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
            // files.push_back(path + "/" + ptr->d_name); // 可以只保留名字
            files.push_back(ptr->d_name); // 可以只保留名字
        }
    }
    closedir(pDir);
    return true;
}
//
bool cmp(std::string a, std::string b)
{
    int a_frame_id = std::stoi(a.substr(0, a.find('_')));
    int b_frame_id = std::stoi(b.substr(0, b.find('_')));
    return a_frame_id<b_frame_id;
}
//
void picture_stream_pub(const string &data_path)
{
    std::string rgb_path = data_path + "rgb/";
    // std::string tof_path = data_path + "tof/";
    std::vector<std::string> rgb_files_name;
    rgb_files_name.clear();

    // attention! GetFileName func can get pic name , but it is disorder
    if ( ! GetFileName(rgb_path, rgb_files_name)) {
        cerr << "Failed to open file! Please check data path!" << endl;
        return;
    }
    std::cout << "rgb_files_name.size() is :" << rgb_files_name.size() << std::endl;
    // so u better to sort those pic name to ensure the vedio is fluently
    sort(rgb_files_name.begin(), rgb_files_name.end(),cmp);

    // read rgb
    cv::Mat frame;
    for (auto i : rgb_files_name) {
        //printf name
        if(0) {
            std::cout << rgb_path + i << std::endl;
        }
        double time_stamp = (double)std::stoi(i.substr(i.find('_')+1, -1));
        int frame_id = std::stoi(i.substr(0, i.find('_')));
        std::string image_path_tmp = rgb_path + i;
        //
        frame = cv::imread(image_path_tmp, -1);
        if (!frame.empty())
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame).toImageMsg();
            msg->header.stamp = ros::Time::now(); //Todo: should use image time_stamp
            msg->header.frame_id = frame_id;
            pub_img.publish(msg);
            cv::imshow("Frame", frame);
        }
        //20fps //Todo: should use image time_stamp
        int key = cv::waitKey(50); 
        if (key == 'q' || key == 27) {
            cout << "q or esc key is pressed by the user. Stopping the video" << endl;
            break;
        }
    }
}