#include <ros/ros.h>
#include "traffic_light/traffic_srv.h"
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <numeric> // std::accumulate

#define STOP 0
#define GO 1

class Traffic
{
    private:
        bool flag;  // 출발 플래그
        bool redFlag; // 빨간색을 보았는가
        bool crosswalk; // 횡단보도인지 아닌지 -> request로 받음
        cv::Mat frame; // 이미지
        ros::Subscriber image_sub;
        std::vector<int> mvf; // 이동 평균 필터 10개만
        int count;  // 이동 평균 필터 개수
        ros::ServiceServer traffic_server;

    public:
        Traffic(ros::NodeHandle& nh); // 생성자

        void image_callBack(const sensor_msgs::ImageConstPtr& msg);
        bool go_crosswalk(traffic_light::traffic_srv::Request &req, traffic_light::traffic_srv::Response &res);
        void process_image();
};