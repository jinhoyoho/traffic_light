#include "traffic_light/traffic.h"

Traffic::Traffic(ros::NodeHandle& nh)
{
    flag = false; // flag false로 생성
    redFlag = false;    // 빨간 신호등을 보았는가요?
    crosswalk = false; // request를 받았나요?
    count = 0;
    image_sub = nh.subscribe("traffic_image", 1, &Traffic::image_callBack, this);
    traffic_server = nh.advertiseService("traffic_srv", &Traffic::go_crosswalk, this);
    std::cout << "Traffic on" << "\n";
}

void Traffic::image_callBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        if(msg)
        {
            frame = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);
            cv::imshow("traffic image", frame);
            if (cv::waitKey(10) == 27) exit(1);  // esc키로 종료
            this->process_image();
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert image! %s", e.what());
        ROS_ERROR("Received image encoding: %s", msg->encoding.c_str());
    }
}


bool Traffic::go_crosswalk(traffic_light::traffic_srv::Request &req, traffic_light::traffic_srv::Response &res)
{
    crosswalk = req.CrossWalk;  // request를 받음
    
    ROS_INFO("Traffic request: %d(1 is True, 0 is False)", crosswalk);

    ros::Time traffic_time = ros::Time::now(); // 40초: 빨간불부터 초록불까지 한 cycle
    ros::Duration cycle_duration(40.0); // 40초

    while(crosswalk)
    {
        if(flag)
        {
            break;
        }

        ros::Time current_time = ros::Time::now(); // 현재 시간

        // 경과 시간 계산
        ros::Duration elapsed_time = current_time - traffic_time;

        // 40초가 지났는지 확인
        if (elapsed_time >= cycle_duration) {
            ROS_INFO("Finish 40seconds");
            break;  // 탈출!
        }

        ros::spinOnce();    // 콜백 처리
        ros::Duration(0.1).sleep(); // CPU 사용을 줄이기 위해 잠시 대기
    }

    res.Go = true;  // 무조건 true 값 전달
    return res.Go;
}

void Traffic::process_image()
{
    try{
        // BGR 이미지를 HSV로 변환
        cv::Mat hsvImage_red;
        cv::Mat hsvImage_green;
        cv::cvtColor(frame, hsvImage_red, cv::COLOR_BGR2HSV);
        cv::cvtColor(frame, hsvImage_green, cv::COLOR_BGR2HSV);

        // 초록색 범위 정의 (HSV)
        cv::Scalar lowerGreen(40, 15, 170); // 낮은 초록색 범위
        cv::Scalar upperGreen(70, 70, 255); // 높은 초록색 범위

        cv::Scalar lowerRed(0, 0, 70); // 낮은 빨간색 범위
        cv::Scalar upperRed(25, 160, 255); // 높은 빨간색 범위

        // 초록색 픽셀 마스크 생성
        cv::Mat greenMask;
        cv::inRange(hsvImage_green, lowerGreen, upperGreen, greenMask);

        cv::Mat redMask;
        cv::inRange(hsvImage_red, lowerRed, upperRed, redMask);


        // 전체 픽셀 개수
        int totalPixelCount = hsvImage_green.rows * hsvImage_green.cols;


        // 초록색 픽셀 수 카운트
        int greenPixelCount = cv::countNonZero(greenMask);
        // 초록색 비율 계산
        double greenRatio = static_cast<double>(greenPixelCount) / totalPixelCount;

        // 빨간색 픽셀 수 카운트
        int redPixelCount = cv::countNonZero(redMask);
        // 빨간색 비율 계산
        double redRatio = static_cast<double>(redPixelCount) / totalPixelCount;

        
        // 초록색이 더 크다면
        if(greenRatio > redRatio)   
        {
            mvf.push_back(GO);  // 출발 
            count++;    // 개수 증가
        }
        else if (redRatio > greenRatio)
        {
            mvf.push_back(STOP);    //  멈춰!
            count++;
        }


        if (count >= 10)    // 10개가 쌓였다면
        {
            double sum = std::accumulate(mvf.begin(), mvf.end(), 0); // 초기값은 0
            sum = sum / count; // 개수만큼 나누기

            mvf.erase(mvf.begin());  // 맨 앞 요소 삭제
            count--;


            if(sum >= 0.5) // 5개만 넘어도 출발
            {
                // 빨간 신호등을 봐야 출발
                if(redFlag && crosswalk)
                {
                    ROS_INFO("GO!");
                    
                    flag = true;        // 출발 플래그 publish 해줘야 함
                    redFlag = false;    // 다시 빨간 신호등을 봐야 출발
                    crosswalk = false;

                    mvf.clear();    // 모든 벡터 지우기
                    count = 0;      // 개수는 0
                }
            }
            else
            {
                redFlag = true;
            }
        }

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not crop image! %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traffic_light");
    ros::NodeHandle nh;

    ROS_INFO("Start traffic light!");

    Traffic tf(nh);

    ros::spin();

    return 0;
}