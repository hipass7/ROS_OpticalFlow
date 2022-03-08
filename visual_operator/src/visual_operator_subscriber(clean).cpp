#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <sys/stat.h>

using namespace cv;
using namespace std;

static const std::string TOPIC_NAME = "image_raw";

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::imwrite("rgb.bmp", cv_ptr->image);
        cv::imshow("mission #2", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
    }
}

int main(int argc, char** argv)
{
    VideoCapture capture(0);
    if (!capture.isOpened()){
        //error in opening the video input
        cerr << "Unable to open camera!" << endl;
        return 0;
    }
    Mat frame1, prvs;
    capture >> frame1;
    cvtColor(frame1, prvs, COLOR_BGR2GRAY);
    while(true){
        Mat frame2, next;
        capture >> frame2;
        if (frame2.empty())
            break;
        cvtColor(frame2, next, COLOR_BGR2GRAY);
        Mat flow(prvs.size(), CV_32FC2);
        calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
        // visualization
        Mat flow_parts[2];
        split(flow, flow_parts);
        Mat magnitude, angle, magn_norm;
        cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
        normalize(magnitude, magn_norm, 0.0f, 1.0f, NORM_MINMAX);
		Mat oang = angle * 3.141592 / 180.0;
        angle *= ((1.f / 360.f) * (180.f / 255.f));

        //build hsv image
        Mat _hsv[3], hsv, hsv8, bgr;
        _hsv[0] = angle;
        _hsv[1] = Mat::ones(angle.size(), CV_32F);
        _hsv[2] = magn_norm;
        merge(_hsv, 3, hsv);
        hsv.convertTo(hsv8, CV_8U, 255.0);
        cvtColor(hsv8, bgr, COLOR_HSV2BGR);
        imshow("frame2", bgr);

		// representation using vectors
		int step = 10;
		Mat img_vec = frame2;
		float test1 = 0.0, test2 = 0.0;
		for (int r=0; r<angle.rows; r+=step) {
			for (int c=0; c<angle.cols; c+=step){
				float ang = oang.at<float>(r,c);
				float m = magn_norm.at<float>(r,c) * 20.0;
				if (m > 0.5){
					Point pt1 = cv::Point(c, r);
					Point pt2 = cv::Point(c + m * cos(ang) , r + m * sin(ang));
					line(img_vec, pt1, pt2, Scalar(0, 255, 0), 1, 8, 0);
					test1 += m * cos(ang);
					test2 += m * sin(ang);
						}

			}
		}

	if (test2 > test1 && test2 > 3000)
	    ROS_INFO("DOWN");
	else if (test2 < test1 && test2 < -3000)
	    ROS_INFO("UP");
	else if (test1 > test2 && test1 > 3000)
	    ROS_INFO("RIGHT");
	else if (test1 < test2 && test1 < -3000)
	    ROS_INFO("LEFT");

        imshow("mission #2", img_vec);

        int keyboard = waitKey(30);
        if (keyboard == 'q' || keyboard == 27)
            break;
        prvs = next;
    }	

	return 0;	
}


