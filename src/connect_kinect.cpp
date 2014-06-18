#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include <iostream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <opencv/cv.h>

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <vector>

#define WINDOW_NAME "Kinect RGB Image"
using namespace std;
using namespace cv;


void imageCallback (const sensor_msgs::Image::ConstPtr& img);

  Mat queryImg;
vector<KeyPoint> queryKeypoints;
Mat queryDescriptors;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  queryImg = imread("../pics/can.jpg");

  if(queryImg.empty())
    std::cout << "fuck";

  // Detect keypoints in both images.
  SiftFeatureDetector detector(10);
  detector.detect(queryImg, queryKeypoints);

  SiftDescriptorExtractor extractor;
  extractor.compute(queryImg, queryKeypoints, queryDescriptors);


  cv::namedWindow(WINDOW_NAME);

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/camera/rgb/image_color", 4, imageCallback);

  ros::spin();

  cv::destroyWindow(WINDOW_NAME);

  return 0;
}

void imageCallback (const sensor_msgs::Image::ConstPtr& img)
{
  cv_bridge::CvImagePtr cv_ptr;

  try {

        cv_ptr = cv_bridge::toCvCopy(*img, sensor_msgs::image_encodings::BGR8); //enc::RGB8 also used

    } catch (cv_bridge::Exception& e) {

        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat trainImg = cv_ptr->image;
    SiftFeatureDetector detector(10);
    vector<KeyPoint> trainKeypoints;
    detector.detect(trainImg, trainKeypoints);

    SiftDescriptorExtractor extractor;
    Mat trainDescriptors;
    extractor.compute(trainImg, trainKeypoints, trainDescriptors);

    cv::FlannBasedMatcher matcher;
    vector<DMatch> matches;
    matcher.match(queryDescriptors, trainDescriptors, matches);

    Mat img_matches;
    drawMatches(queryImg, queryKeypoints, trainImg, trainKeypoints, matches, img_matches);
    cv::imshow(WINDOW_NAME, img_matches);


    //cv::imshow(WINDOW_NAME, cv_ptr->image);
    cv::waitKey(2);
}