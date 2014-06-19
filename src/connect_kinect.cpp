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
#include <string>

#define WINDOW_NAME "Kinect RGB Image"
#define NP 40
using namespace std;
using namespace cv;

bool computeKeyPoints(string filename);
void imageCallback (const sensor_msgs::Image::ConstPtr& img);
bool changeDetectorExtractor(string type);

cv::Ptr<cv::FeatureDetector> detector;
cv::Ptr<cv::DescriptorExtractor> extractor;

Mat queryImg;
vector<KeyPoint> queryKeypoints;
Mat queryDescriptors;
vector<string> filenames;
int currentfile = 0;
bool PAUSE = false;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  for(int i = 1; i < argc; i++)
  {
    string x("../pics/longhammer/");
    filenames.push_back(x+argv[i]);
  }
  cv::initModule_nonfree();
  changeDetectorExtractor("SIFT");

  if(!computeKeyPoints(filenames[currentfile]))
  {
    std::cout << "error loading file";
    return -1;
  }

  cv::namedWindow(WINDOW_NAME);

  ros::NodeHandle n;

  // subscribe to the kinect camera publisher, only grab the RGB coordinates
  ros::Subscriber sub = n.subscribe("/camera/rgb/image_color", 4, imageCallback);
;
  ros::spin();

  cv::destroyWindow(WINDOW_NAME);

  return 0;
}

bool changeDetectorExtractor(string type)
{
  detector = new cv::SurfFeatureDetector(4000,8, 6, true, true);//cv::FeatureDetector::create(type);
  if(detector==NULL)
    std::cout<<"detector NULL"<<std::endl;

  extractor = cv::DescriptorExtractor::create("SIFT");
  if(extractor==NULL){std::cout<<"descriptorExtractor NULL"<<std::endl;}
}

bool computeKeyPoints(string filename)
{
  queryImg = imread(filename);

  if(queryImg.empty())
  {
    return false;
  }

  // Detect keypoints in both images.
  //SiftFeatureDetector detector(NP);
  detector->detect(queryImg, queryKeypoints);

  //SiftDescriptorExtractor extractor;
  extractor->compute(queryImg, queryKeypoints, queryDescriptors);

  return true;
}

void imageCallback (const sensor_msgs::Image::ConstPtr& img)
{
  cv_bridge::CvImagePtr cv_ptr;

  if(!PAUSE)
  {
    try 
      {
        cv_ptr = cv_bridge::toCvCopy(*img, sensor_msgs::image_encodings::BGR8); //enc::RGB8 also used
      }
      catch (cv_bridge::Exception& e) 
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      cv::Mat trainImg = cv_ptr->image;

      vector<KeyPoint> trainKeypoints;
      detector->detect(trainImg, trainKeypoints);

      //SiftDescriptorExtractor extractor;
      Mat trainDescriptors;
      extractor->compute(trainImg, trainKeypoints, trainDescriptors);

      cv::FlannBasedMatcher matcher (new cv::flann::KDTreeIndexParams(2), new cv::flann::SearchParams(32));
      vector<DMatch> matches;
      matcher.match(queryDescriptors, trainDescriptors, matches);

      Mat img_matches;
      drawMatches(queryImg, queryKeypoints, trainImg, trainKeypoints, matches, img_matches);
      if(!img_matches.empty())
        cv::imshow(WINDOW_NAME, img_matches);
  }

   
    char key = cv::waitKey(2);
    switch(key)
    {
      case 'n' :
        currentfile++;
        if(currentfile == filenames.size())
          currentfile = 0;

        if(!computeKeyPoints(filenames[currentfile]))
          ROS_ERROR("Could not load new file in image callback");
          return;
      break;

      case ' ' :
       PAUSE =! PAUSE;

      break;
    }
}

