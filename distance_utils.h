#ifndef __DISTANCE_UTIL_H__
#define __DISTANCE_UTIL_H__

// #include <camodocal/camera_models/CataCamera.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/ccalib/omnidir.hpp>
#include <math.h>
#include <string.h>

#define PI 3.1415926

using namespace std;
using namespace cv;
// using namespace camodocal;

struct userdata{
    Mat im;
    vector<Point2f> points;
};


void ProjectPointFromOriginToUndistorted(cv::Matx33d K, cv::Vec4d D, cv::Point2f input, cv::Point2f& output);

// void MEITest(cv::Point2f input, cv::Point2f& output);

void ComputeDistance(cv::Point2f input, cv::Mat cameraMatrix,cv::Mat rvecM,cv::Mat tvec,cv::Point2f& output);

void OnMousePoints(int event, int x, int y, int flags, void *data_ptr);
void GetPoints(string imgpath,userdata &srcdata);

void OnMouseRectangle(int event, int x, int y, int flags, void* param);
void GetRectangle(string imgpath,cv::Point2f& output);



#endif