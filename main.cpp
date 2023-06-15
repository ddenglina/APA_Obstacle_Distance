/*已知内外参,计算距离*/

#include "distance_utils.h"

// static string INTRINC_PATH="output_config/KB_intrinc.yml";
// static string EXTRINC_PATH = "output_config/KB_extrinc.yml";
static string INTRINC_PATH="../output_config/mei_intrinc.yaml";
static string EXTRINC_PATH = "../output_config/mei_extrinc.yaml";

int main()
{   
    int imgnum = 10;
    for(int i=9;i<imgnum;i++)
    {
        string IMG_raw="/home/denglina/Downloads/data/2023-6-3/rawdata/useimg53/front_person_"+to_string(i+1)+".jpg";
        cout << "i:"<<i+1<<endl;

        // Step1: get pixcel point
        userdata srcdata;
        GetPoints(IMG_raw,srcdata);
        int x_p = srcdata.points[0].x; 
        int y_p = srcdata.points[0].y; 
        cout<<"pointx:"<<x_p<<"  pointy:"<<y_p<<endl;

        // Step2：read yml, and get camera args
        cv::Mat cameraMatrix,distCoeffs,Xi;
        cv::FileStorage fs(INTRINC_PATH,cv::FileStorage::READ);
        fs["cameraMatrix0"] >> cameraMatrix;
        fs["distCoeffs0"] >> distCoeffs;
        fs["xi0"] >> Xi;
        // it's type is CV_32FC1 in .so lib, so make a convert
        cameraMatrix.convertTo(cameraMatrix, CV_64F);
        distCoeffs.convertTo(distCoeffs, CV_64F);
        Xi.convertTo(Xi, CV_64F);
        fs.release();

        cv::Mat rotateMatrix,tVec;
        cv::FileStorage fs1(EXTRINC_PATH,cv::FileStorage::READ);
        fs1["rotateMatrix0"] >> rotateMatrix;
        fs1["tVec0"] >> tVec;
        rotateMatrix.convertTo(rotateMatrix, CV_64F);
        tVec.convertTo(tVec, CV_64F);
        fs1.release();

        // // Step3: undistort point
        // // 方案1: KB
        // // cv::Point2f inPoint,outPoint;
        // // inPoint.x = x_p;
        // // inPoint.y = y_p;
        // // ProjectPointFromOriginToUndistorted(cameraMatrix,distCoeffs,inPoint,outPoint); 
        // // 
        // // MEITest(inPoint,outPoint); //方案2: MEI, camodocal库实现的

        // // 方案2: MEI, Opencv扩展库实现的库实现的
        cv::Mat inPoint(1,2,CV_64FC2),outPoint_;
        inPoint.at<double>(0, 0) = x_p; 
        inPoint.at<double>(0, 1) = y_p;
        cv::omnidir::undistortPoints(inPoint,outPoint_,cameraMatrix,distCoeffs,Xi,cv::Mat()); // MEI, Opencv扩展库实现的,无畸变的单位球面坐标
        cv::Point2f outPoint;
        outPoint.x = outPoint_.at<double>(0, 0);
        outPoint.y = outPoint_.at<double>(0, 1);
        outPoint.x = cameraMatrix.at<double>(0,0) * outPoint.x + cameraMatrix.at<double>(0,2);
        outPoint.y = cameraMatrix.at<double>(1,1) * outPoint.y + cameraMatrix.at<double>(1,2);
        cout<<"undistort pointx:"<<outPoint.x<<"  pointy:"<<outPoint.y<<endl;

        //Step4: get distance
        cv::Point2f wcPoint;
        ComputeDistance(outPoint,cameraMatrix,rotateMatrix,tVec,wcPoint);
        cout<<"X:"<<wcPoint.x<<"  Y:"<<wcPoint.y<<"  Z:0"<<endl;
        cout<<"---------------------------------------"<<endl;
        // waitKey(0);
    }
}