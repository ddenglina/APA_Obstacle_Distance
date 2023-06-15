#include "distance_utils.h"

void OnMousePoints(int event, int x, int y, int flags, void *data_ptr)
{
    switch (event)
    {
        case CV_EVENT_LBUTTONDOWN:
            userdata *data = ((userdata *) data_ptr);
            Point p3 = cvPoint(x, y);//获取鼠标坐标点
            circle(data->im, Point(x,y),1,Scalar(0,255,255), 1, cv::LINE_4);
			// cout << "(" << x << ", " << y << ")"<<endl; 
            data->points.emplace_back(x, y);
            imshow("src", data->im);
    }
	// int x_p = srcdata.points[0].x;
	// int y_p = srcdata.points[0].y;
}

void GetPoints(string imgpath,userdata &srcdata)
{
	// 源图像
	srcdata.im = imread(imgpath);
	cv::namedWindow("src");
	imshow("src", srcdata.im);
	waitKey(1);
	setMouseCallback("src", OnMousePoints,&srcdata);
	waitKey(0);
	destroyAllWindows();

	if(srcdata.points.size()==0){
		std::cout << "you need to click ONE point in the show img!!!" << std::endl;
	}
}


Rect boxPhi;
bool g_drawbox = false;
//鼠标交互,画矩形框
void OnMouseRectangle(int event, int x, int y, int flags, void* param)
{
	Mat& image = *(Mat*)param;
	switch (event)
	{
		//按下左键
		case EVENT_LBUTTONDOWN:
		{
			g_drawbox = true;
			boxPhi = Rect(x, y, 0, 0);
		}
		break;
		//鼠标移动
		case EVENT_MOUSEMOVE:
		{
			if (g_drawbox)
			{
				boxPhi.width = x - boxPhi.x;
				boxPhi.height = y - boxPhi.y;
			}
		}
		break;
		//鼠标左键抬起
		case EVENT_LBUTTONUP:
		{
			g_drawbox = false;
			if (boxPhi.width < 0)
			{
				boxPhi.x += boxPhi.width;
				boxPhi.width *= -1;
			}
			if (boxPhi.height < 0)
			{
				boxPhi.y += boxPhi.height;
				boxPhi.height *= -1;
			}
			rectangle(image, boxPhi.tl(), boxPhi.br(), Scalar(0,0,255));
			// 记录矩形框左上角,右下角坐标
			userdata *data = ((userdata *) param);
			data->points.emplace_back(boxPhi.tl());
			data->points.emplace_back(boxPhi.br());
			// cout<<"boxPhi.tl():"<<boxPhi.tl()<<endl;
			// cout<<"boxPhi.br():"<<boxPhi.br()<<endl;
		}
		break;
	}
}

void GetRectangle(string imgpath,cv::Point2f& output)
{
	userdata srcdata;
	srcdata.im  = imread(imgpath);
	// imshow("src", srcdata.im );
	
    Mat src1;
	srcdata.im.copyTo(src1);
	//鼠标交互获取前景区域
	// namedWindow("获取前景区域", WINDOW_AUTOSIZE);
	setMouseCallback("获取前景区域", OnMouseRectangle, &srcdata);

	while (true)
	{
		Mat tempImage;
		src1.copyTo(tempImage);
		if (g_drawbox)rectangle(tempImage, boxPhi.tl(), boxPhi.br(), Scalar(0,0,255));
		imshow("获取前景区域", tempImage);
		if (waitKey(10) == 27)break;
	}

	//--------------------------------
 	//Step1:获取参考点坐标
	output.x = srcdata.points[1].x;
	output.y = int((srcdata.points[1].y + srcdata.points[0].y)/2);
	// cout<<"centerx:"<<output.x<<"  centery:"<<output.y<<endl;
}


void ComputeDistance(cv::Point2f input, cv::Mat cameraMatrix,cv::Mat rotateMatrix,cv::Mat tVec,cv::Point2f& output)
{	
	double s = 0;
	double zConst = 0;//实际坐标系的距离
	cv::Mat imagePoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); //u,v,1
	imagePoint.at<double>(0, 0) = input.x;
	imagePoint.at<double>(1, 0) = input.y;
	
	cv::Mat tempMat = rotateMatrix.inv() * cameraMatrix.inv();
	cv::Mat tempMat1 = tempMat * imagePoint;
	cv::Mat tempMat2 = rotateMatrix.inv() * tVec;
	s = (zConst + tempMat2.at<double>(2, 0))/tempMat1.at<double>(2, 0);
	cv::Mat wcPoint = tempMat1*s - tempMat2;
	output.x = wcPoint.at<double>(0,0);
	output.y = wcPoint.at<double>(1,0);

	// cout<<"wcPoint:"<<wcPoint<<endl;
	// cout<<"tempMat"<<tempMat<<endl;
	// cout<<"tempMat1"<<tempMat1<<endl;
	// cout<<"tempMat2"<<tempMat2<<endl;
	// cout<<"s"<<s<<endl;

}


// void MEITest(cv::Point2f input, cv::Point2f& output)
// {
//     /* const std::string& cameraName,
//                 int w, int h,
//                 double xi,
//                 double k1, double k2, double p1, double p2,
//                 double gamma1, double gamma2, double u0, double v0*/
//     CataCamera camera1("camera", 1280, 720,
//                       0.579600, -0.186600, 0.020100,  -0.000200, 0.000400,
//                       540.051025, 539.656128, 643.645020,  353.622803);
    
   
//     Eigen::Vector2d p(input.x,input.y);

//     Eigen::Vector3d P_est;
//     camera1.liftProjective(p, P_est); // 将有畸变的像素坐标转化为无畸变的单位球面坐标系。
//     // P_est.normalize();

//     //转化为归一化像素坐标
//     output.x = camera1.getParameters().gamma1() * P_est.x() / P_est.z() + camera1.getParameters().u0();
//     output.y = camera1.getParameters().gamma2() * P_est.y() / P_est.z() + camera1.getParameters().v0();

//     cout<<"P_est"<<P_est<<endl;
//     // cout<<"pointx:"<<pointx<<"  pointy:"<<pointy<<endl;
// }



void ProjectPointFromOriginToUndistorted(cv::Matx33d K, cv::Vec4d D, cv::Point2f input, cv::Point2f& output)
{
	float x, y, theta, theta_d, r, x_origin, y_origin;
	float theta_cal[2], d_theta;
 
	x_origin = (input.x - K(0, 2))/K(0, 0);
	y_origin = (input.y - K(1, 2))/K(1, 1);
	theta_d = sqrt(x_origin*x_origin + y_origin*y_origin);
	theta_cal[0] = 0;
	theta_cal[1] = CV_PI / 2;
	d_theta = fabs(theta_cal[1] - theta_cal[0]);
	while (d_theta > 0.01)
	{
		float val[3];
		float middle_theta_cal = 0.5 * (theta_cal[1] + theta_cal[0]);
 
		val[0] = theta_cal[0] + D(0)*pow(theta_cal[0], 3)
			+ D(1)*pow(theta_cal[0], 5) + D(2)*pow(theta_cal[0], 7)
			+ D(3)*pow(theta_cal[0], 9) - theta_d;
 
		val[1] = theta_cal[1] + D(0)*pow(theta_cal[1], 3)
			+ D(1)*pow(theta_cal[1], 5) + D(2)*pow(theta_cal[1], 7)
			+ D(3)*pow(theta_cal[1], 9) - theta_d;
 
		val[2] = middle_theta_cal + D(0)*pow(middle_theta_cal, 3)
			+ D(1)*pow(middle_theta_cal, 5) + D(2)*pow(middle_theta_cal, 7)
			+ D(3)*pow(middle_theta_cal, 9) - theta_d;
 
		if (fabs(val[2]) < 1e-6)
		{
			break;
		}
		if (val[0] * val[2] > 0)
		{
			theta_cal[0] = middle_theta_cal;
		}
		else
		{
			theta_cal[1] = middle_theta_cal;
		}
		d_theta = theta_cal[1] - theta_cal[0];
	}
 
	theta = 0.5 * (theta_cal[1] + theta_cal[0]);
	x = tan(theta) / sqrt(x_origin*x_origin + y_origin*y_origin) * x_origin;
	y = tan(theta) / sqrt(x_origin*x_origin + y_origin*y_origin) * y_origin;
	output.x = x*K(0, 0) + K(0, 2);
	output.y = y*K(1, 1) + K(1, 2);
}