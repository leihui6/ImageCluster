#include<opencv2\opencv.hpp>
#include <vector>
#include <corecrt_math_defines.h>
#include "ImageCluster.h"

using namespace cv;
using namespace std;
//
//void get_point_by_radius(cv::Point &p, float radius, float angle, cv::Point & result_p)
//{
//	float radian = M_PI / (180 / angle);
//	float
//		addition_x = radius * cosf(radian),
//		addition_y = radius * sinf(radian);
//
//	result_p.x = p.x + addition_x;
//	result_p.y = p.y - addition_y;
//}
//
//bool is_vertical(cv::Point & p1, cv::Point & p2)
//{
//	if (p1.x * p2.x + p1.y * p2.y == 0)
//	{
//		return true;
//	}
//	return false;
//}
//
//void get_point_along_with_vector_distance(cv::Point &p, cv::Point &e_p, float distance, vector<cv::Point> & result_p)
//{
//	
//	/*
//				 (x2,y2)
//					|
//					|
//					|
//					|
//	[p](x1,y1)------(x0,y0)[e_p]
//	
//	*/
//	float x[2], y[2];
//	float x0, y0, x1, y1;
//	x0 = e_p.x;
//	y0 = e_p.y;
//	x1 = p.x;
//	y1 = p.y;
//
//	float 
//		delta_x1 = x1 - y0,
//		delta_y1 = y1 - y0;
//
//	float d = distance/2;
//
//	x[0] = (delta_x1*x0 + delta_y1 * y0 - delta_y1 * (d*delta_x1* pow((1 / (delta_x1  * delta_x1 + delta_y1 * delta_y1)), (1 / 2)) + (delta_x1  * delta_x1  * y0) / (delta_x1  * delta_x1 + delta_y1 * delta_y1) + (delta_y1  * delta_y1  * y0) / (delta_x1  * delta_x1 + delta_y1 * delta_y1))) / delta_x1;
//	x[1] = (delta_x1*x0 + delta_y1 * y0 - delta_y1 * ((delta_x1  * delta_x1  * y0) / (delta_x1  * delta_x1 + delta_y1 * delta_y1) - d * delta_x1* pow((1 / (delta_x1  * delta_x1 + delta_y1 * delta_y1)), (1 / 2)) + (delta_y1  * delta_y1  * y0) / (delta_x1  * delta_x1 + delta_y1 * delta_y1))) / delta_x1;
//
//	y[0] = d * delta_x1* pow((1 / (delta_x1  * delta_x1 + delta_y1 * delta_y1)), (1 / 2)) + (delta_x1  * delta_x1 *y0) / (delta_x1  * delta_x1 + delta_y1 * delta_y1) + (delta_y1  * delta_y1 *y0) / (delta_x1  * delta_x1 + delta_y1 * delta_y1);
//	y[1] = (delta_x1  * delta_x1 *y0) / (delta_x1  * delta_x1 + delta_y1 * delta_y1) - d * delta_x1* pow((1 / (delta_x1  * delta_x1 + delta_y1 * delta_y1)), (1 / 2)) + (delta_y1  * delta_y1 *y0) / (delta_x1  * delta_x1 + delta_y1 * delta_y1);
//
//	cv::Point p1(delta_x1, delta_y1);
//
//	vector<cv::Point> p2_vec;
//	p2_vec.push_back(cv::Point(x[0] - x0, y[0] - y0));
//	p2_vec.push_back(cv::Point(x[1] - x0, y[1] - y0));
//	p2_vec.push_back(cv::Point(x[1] - x0, y[0] - y0));
//	p2_vec.push_back(cv::Point(x[0] - x0, y[1] - y0));
//
//	for (size_t i = 0; i < 4; ++i)
//	{
//		if (is_vertical(p1, p2_vec[i]))
//		{
//			if (i == 0)
//				result_p.push_back(cv::Point(x[0], y[0]));
//			else if (i == 1)
//				result_p.push_back(cv::Point(x[1], y[1]));
//			else if (i == 2)
//				result_p.push_back(cv::Point(x[1], y[0]));
//			else if (i == 3)
//				result_p.push_back(cv::Point(x[0], y[1]));
//		}
//	}
//}

int main()
{
	//const int triangle_radius = 3;
	//const int hexagon_radius = 70;
	//
	//const int serial_width = 120;
	//const int serial_height = 30;

	Mat img = imread("photo2.jpg");

	cv::resize(img, img,cv::Size(480,640));
	cout << "width=" << img.cols << " height=" << img.rows << endl;
	
	cv::cvtColor(img, img, COLOR_RGB2GRAY);

	Mat img_bin(cv::Size(img.rows, img.cols), CV_8UC1);

	cv::threshold(img, img_bin, 125, 255, cv::THRESH_OTSU);
	bitwise_not(img_bin, img_bin);

	imshow("test", img_bin);
	waitKey(10);

	ImageCluster image_cluster;
	
	image_cluster.load_image(img_bin.data,img_bin.cols,img_bin.rows);

	image_cluster.init_kernel_size(4, 4);

	image_cluster.cluster();

	return 1;
	//vector<cv::Point> points;
	//for (int y = 0; y < img_bin.rows; y++) {
	//	for (int x = 0; x < img_bin.cols; x++) {
	//		if (255 == img_bin.at<uchar>(y, x))
	//		{
	//			points.push_back(cv::Point(x, y));	
	//		}
	//	}
	//}
	//RotatedRect minRect = cv::minAreaRect(points);
	//Point2f vertex[4];
	//minRect.points(vertex);

	//std::vector<cv::Point> vertex_vec;
	//vertex_vec.assign(vertex, vertex + 4);

	//Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	//cv::erode(img_bin, img_bin, kernel);

	//bitwise_not(img_bin, img_bin);

	//// 计算最小包围和中心点
	//cv::Moments center = cv::moments(vertex_vec, true);
	//cv::Point center_point(center.m10 / center.m00, center.m01 / center.m00);

	//// 测试半径长度
	//cv::circle(img_bin, center_point, hexagon_radius, Scalar(0));

	//cv::Point p;
	//vector<cv::Point> p_vec;
	//for (float a = 1; a < 360; ++a)
	//{
	//	get_point_by_radius(center_point, hexagon_radius, a, p);

	//	get_point_along_with_vector_distance(center_point, p, serial_width/2, p_vec);

	//	cv::circle(img, p, 3, Scalar(255), -1);
	//	cv::circle(img, p_vec[0], 4, Scalar(255), -1);
	//	cv::circle(img, p_vec[1], 4, Scalar(255), -1);
	//	break;
	//}
	//
	//for (int i = 0; i < 4; i++)
	//	line(img_bin, vertex[i], vertex[(i + 1) % 4], Scalar::all(0), 1, 8);
	//imshow("TEST", img);
	//imshow("TEST_bin", img_bin);
	//
	//waitKey();
}