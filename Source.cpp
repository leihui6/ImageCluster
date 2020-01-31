#include "ImageCluster.h"

using namespace cv;
using namespace std;

int main()
{
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
}