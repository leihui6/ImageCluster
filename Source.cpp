#include "ImageCluster.h"

using namespace cv;
using namespace std;

int main()
{
	Mat img = imread("sample/photo2.jpg");

	if (!img.data)
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}

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

	image_cluster.init_kernel_size(2, 2);

	image_cluster.cluster(20);

	return 1;
}